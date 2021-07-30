#![no_std]
#![feature(
    test,
    start,
    array_map,
    const_panic,
    isa_attribute,
    core_intrinsics,
    maybe_uninit_ref,
    bindings_after_at,
    stmt_expr_attributes,
    default_alloc_error_handler,
    const_fn_floating_point_arithmetic,
)]

extern crate alloc;

mod gfx;
mod heap;
mod mem;

use core::fmt::Write;
use alloc::{vec::Vec, vec};
use vek::*;
use num_traits::float::Float;
use gba::{
    io::{
        irq::{set_irq_handler, IrqFlags, IrqEnableSetting, IE, IME, BIOS_IF},
        display::{
            DisplayControlSetting, DisplayStatusSetting, DisplayMode,
            DISPCNT, DISPSTAT, VCOUNT, VBLANK_SCANLINE,
        },
        background::{BackgroundControlSetting, BG2HOFS},
        timers::{TimerControlSetting, TimerTickRate, TM2CNT_H, TM2CNT_L},
        keypad::read_key_input,
    },
    bios,
    vram::bitmap::{Mode3, Mode5},
    Color,
};
pub use mem::*;

pub type F32 = fixed::types::I16F16;

pub const fn num(x: f32) -> F32 {
    use fixed::traits::Fixed;
    F32::from_bits((x * (1 << F32::FRAC_NBITS) as f32) as <F32 as Fixed>::Bits)
}

fn normalize_quat_fast(q: Quaternion<F32>) -> Quaternion<F32> {
    fn finvsqrt(x: f32) -> f32 {
        let y = f32::from_bits(0x5f375a86 - (x.to_bits() >> 1));
        y * (1.5 - ( x * 0.5 * y * y ))
    }
    fn fsqrt(x: f32) -> f32 {
        f32::from_bits((x.to_bits() + (127 << 23)) >> 1)
    }
    let v = q.into_vec4();
    (v * F32::from_num(finvsqrt(v.magnitude_squared().to_num::<f32>()))).into()
}

fn cos_fast(mut x: F32) -> F32 {
    use core::f32;
    x *= num(f32::consts::FRAC_1_PI / 2.0);
    x -= num(0.25) + (x + num(0.25)).floor();
    x *= num(16.0) * (x.abs() - num(0.5));
    x += num(0.225) * x * (x.abs() - num(1.0));
    x
}

fn sin_fast(x: F32) -> F32 {
    use core::f32;
    cos_fast(x - num(f32::consts::PI / 2.0))
}

fn tan_fast(x: F32) -> F32 {
    sin_fast(x) / cos_fast(x)
}

fn rotation_3d(angle_radians: F32, axis: Vec3<F32>) -> Quaternion<F32> {
    // let axis = axis.normalized();
    let Vec3 { x, y, z } = axis * sin_fast(angle_radians * num(0.5));
    let w = cos_fast(angle_radians * num(0.5));
    Quaternion { x, y, z, w }
}

#[repr(transparent)]
#[derive(Copy, Clone)]
struct NumWrap(F32);

impl core::ops::Mul<NumWrap> for NumWrap {
    type Output = NumWrap;

    fn mul(self, rhs: Self) -> Self { NumWrap(self.0 * rhs.0) }
}

impl vek::ops::MulAdd<NumWrap, NumWrap> for NumWrap {
    type Output = NumWrap;

    fn mul_add(self, mul: NumWrap, add: NumWrap) -> NumWrap {
        NumWrap(self.0 * mul.0 + add.0)
    }
}

fn apply(m: Mat3<F32>, n: Mat3<F32>) -> Mat3<F32> {
    (m.map(NumWrap) * n.map(NumWrap)).map(|e| e.0)
}

fn apply4(m: Mat4<F32>, n: Mat4<F32>) -> Mat4<F32> {
    (m.map(NumWrap) * n.map(NumWrap)).map(|e| e.0)
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    gba::error!("Panic: {:?}", info);
    Mode3::clear_to(Color::from_rgb(0xFF, 0, 0));
    loop {}
}

#[start]
fn main(_argc: isize, _argv: *const *const u8) -> isize {
    heap::init();

    gba::info!("Starting...");

    set_irq_handler(irq_handler);
    IME.write(IrqEnableSetting::IRQ_YES);

    DISPSTAT.write(DisplayStatusSetting::new()
        .with_hblank_irq_enable(true)
        .with_vblank_irq_enable(true));

    TM2CNT_H.write(TimerControlSetting::new()
        .with_tick_rate(TimerTickRate::CPU1024)
        .with_enabled(true));

    let model = wavefront::Obj::from_lines(include_str!("../data/ship-small.obj").lines()).unwrap();
    let mut ship_verts = Vec::new();
    let mut ship_tris = Vec::new();
    for &p in model.positions() {
        ship_verts.push(Vec3::<f32>::from(p).map(num));
    }
    model
        .triangles()
        .for_each(|vs| {
            let pos = vs.map(|v| Vec3::<f32>::from(v.position()));
            let cross = (pos[1] - pos[0]).cross(pos[2] - pos[0]);
            ship_tris.push((
                (cross / micromath::F32Ext::sqrt(cross.magnitude_squared())).map(num),
                vs.map(|v| v.position_index() as u16),
            ));
        });

    gba::info!("Model has {} vertices and {} triangles", ship_verts.len(), ship_tris.len());

    let mut pos = Vec3::new(0.0, 0.0, 3.0).map(num);
    let mut ori = normalize_quat_fast(Quaternion::<F32>::identity());

    let mut tick = 0;
    let mut last_time = 0;
    let mut sum_fps = 0.0;

    let mut screen = unsafe { gfx::mode5::init() };
    let mut scene = unsafe { gfx::scene::init() };

    let mut time_mvp = 0;
    let mut time_clear = 0;
    let mut time_model = 0;
    let mut time_vertices = 0;
    let mut time_faces = 0;
    let mut time_render = 0;

    loop {
        let new_time = TM2CNT_L.read();
        if tick % 32 == 0 {
            if new_time > last_time {
                gba::info!("FPS: {}", sum_fps / 32.0);
                gba::info!(
                    "Timings: {{ mvp = {}, clear = {}, model = {}, vertices = {}, faces = {}, render = {} }}",
                    time_mvp,
                    time_clear,
                    time_model,
                    time_vertices,
                    time_faces,
                    time_render,
                );
            }
            sum_fps = 0.0;
        }
        let fps = (16_780_000.0 / (new_time - last_time) as f32) / 1024.0;
        sum_fps += fps;
        last_time = new_time;

        let dt = num(fps).recip();

        // Wait for vblank
        IE.write(IrqFlags::new().with_vblank(true));
        // bios::vblank_interrupt_wait();

        screen.flip();

        let keys = read_key_input();

        time_mvp = gba::time_this01! {{
            ori = normalize_quat_fast(ori
                * rotation_3d(
                    if keys.down() { num(4.0) * dt } else { num(0.0) }
                    - if keys.up() { num(4.0) * dt } else { num(0.0) },
                    Vec3::unit_x(),
                )
                * rotation_3d(
                    if keys.right() { num(4.0) * dt } else { num(0.0) }
                    - if keys.left() { num(4.0) * dt } else { num(0.0) },
                    Vec3::unit_y(),
                )
                * rotation_3d(
                    if keys.r() { num(4.0) * dt } else { num(0.0) }
                    - if keys.l() { num(4.0) * dt } else { num(0.0) },
                    Vec3::unit_z(),
                ));

            pos += gfx::scene::transform_pos(Mat4::from(ori).transposed(), Vec3::unit_z() * (
                if keys.a() { num(0.05) } else { num(0.0) }
                    - if keys.b() { num(0.05) } else { num(0.0) }
            )).xyz();
        }};

        let mut fb = screen.back();

        time_clear = gba::time_this01! {{
            fb.clear(Color::from_rgb(1, 3, 4).0);
        }};

        fn perspective_fov_rh_zo(fov_y_radians: F32, width: F32, height: F32, near: F32, far: F32) -> Mat4<F32> {
            let rad = fov_y_radians;
            let h = cos_fast(rad * num(0.5)) / sin_fast(rad * num(0.5));
            let w = h * height / width;

            let m00 = w;
            let m11 = h;
            let m22 = -(far + near) / (far - near);
            let m23 = -(num(2.0) * far * near) / (far - near);
            let m32 = -num(1.0);
            let mut m = Mat4::new(
                m00, num(0.0), num(0.0), num(0.0),
                num(0.0), m11, num(0.0), num(0.0),
                num(0.0), num(0.0), m22, m23,
                num(0.0), num(0.0), m32, num(0.0)
            );
            m
        }

        let proj = perspective_fov_rh_zo(num(1.0), num(fb.screen_size().x as f32), num(fb.screen_size().y as f32), num(0.5), num(256.0));

        let mut frame = scene.begin_frame(gfx::scene::SceneState {
            proj,
            view: Mat4::identity(),
            light_dir: Vec3::new(0.0, -1.0, 0.0).normalized().map(num),
            ambiance: num(0.2),
            light_col: Rgb::new(1.0, 0.0, 0.5).map(num),
        });

        let mut ship_model;
        time_model = gba::time_this01! {{
            ship_model = frame.add_model(apply4(Mat4::translation_3d(pos), Mat4::from(apply(Mat3::from(ori), Mat3::scaling_3d(num(0.2))))));
        }};

        time_vertices = gba::time_this01! {{
            for &v in &ship_verts {
                frame.add_vert(ship_model, v);
            }
        }};

        time_faces = gba::time_this01! {{
            for &(norm, indices) in &ship_tris {
                let color = Rgb::new(1.0, 1.0, 1.0).map(num);

                let verts = [
                    indices[0],
                    indices[1],
                    indices[2],
                ];
                frame.add_convex(ship_model, (verts, color), norm);
            }
        }};

        // frame.add_flat_quad(
        //     0,
        //     ([
        //         Vec3::new(-0.3, -0.5, 0.0).map(num),
        //         Vec3::new(0.0, 1.0, 0.0).map(num),
        //         Vec3::new(0.8, 0.8, 0.0).map(num),
        //         Vec3::new(1.0, 0.0, 0.0).map(num),
        //     ], Rgb::broadcast(num(1.0))),
        //     -Vec3::unit_z(),
        // );

        time_render = gba::time_this01! {{
            frame.render(fb);
        }};

        tick += 1;
    }
}

extern "C" fn irq_handler(flags: IrqFlags) {
    if flags.vblank() {
        vblank_handler();
    }
    if flags.hblank() {
        hblank_handler();
    }
    if flags.vcounter() {
        vcounter_handler();
    }
    if flags.timer0() {
        timer0_handler();
    }
    if flags.timer1() {
        timer1_handler();
    }
}

fn vblank_handler() { BIOS_IF.write(BIOS_IF.read().with_vblank(true)); }
fn hblank_handler() { BIOS_IF.write(BIOS_IF.read().with_hblank(true)); }
fn vcounter_handler() { BIOS_IF.write(BIOS_IF.read().with_vcounter(true)); }
fn timer0_handler() { BIOS_IF.write(BIOS_IF.read().with_timer0(true)); }
fn timer1_handler() { BIOS_IF.write(BIOS_IF.read().with_timer1(true)); }

#[no_mangle]
pub unsafe extern fn __truncdfsf2() {}

// #[no_mangle]
// pub unsafe extern "C" fn memcpy(dst: *mut u8, src: *const u8, n: usize) -> *mut u8 {
//     mem::copy_fast(
//         core::slice::from_raw_parts(src, n),
//         core::slice::from_raw_parts_mut(dst, n),
//     );
//     dst
// }
