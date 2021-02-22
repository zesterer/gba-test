#![no_std]
#![feature(
    start,
    default_alloc_error_handler,
    array_map,
    isa_attribute,
    core_intrinsics,
    bindings_after_at,
)]

mod gfx;
mod heap;
mod mem;

use core::fmt::Write;
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
        timers::{TimerControlSetting, TimerTickRate, TM0CNT_H, TM0CNT_L},
        keypad::read_key_input,
    },
    bios,
    vram::bitmap::{Mode3, Mode5},
    Color,
};
pub use mem::*;

pub type F32 = fixed::types::I16F16;

pub fn fp(x: f32) -> F32 { F32::from_num(x) }

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    gba::error!("Panic: {:?}", info);
    Mode3::clear_to(Color::from_rgb(0xFF, 0, 0));
    loop {}
}

#[start]
fn main(_argc: isize, _argv: *const *const u8) -> isize {
    heap::init();

    let vertices = [
        Vec3::new(-1.0, -1.0, -1.0),
        Vec3::new(-1.0, -1.0,  1.0),
        Vec3::new(-1.0,  1.0, -1.0),
        Vec3::new(-1.0,  1.0,  1.0),
        Vec3::new( 1.0, -1.0, -1.0),
        Vec3::new( 1.0, -1.0,  1.0),
        Vec3::new( 1.0,  1.0, -1.0),
        Vec3::new( 1.0,  1.0,  1.0),
    ].map(|v| v.map(fp));

    const INDICES: &[usize] = &[
        0, 2, 3, 0, 3, 1, // -x
        4, 7, 6, 4, 5, 7, // +x
        0, 5, 4, 0, 1, 5, // -y
        2, 6, 7, 2, 7, 3, // +y
        0, 4, 6, 0, 6, 2, // -z
        1, 7, 5, 1, 3, 7, // +z
    ];

    let norms = [
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ].map(|v| v.map(fp));

    gba::info!("Starting...");

    set_irq_handler(irq_handler);
    IME.write(IrqEnableSetting::IRQ_YES);

    DISPSTAT.write(DisplayStatusSetting::new()
        .with_hblank_irq_enable(true)
        .with_vblank_irq_enable(true));

    TM0CNT_H.write(TimerControlSetting::new()
        .with_tick_rate(TimerTickRate::CPU1024)
        .with_enabled(true));

    let mut ori = Quaternion::<f32>::from_xyzw(0.25, 0.5, 0.25, 0.25).normalized();

    let light_dir = Vec3::new(1.0, -1.0, -1.0).normalized().map(fp);

    let mut tick = 0;
    let mut last_time = 0;
    let mut fps = 0.0;

    let mut screen = unsafe { gfx::mode5::init() };

    loop {
        let new_time = TM0CNT_L.read();
        if tick % 32 == 0 {
            if new_time > last_time {
                gba::info!("FPS: {}", fps / 32.0);
            }
            fps = 0.0;
        }
        fps += (16_780_000.0 / (new_time - last_time) as f32) / 1024.0;
        last_time = new_time;

        // Wait for vblank
        IE.write(IrqFlags::new().with_vblank(true));
        // bios::vblank_interrupt_wait();

        screen.flip();

        // use gba::vram::bitmap::Page;
        // let page = if screen.fb_index() == 0 { Page::Zero } else { Page::One };
        let mut fb = screen.back();

        fb.clear(0x0000);
        let col = if tick % 2 == 0 { 0 } else { 0xFFFF };
        // for i in 0..1_000 {
        //     unsafe { fb.line_unchecked(Vec2::new(16 + i & 31, 16), Vec2::new(24, 20 + (i + 16) & 31), col) };
        //     // fb.line(Vec2::new(16 + i & 31, 16), Vec2::new(24, 20 + (i + 16) & 31), col);
        //     // gba::vram::bitmap::Mode5::draw_line(page, 16 + i & 31, 16, 24, 20 + (i + 16) & 31, Color::from_rgb(col, col, col));
        // }

        let keys = read_key_input();

        fn rotation_3d(angle_radians: f32, axis: Vec3<f32>) -> Quaternion<f32> {
            let axis = axis.normalized();
            let Vec3 { x, y, z } = axis * micromath::F32Ext::sin(angle_radians / 2.0);
            let w = micromath::F32Ext::cos(angle_radians / 2.0);
            Quaternion { x, y, z, w }
        }

        ori = (ori
            * rotation_3d(
                if keys.up() { 0.1 } else { 0.0 }
                - if keys.down() { 0.1 } else { 0.0 },
                Vec3::unit_x(),
            )
            * rotation_3d(
                if keys.right() { 0.1 } else { 0.0 }
                - if keys.left() { 0.1 } else { 0.0 },
                Vec3::unit_y(),
            )
            * rotation_3d(
                if keys.b() { 0.1 } else { 0.0 }
                - if keys.a() { 0.1 } else { 0.0 },
                Vec3::unit_z(),
            )
        ).normalized();

        let mvp = Mat3::from(ori).map(fp);

        // fb.convex(&[Vec2::new(5, 5), Vec2::new(20, 80), Vec2::new(30, 90), Vec2::new(50, 32)], 0xFF);

        // for i in 0..15_0 {
        //     let i = i % 122;
        //     let offset = Vec2::new(i % 16, (i / 16) % 16) * 16;
        //     fb.tri(&[Vec2::new(0, 0) + offset, Vec2::new(16, 16) + offset, Vec2::new(0, 16) + offset], 0xFFFF);
        // }

        let center = fb.size() / 2;

        let mut cube = |offset: Vec2<F32>, scale: F32| {
            // Quad
            for (i, indices) in INDICES.chunks(6).enumerate() {
                let norm = norms[i];

                // Cheap backface culling using normal to skip unnecessary work
                if mvp.cols.z.dot(norm) > fp(0.0) {
                    continue;
                }

                let mul = |m: Mat3<_>, v: Vec3<_>| Vec3 {
                    x: m.cols.x.dot(v),
                    y: m.cols.y.dot(v),
                    z: m.cols.z.dot(v),
                };

                let wnorm = mul(mvp, norm);
                let light = wnorm.dot(light_dir).max(fp(0.25));

                let rgb = norm.map(|e| ((e * fp(15.0) + fp(17.0)) * light).to_num::<u16>() % 32);
                let rgb = Color::from_rgb(rgb.x, rgb.y, rgb.z).0;

                // Polygon
                for indices in indices.chunks(3) {
                    let mul_xy = |m: Mat3<_>, v: Vec3<_>| Vec2 {
                        x: m.cols.x.dot(v),
                        y: m.cols.y.dot(v),
                    };

                    let verts = [
                        (mul_xy(mvp, vertices[indices[0]]) * scale * fp(36.0) + offset).map(|e| e.to_num()),
                        (mul_xy(mvp, vertices[indices[1]]) * scale * fp(36.0) + offset).map(|e| e.to_num()),
                        (mul_xy(mvp, vertices[indices[2]]) * scale * fp(36.0) + offset).map(|e| e.to_num()),
                    ];

                    fb.tri(&verts, rgb);
                }
            }
        };

        cube(center.map(F32::from_num), fp(1.0));
        // cube((-Vec2::unit_x() - Vec2::unit_y()) * fp(32.0), fp(0.5));
        // cube((Vec2::unit_x() - Vec2::unit_y()) * fp(32.0), fp(0.5));
        // cube((-Vec2::unit_x() + Vec2::unit_y()) * fp(32.0), fp(0.5));
        // cube((Vec2::unit_x() + Vec2::unit_y()) * fp(32.0), fp(0.5));

        tick += 1;
    }

    loop {}
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
