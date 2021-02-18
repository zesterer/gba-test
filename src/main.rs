#![no_std]
#![feature(start, default_alloc_error_handler, array_map)]

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

pub type F32 = fixed::types::I20F12;

mod gfx;
mod mem;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    gba::error!("Panic: {:?}", info);
    Mode3::clear_to(Color::from_rgb(0xFF, 0, 0));
    loop {}
}

#[start]
fn main(_argc: isize, _argv: *const *const u8) -> isize {
    mem::init();

    const R: Rgba<f32> = Rgba::new(1.0, 0.0, 0.0, 1.0);
    const Y: Rgba<f32> = Rgba::new(1.0, 1.0, 0.0, 1.0);
    const G: Rgba<f32> = Rgba::new(0.0, 1.0, 0.0, 1.0);
    const B: Rgba<f32> = Rgba::new(0.0, 0.0, 1.0, 1.0);

    let vertices = [
        Vec3::new(-1.0, -1.0, -1.0).map(F32::from_num),
        Vec3::new(-1.0, -1.0,  1.0).map(F32::from_num),
        Vec3::new(-1.0,  1.0, -1.0).map(F32::from_num),
        Vec3::new(-1.0,  1.0,  1.0).map(F32::from_num),
        Vec3::new( 1.0, -1.0, -1.0).map(F32::from_num),
        Vec3::new( 1.0, -1.0,  1.0).map(F32::from_num),
        Vec3::new( 1.0,  1.0, -1.0).map(F32::from_num),
        Vec3::new( 1.0,  1.0,  1.0).map(F32::from_num),
    ];

    const INDICES: &[usize] = &[
        0, 3, 2, 0, 1, 3, // -x
        7, 4, 6, 5, 4, 7, // +x
        5, 0, 4, 1, 0, 5, // -y
        2, 7, 6, 2, 3, 7, // +y
        0, 6, 4, 0, 2, 6, // -z
        7, 1, 5, 3, 1, 7, // +z
    ];

    let norms = [
        Vec3::new(-1.0, 0.0, 0.0).map(F32::from_num),
        Vec3::new(1.0, 0.0, 0.0).map(F32::from_num),
        Vec3::new(0.0, -1.0, 0.0).map(F32::from_num),
        Vec3::new(0.0, 1.0, 0.0).map(F32::from_num),
        Vec3::new(0.0, 0.0, -1.0).map(F32::from_num),
        Vec3::new(0.0, 0.0, 1.0).map(F32::from_num),
    ];

    gba::info!("Starting...");

    set_irq_handler(irq_handler);
    IME.write(IrqEnableSetting::IRQ_YES);

    DISPSTAT.write(DisplayStatusSetting::new()
        .with_hblank_irq_enable(true)
        .with_vblank_irq_enable(true));

    TM0CNT_H.write(TimerControlSetting::new()
        .with_tick_rate(TimerTickRate::CPU1024)
        .with_enabled(true));

    let mut ori = Quaternion::<f32>::identity();

    let light_dir = Vec3::new(1.0, -1.0, -1.0).normalized().map(F32::from_num);

    let mut tick = 0;
    let mut last_time = 0;
    let mut fps = 0.0;
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

        unsafe {
            let bg2 = 0x4000000 as *mut u16;
            // Swap axes and fill
            // bg2.offset(0x10).write_volatile(0);
            // bg2.offset(0x11).write_volatile(259);
            // bg2.offset(0x12).write_volatile(137);
            // bg2.offset(0x13).write_volatile(0);

            // Stretch fill
            bg2.offset(0x10).write_volatile(171);
            bg2.offset(0x13).write_volatile(207);
        };

        // Wait for vblank
        IE.write(IrqFlags::new().with_hblank(true).with_vblank(true));
        bios::vblank_interrupt_wait();
        IE.write(IrqFlags::new().with_hblank(true).with_vblank(true));

        // Set up display
        DISPCNT.write(DisplayControlSetting::new()
            .with_mode(DisplayMode::Mode5)
            .with_frame1(!gfx::flip())
            .with_bg2(true));

        // Clear back buffer
        use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA2 as DMA};
        unsafe {
            #[repr(align(16))]
            struct DmaData([u16; 2]);
            static mut COL: DmaData = DmaData([0; 2]);
            COL.0 = [Color::from_rgb(70, 80, 120).0; 2];
            DMA::set_source(&COL as *const _ as *const _);
            DMA::set_dest(gfx::row_ptr_flip(0, false) as *mut _ as *mut _);
            DMA::set_count((gfx::SIZE.product() / 2) as u16);
            DMA::set_control(DMAControlSetting::new()
                .with_source_address_control(DMASrcAddressControl::Fixed)
                .with_dest_address_control(DMADestAddressControl::Increment)
                .with_dma_repeat(false)
                .with_use_32bit(true)
                .with_start_time(DMAStartTiming::Immediate)
                .with_enabled(true));
        }

        let angle_radians = tick as f32 / 30.0;
        let c = micromath::F32Ext::cos(angle_radians);
        let s = micromath::F32Ext::sin(angle_radians);

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
                if keys.a() { 0.1 } else { 0.0 }
                - if keys.b() { 0.1 } else { 0.0 },
                Vec3::unit_z(),
            )
        ).normalized();

        let mvp = Mat3::from(ori).map(F32::from_num);

        let cube = |offset: Vec2<F32>| {
            for (i, indices) in INDICES.chunks(3).enumerate() {
                let norm = norms[i / 2];

                // Cheap backface culling using normal to skip unnecessary work
                if mvp.cols.z.dot(norm) > F32::from_num(0.0) {
                    continue;
                }

                let mul = |m: Mat3<_>, v: Vec3<_>| Vec3 {
                    x: m.cols.x.dot(v),
                    y: m.cols.y.dot(v),
                    z: m.cols.z.dot(v),
                };

                let a = (mul(mvp, vertices[indices[0]]).xy() + offset) * F32::from_num(36);
                let b = (mul(mvp, vertices[indices[1]]).xy() + offset) * F32::from_num(36);
                let c = (mul(mvp, vertices[indices[2]]).xy() + offset) * F32::from_num(36);

                let wnorm = mul(mvp, norm);
                let light = wnorm.dot(light_dir).max(F32::from_num(0.25));

                let rgb = norm.map(|e| ((e * F32::from_num(15.0) + F32::from_num(17.0)) * light).to_num::<u16>() % 32);

                gfx::tri(a.map(|e| e.to_num()), c.map(|e| e.to_num()), b.map(|e| e.to_num()), Color::from_rgb(rgb.x, rgb.y, rgb.z).0);
            }
        };

        cube(Vec2::zero());
        // cube(Vec2::unit_x() * F32::from_num(2.0));
        // cube(-Vec2::unit_x() * F32::from_num(2.0));

        // let sz = gfx::VIEW.map(|e| e as isize) / 2 - 1;
        // gfx::line(Vec2::new(-sz.x, -sz.y), Vec2::new(sz.x, -sz.y), 0xFF);
        // gfx::line(Vec2::new(-sz.x, sz.y), Vec2::new(sz.x, sz.y), 0xFF);
        // gfx::line(Vec2::new(-sz.x, -sz.y), Vec2::new(-sz.x, sz.y), 0xFF);
        // gfx::line(Vec2::new(sz.x, -sz.y), Vec2::new(sz.x, sz.y), 0xFF);

        // gfx::line(Vec2::broadcast(4), Vec2::new(16, 4), Color::from_rgb(255, 0, 0).0);
        // gfx::line(Vec2::broadcast(4), Vec2::new(4, 16), Color::from_rgb(0, 255, 0).0);

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
fn hblank_handler() {
    let scanline = VCOUNT.read();

    // unsafe {
    //     use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA0};
    //     static CLEAR: [u16; 2] = [0xFFFF; 2];
    //     DMA0::set_source(&CLEAR as *const _ as *const _);
    //     DMA0::set_dest(gfx::row_ptr_flip(scanline.saturating_sub(1) as isize, false) as *mut _ as *mut _);
    //     DMA0::set_count((gfx::SIZE.w / 2) as u16);
    //     DMA0::set_control(DMAControlSetting::new()
    //         .with_source_address_control(DMASrcAddressControl::Fixed)
    //         .with_dest_address_control(DMADestAddressControl::Increment)
    //         .with_dma_repeat(false)
    //         .with_use_32bit(true)
    //         .with_start_time(DMAStartTiming::Immediate)
    //         .with_enabled(true));
    // }

    // static mut LAST: u16 = 0;
    // for y in unsafe { LAST }..scanline {
    //     let mut row = gfx::row_ptr_flip(y as isize, false);
    //     for _ in 0..gfx::SIZE.w {
    //         unsafe {
    //             row.write_volatile(0);
    //             row = row.offset(1);
    //         }
    //     }
    // }

    // unsafe {
    //     LAST = if scanline == 0 { 0 } else { LAST.max(scanline).min(VBLANK_SCANLINE) };
    // }

    // if scanline <= VBLANK_SCANLINE {
    //     let mut row = gfx::row_ptr_flip((scanline as isize).saturating_sub(1), true);
    //     use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA2 as DMA};
    //     unsafe {
    //         #[repr(align(16))]
    //         struct DmaData([u16; 2]);
    //         static mut COL: DmaData= DmaData([0x0000; 2]);
    //         DMA::set_source(&COL as *const _ as *const _);
    //         DMA::set_dest(row as *mut _ as *mut _);
    //         DMA::set_count((gfx::SIZE.w / 2) as u16);
    //         DMA::set_control(DMAControlSetting::new()
    //             .with_source_address_control(DMASrcAddressControl::Fixed)
    //             .with_dest_address_control(DMADestAddressControl::Increment)
    //             .with_dma_repeat(false)
    //             .with_use_32bit(true)
    //             .with_start_time(DMAStartTiming::Immediate)
    //             .with_enabled(true));
    //     }
    // }

    // use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA0};
    // unsafe {
    //     static CLEAR: [u16; 2] = [0xFF00; 2];
    //     DMA0::set_source(&CLEAR as *const _ as *const _);
    //     DMA0::set_dest(gfx::row_ptr_flip(scanline as isize, true) as *mut _ as *mut _);
    //     DMA0::set_count((gfx::SIZE.w / 2) as u16);
    //     DMA0::set_control(DMAControlSetting::new()
    //         .with_source_address_control(DMASrcAddressControl::Fixed)
    //         .with_dest_address_control(DMADestAddressControl::Increment)
    //         .with_dma_repeat(true)
    //         .with_use_32bit(true)
    //         .with_start_time(DMAStartTiming::Immediate)
    //         .with_enabled(true));
    // }

    BIOS_IF.write(BIOS_IF.read().with_hblank(true));
}
fn vcounter_handler() { BIOS_IF.write(BIOS_IF.read().with_vcounter(true)); }
fn timer0_handler() { BIOS_IF.write(BIOS_IF.read().with_timer0(true)); }
fn timer1_handler() { BIOS_IF.write(BIOS_IF.read().with_timer1(true)); }

#[no_mangle]
pub unsafe extern fn __truncdfsf2() {}
