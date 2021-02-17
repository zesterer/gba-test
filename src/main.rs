#![no_std]
#![feature(start, default_alloc_error_handler, array_map)]

use core::fmt::Write;
use vek::*;
use micromath::F32Ext;
use gba::{
    io::{
        irq::{set_irq_handler, IrqFlags, IrqEnableSetting, IE, IME, BIOS_IF},
        display::{
            DisplayControlSetting, DisplayStatusSetting, DisplayMode,
            DISPCNT, DISPSTAT, VCOUNT, VBLANK_SCANLINE,
        },
        background::{BackgroundControlSetting, BG2HOFS},
        timers::{TimerControlSetting, TimerTickRate, TM0CNT_H, TM0CNT_L},
    },
    bios,
    vram::bitmap::{Mode3, Mode5},
    Color,
};

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

    gba::info!("Starting...");

    set_irq_handler(irq_handler);
    IME.write(IrqEnableSetting::IRQ_YES);

    DISPSTAT.write(DisplayStatusSetting::new()
        .with_hblank_irq_enable(true)
        .with_vblank_irq_enable(true));

    TM0CNT_H.write(TimerControlSetting::new()
        .with_tick_rate(TimerTickRate::CPU1024)
        .with_enabled(true));

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

        // Set up display
        DISPCNT.write(DisplayControlSetting::new()
            .with_mode(DisplayMode::Mode5)
            .with_frame1(!gfx::flip())
            .with_bg2(true));

        unsafe {
            let bg2 = 0x4000000 as *mut u16;
            bg2.offset(0x10).write_volatile(0);
            bg2.offset(0x11).write_volatile(259);
            bg2.offset(0x12).write_volatile(137);
            bg2.offset(0x13).write_volatile(0);
        };

        // Wait for vblank
        IE.write(IrqFlags::new().with_hblank(true).with_vblank(true));
        bios::vblank_interrupt_wait();
        IE.write(IrqFlags::new().with_hblank(true).with_vblank(true));

        // Clear screen
        use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA2 as DMA};
        unsafe {
            #[repr(align(16))]
            struct DmaData([u16; 2]);
            static mut COL: DmaData= DmaData([0x0000; 2]);
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

        // let mvp = Mat4::perspective_fov_lh_zo(1.3, gfx::VIEW.x as f32, gfx::VIEW.y as f32, 0.01, 100.0)
        //     * Mat4::translation_3d(Vec3::new(0.0, 0.0, 3.0))
        //     * Mat4::rotation_x((tick as f32 * 0.002).sin() * 8.0)
        //     * Mat4::rotation_y((tick as f32 * 0.004).cos() * 4.0)
        //     * Mat4::rotation_z((tick as f32 * 0.008).sin() * 2.0)
        //     * Mat4::scaling_3d(Vec3::new(1.0, -1.0, 1.0));

        // const R: Rgba<f32> = Rgba::new(1.0, 0.0, 0.0, 1.0);
        // const Y: Rgba<f32> = Rgba::new(1.0, 1.0, 0.0, 1.0);
        // const G: Rgba<f32> = Rgba::new(0.0, 1.0, 0.0, 1.0);
        // const B: Rgba<f32> = Rgba::new(0.0, 0.0, 1.0, 1.0);

        // const VERTICES: &[(Vec4<f32>, Rgba<f32>)] = &[
        //     (Vec4::new(-1.0, -1.0, -1.0, 1.0), R),
        //     (Vec4::new(-1.0, -1.0,  1.0, 1.0), Y),
        //     (Vec4::new(-1.0,  1.0, -1.0, 1.0), G),
        //     (Vec4::new(-1.0,  1.0,  1.0, 1.0), B),
        //     (Vec4::new( 1.0, -1.0, -1.0, 1.0), B),
        //     (Vec4::new( 1.0, -1.0,  1.0, 1.0), G),
        //     (Vec4::new( 1.0,  1.0, -1.0, 1.0), Y),
        //     (Vec4::new( 1.0,  1.0,  1.0, 1.0), R),
        // ];

        // const INDICES: &[usize] = &[
        //     0, 3, 2, 0, 1, 3, // -x
        //     7, 4, 6, 5, 4, 7, // +x
        //     5, 0, 4, 1, 0, 5, // -y
        //     2, 7, 6, 2, 3, 7, // +y
        //     0, 6, 4, 0, 2, 6, // -z
        //     7, 1, 5, 3, 1, 7, // +z
        // ];

        // for indices in INDICES.chunks(3) {
        //     let a = ((mvp * VERTICES[indices[0]].0).xy() * 48.0).map(|e| e as isize);
        //     let b = ((mvp * VERTICES[indices[1]].0).xy() * 48.0).map(|e| e as isize);
        //     let c = ((mvp * VERTICES[indices[2]].0).xy() * 48.0).map(|e| e as isize);

        //     gfx::tri(a, b, c, Color::from_rgb(255, 255, 0).0);
        // }

        // gfx::clear();

        // unsafe {
        //     static mut COL: [u16; 2] = [0; 2];
        //     gba::bios::cpu_set32(
        //         &[0u32; 8] as *const _ as *const _,
        //         gfx::row_ptr_flip(0, true) as *mut _,
        //         gfx::SIZE.w as u32 / 2,
        //         true,
        //     );
        // }

        // let center = gfx::SIZE.map(|e| e as isize / 2).into();
        let [a, b, c] = [0, 1, 2]
            .map(|i| {
                let offset = tick as f32 * 0.05 + i as f32 * core::f32::consts::TAU / 3.0;
                (Vec2::new(offset.cos(), offset.sin()) * (180.0 + (tick as f32 / 20.0).sin() * 32.0 * 0.0)).map(|e| e as isize)
            });

        gfx::tri(a, b, Vec2::zero(), Color::from_rgb(255, 255, 0).0);
        gfx::tri(b, c, Vec2::zero(), Color::from_rgb(0, 255, 0).0);
        gfx::tri(c, a, Vec2::zero(), Color::from_rgb(0, 255, 255).0);

        let sz = gfx::VIEW.map(|e| e as isize) / 2 - 1;
        gfx::line(Vec2::new(-sz.x, -sz.y), Vec2::new(sz.x, -sz.y), 0xFF);
        gfx::line(Vec2::new(-sz.x, sz.y), Vec2::new(sz.x, sz.y), 0xFF);
        gfx::line(Vec2::new(-sz.x, -sz.y), Vec2::new(-sz.x, sz.y), 0xFF);
        gfx::line(Vec2::new(sz.x, -sz.y), Vec2::new(sz.x, sz.y), 0xFF);

        gfx::line(Vec2::broadcast(4), Vec2::new(16, 4), Color::from_rgb(255, 0, 0).0);
        gfx::line(Vec2::broadcast(4), Vec2::new(4, 16), Color::from_rgb(0, 255, 0).0);

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

    // if scanline <= VBLANK_SCANLINE.min(gfx::SIZE.h as u16) {
    //     let mut row = gfx::row_ptr_flip((scanline as isize).saturating_sub(1), true);
    //     unsafe { gba::bios::cpu_set32(
    //         &[scanline / 4; 8] as *const _ as *const _,
    //         row as *mut _,
    //         gfx::SIZE.w as u32 / 2,
    //         true,
    //     ); }
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
