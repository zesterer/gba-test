use vek::*;
use gba::io::display::{DisplayControlSetting, DisplayMode, DISPCNT};
use core::cell::UnsafeCell;

pub type Px = u16;

const FB_SIZE: Vec2<usize> = Vec2::new(160, 128);

#[repr(align(256))]
pub struct Framebuffer(UnsafeCell<[Px; FB_SIZE.x * FB_SIZE.y]>);

impl Framebuffer {
    pub const fn size(&self) -> Vec2<usize> { FB_SIZE }

    #[inline(always)]
    pub fn raw(&mut self) -> &mut [Px; FB_SIZE.x * FB_SIZE.y] {
        self.0.get_mut()
    }

    #[inline(always)]
    pub fn ptr(&mut self) -> *mut Px {
        self.0.get_mut() as *mut _ as *mut _
    }
}

static mut FB0_VISIBLE: bool = true;

#[repr(align(256))]
pub struct Screen([Framebuffer; 2]);

impl Screen {
    fn write_cfg(&mut self) {
        // Set up display
        DISPCNT.write(DisplayControlSetting::new()
            .with_mode(DisplayMode::Mode5)
            .with_frame1(!unsafe { FB0_VISIBLE })
            .with_bg2(true));

        unsafe {
            let bg2 = 0x4000000 as *mut u16;
            // Swap axes and fill
            // bg2.offset(0x10).write_volatile(0);
            // bg2.offset(0x11).write_volatile(259);
            // bg2.offset(0x12).write_volatile(137);
            // bg2.offset(0x13).write_volatile(0);

            // Stretch fill
            // bg2.offset(0x10).write_volatile(171);
            // bg2.offset(0x13).write_volatile(207);
        };
    }

    pub fn flip(&mut self) {
        unsafe { FB0_VISIBLE ^= true; };
        self.write_cfg();
    }

    pub fn fb_index(&mut self) -> usize {
        if unsafe { FB0_VISIBLE } { 1 } else { 0 }
    }

    #[inline(always)]
    pub fn back(&mut self) -> &mut Framebuffer {
        &mut self.0[if unsafe { FB0_VISIBLE } { 1 } else { 0 }]
    }
}

/// Must only be called once.
pub unsafe fn init() -> &'static mut Screen {
    FB0_VISIBLE = true;

    const SCREEN: *mut Screen = 0x600_0000 as *mut Screen;

    let screen = &mut *SCREEN;
    screen.write_cfg();
    screen
}
