use vek::*;
use bresenham::*;
use gba::{
    vram::bitmap::{Page, Mode5},
    Color,
};

// pub const SIZE: Extent2<usize> = Extent2::new(240, 160);
pub const SIZE: Extent2<usize> = Extent2::new(160, 128);

const SCREEN: *mut u16 = 0x600_0000 as *mut u16;

pub const VIEW: Vec2<isize> = Vec2::new(186, 130);

// Only safe in debug mode
#[inline(always)]
pub fn put(pos: Vec2<isize>, col: u16) {
    unsafe { SCREEN
        .offset(if FLIPPED { SIZE.product() as isize } else { 0 })
        .offset(pos.y * SIZE.w as isize + pos.x)
        .write_volatile(col); }
}

pub fn clear() {
    Mode5::clear_to(if unsafe { FLIPPED } { Page::One } else { Page::Zero }, Color::from_rgb(0, 0, 0));

    // unsafe { gba::bios::cpu_set32(
    //     &[0u32; 8] as *const _,
    //     SCREEN.offset(if FLIPPED { SIZE.product() as isize } else { 0 }) as *mut _,
    //     SIZE.product() as u32 / 2,
    //     true,
    // ); }
}

static mut FLIPPED: bool = false;

pub fn flip() -> bool {
    unsafe { FLIPPED ^= true; FLIPPED }
}

pub fn row_ptr_flip(y: isize, flip: bool) -> *mut u16 {
    unsafe { SCREEN
        .offset(if FLIPPED ^ flip { SIZE.product() as isize } else { 0 })
        .offset(y * SIZE.w as isize) }
}

pub fn row_ptr(y: isize) -> *mut u16 {
    row_ptr_flip(y, false)
}

fn to_scr(pos: Vec2<isize>) -> Vec2<isize> {
    Vec2::from(SIZE.map(|e| e as isize)) / 2 + pos * Vec2::new(7, 8) / 8
}

pub fn line(a: Vec2<isize>, b: Vec2<isize>, col: u16) {
    Bresenham::new(to_scr(a).into_tuple(), to_scr(b).into_tuple()).for_each(|p| {
        put(p.into(), col);
    });
}

pub fn tri(a: Vec2<isize>, b: Vec2<isize>, c: Vec2<isize>, mut col: u16) {
    // Backface culling
    if c.determine_side(a, b) < 0 {
        return;
        // col = 0xF00F;
    }

    let a = to_scr(a);
    let b = to_scr(b);
    let c = to_scr(c);

    // let (a_, b_, c_) = (a, b, c);

    let (a, b, c) = if a.y < b.y {
        if a.y < c.y {
            if b.y < c.y { (a, b, c) } else { (a, c, b) }
        } else {
            (c, a, b)
        }
    } else {
        if b.y < c.y {
            if a.y < c.y { (b, a, c) } else { (b, c, a) }
        } else {
            (c, b, a)
        }
    };

    const FACTOR_L2: usize = 16;
    const FACTOR: isize = 1 << FACTOR_L2;

    let (mut x0, mut x1) = (a.x * FACTOR, a.x * FACTOR);

    let mut get_delta = |a: Vec2<isize>, b: Vec2<isize>| if a.y == b.y {
        ((b.x - a.x) * FACTOR)
    } else {
        ((b.x - a.x) * FACTOR) / (b.y - a.y)
    };

    let dxac = get_delta(a, c); debug_assert!((c.x - a.x).abs() * FACTOR >= dxac.abs());
    let dxab = get_delta(a, b); debug_assert!((b.x - a.x).abs() * FACTOR >= dxab.abs());
    let dxbc = get_delta(b, c); debug_assert!((c.x - b.x).abs() * FACTOR >= dxbc.abs());
    let (ldxa, rdxa, ldxb, rdxb) = if dxab > dxac { // rhs
        (dxac, dxab, dxac, dxbc)
    } else {
        (dxab, dxac, dxbc, dxac)
    };

    #[repr(align(16))]
    struct DmaData([u16; 2]);
    static mut COL: DmaData= DmaData([0x0000; 2]);
    unsafe { COL.0 = [col; 2]; }

    let draw_segment = |y0: isize, y1: isize, x0: &mut isize, x1: &mut isize, ldx, rdx| {
        *x0 += ldx * -y0.min(0);
        *x1 += rdx * -y0.min(0);

        let (y0, y1) = (y0.max(0), y1.min(SIZE.h as isize));

        let mut row = row_ptr(y0);
        for y in y0..y1 {
            let start = ((*x0).max(0) as usize >> FACTOR_L2) as isize;
            let end = ((*x1).max(0) >> FACTOR_L2).min(SIZE.w as isize);

            debug_assert!(start <= end);

            let mut px = unsafe { row.offset(start) };
            if end - start < 16 {
                for x in start..end {
                    unsafe { px.write_volatile(col); };
                    px = unsafe { px.offset(1) };
                }
            } else {
                use gba::io::dma::{DMAControlSetting, DMASrcAddressControl, DMADestAddressControl, DMAStartTiming, DMA0 as DMA};
                unsafe {
                    DMA::set_source(&COL as *const _ as *const _);
                    DMA::set_dest(row.offset(start) as *mut _ as *mut _);
                    DMA::set_count((end - start) as u16);
                    DMA::set_control(DMAControlSetting::new()
                        .with_source_address_control(DMASrcAddressControl::Fixed)
                        .with_dest_address_control(DMADestAddressControl::Increment)
                        .with_dma_repeat(false)
                        .with_use_32bit(false)
                        .with_start_time(DMAStartTiming::Immediate)
                        .with_enabled(true));
                }
            }

            *x0 += ldx;
            *x1 += rdx;
            row = unsafe { row.offset(SIZE.w as isize) };
        }
    };

    draw_segment(a.y, b.y, &mut x0, &mut x1, ldxa, rdxa);
    if dxab > dxac {
        x1 = b.x * FACTOR;
    } else {
        x0 = b.x * FACTOR;
    }
    draw_segment(b.y, c.y, &mut x0, &mut x1, ldxb, rdxb);

    // put(a_.map2(SIZE.into(), |e, sz: usize| e.clamped(0, sz as isize - 1)), 0x00FF);
    // put(b_.map2(SIZE.into(), |e, sz: usize| e.clamped(0, sz as isize - 1)), 0x0FF0);
    // put(c_.map2(SIZE.into(), |e, sz: usize| e.clamped(0, sz as isize - 1)), 0xFF00);

    // put(a + Vec2::new(2, 0), 0x00FF);
    // put(b + Vec2::new(2, 0), 0x0FF0);
    // put(c + Vec2::new(2, 0), 0xFF00);
}
