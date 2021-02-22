use super::*;
use crate::mem::*;
use vek::*;
use core::intrinsics::{likely, unlikely};

impl Framebuffer {
    pub fn clear(&mut self, px: Px) {
        unsafe {
            write_u32_fast(
                self.ptr() as *mut _,
                self.size().product() as usize / 2,
                (px as u32) << 16 | px as u32,
                true,
            );
        }
    }

    #[inline(always)]
    pub unsafe fn ptr_at(&mut self, pos: Vec2<usize>) -> *mut Px {
        let idx = pos.y * self.size().x as usize + pos.x;
        debug_assert!(idx < self.size().product() as usize, "Attempted to write to screen out of bounds");
        self.ptr().offset(idx as isize)
    }

    #[inline(always)]
    pub unsafe fn write(&mut self, pos: Vec2<usize>, px: Px) {
        self.ptr_at(pos).write(px);
    }

    pub fn line(&mut self, a: Vec2<isize>, b: Vec2<isize>, px: Px) {
        unsafe { self.line_inner(a, b, px, true); }
    }

    pub unsafe fn line_unchecked(&mut self, a: Vec2<isize>, b: Vec2<isize>, px: Px) {
        self.line_inner(a, b, px, false);
    }

    unsafe fn line_inner(&mut self, a: Vec2<isize>, b: Vec2<isize>, px: Px, checked: bool) {
        /// x is major axis, y is minor axis.
        #[inline(always)]
        fn bresenham<P, X, Y>(mut pos: P, delta: Vec2<usize>, mut x: X, mut y: Y) -> impl Iterator<Item = P>
            where P: Copy, X: FnMut(P) -> P, Y: FnMut(P) -> P,
        {
            let dy2 = (delta.y * 2) as isize;
            let dd2 = dy2 - (delta.x * 2) as isize;
            let mut err = dy2 - delta.x as isize;

            core::iter::repeat_with(move || {
                let old_pos = pos;
                err += if unlikely(err >= 0) {
                    pos = y(pos);
                    dd2
                } else {
                    dy2
                };
                pos = x(pos);
                old_pos
            })
        }

        let delta = b - a;
        let delta_abs = delta.map(|e| e.abs() as usize);
        let ptr = self.ptr_at(a.map(|e| e as usize));
        let dx = if delta.x > 0 { 1 } else { -1 };

        if checked {
            let dy = if delta.y > 0 { 1 } else { -1 };
            if delta_abs.x > delta_abs.y {
                bresenham(a, delta_abs, |p| p + Vec2::new(dx, 0), |p| p + Vec2::new(0, dy))
                    .take(delta_abs.x + 1)
                    .for_each(|p| if likely(p.map2(self.size(), |e, sz| e >= 0 && e < sz as isize).reduce_and()) { self.write(p.map(|e| e as usize), px) });
            } else {
                bresenham(a, delta_abs.yx(), |p| p + Vec2::new(0, dy), |p| p + Vec2::new(dx, 0))
                    .take(delta_abs.y + 1)
                    .for_each(|p| if likely(p.map2(self.size(), |e, sz| e >= 0 && e < sz as isize).reduce_and()) { self.write(p.map(|e| e as usize), px) });
            }
        } else {
            let dy = if delta.y > 0 { 1 } else { -1 } * self.size().x as isize;
            if delta_abs.x > delta_abs.y {
                bresenham(ptr, delta_abs, |p| p.offset(dx), |p| p.offset(dy))
                    .take(delta_abs.x + 1)
                    .for_each(|p| p.write(px));
            } else {
                bresenham(ptr, delta_abs.yx(), |p| p.offset(dy), |p| p.offset(dx))
                    .take(delta_abs.y + 1)
                    .for_each(|p| p.write(px));
            }
        }
    }

    pub fn tri(&mut self, verts @ [a, b, c]: &[Vec2<isize>; 3], px: Px) {
        // Backface culling
        if unlikely(c.determine_side(*a, *b) < 0) {
            return;
        }

        return self.convex(verts, px);

        // Order vertices vertically, convert to fixed point (TODO: do this beforehand?)
        let [a, b, c] = if a.y < b.y {
            if a.y < c.y { if b.y < c.y { [a, b, c] } else { [a, c, b] } } else { [c, a, b] }
        } else {
            if b.y < c.y { if a.y < c.y { [b, a, c] } else { [b, c, a] } } else { [c, b, a] }
        };


        let get_delta = |a: &Vec2<isize>, b: &Vec2<isize>| if a.y == b.y {
            (b.x - a.x) * ONE
        } else {
            // TODO: Fast fixed-range division using LUT
            (b.x - a.x) * ONE / (b.y - a.y)
            // unsafe { fast_div(b.x - a.x, b.y - a.y) }
        };

        // Find x deltas for each segment sides
        let dxab = get_delta(a, b);
        let dxbc = get_delta(b, c);
        let dxac = get_delta(a, c);
        let (ldxa, rdxa, ldxb, rdxb) = if dxab > dxac { // rhs
            (dxac, dxab, dxac, dxbc)
        } else {
            (dxab, dxac, dxbc, dxac)
        };

        let mid = c.x * ONE - (c.y - b.y) * dxac;

        let mut draw_segment = |y0: isize, y1: isize, mut x0: isize, mut x1: isize, ldx, rdx, use_dma| {
            x0 += ldx * -y0.min(0);
            x1 += rdx * -y0.min(0);

            let (y0, y1) = (y0.max(0), y1.min(self.size().y as isize));

            let mut row = unsafe { self.ptr_at(Vec2::new(0, y0 as usize)) };
            for _ in y0..y1 {
                let start = (x0.max(0) as usize >> FRAC).min(self.size().x);
                let end = ((x1 + ONE / 2).max(0) as usize >> FRAC).min(self.size().x);

                if use_dma {
                    unsafe { write_u16_fast(row.offset(start as isize), end - start, px, true); }
                } else {
                    unsafe {
                        for x in start..end {
                            row.offset(x as isize).write_volatile(px);
                        }
                    };
                }

                x0 += ldx;
                x1 += rdx;
                row = unsafe { row.offset(self.size().x as isize) };
            }
        };

        let use_dma = true;//a.x.max(b.x).max(c.x) - a.x.min(b.x).min(c.x) > 8;

        let (x0, x1) = if dxab > dxac {
            draw_segment(a.y, b.y, a.x * ONE, a.x * ONE, ldxa, rdxa, use_dma);
            (mid, b.x * ONE)
        } else {
            draw_segment(a.y, b.y, a.x * ONE, a.x * ONE, ldxa, rdxa, use_dma);
            (b.x * ONE, mid)
        };
        draw_segment(b.y, c.y, x0, x1.max(x0), ldxb, rdxb, use_dma);

        // self.line(*a, *b, 0);
        // self.line(*b, *c, 0);
        // self.line(*c, *a, 0);
    }

    pub fn convex<const N: usize>(&mut self, verts: &[Vec2<isize>; N], px: Px) {
        assert!(N >= 3, "Cannot rasterize convex polygon with fewer than 3 vertices");

        let sz = self.size().map(|e| e as isize);
        let bounds = Aabr {
            min: Vec2::max(verts.iter().fold(sz, |min, v| Vec2::min(min, *v)), Vec2::zero()),
            max: Vec2::min(verts.iter().fold(Vec2::zero(), |max, v| Vec2::max(max, *v)), sz),
        };

        let get_edge = |pos: Vec2<isize>, i| pos.determine_side(verts[i], verts[(i + 1) % N]);

        let mut edge = [0; N];
        let mut edge_delta_x = [0; N];
        let mut edge_delta_y = [0; N];

        for i in 0..N {
            let edge_min = get_edge(bounds.min, i);
            edge_delta_x[i] = get_edge(bounds.min + Vec2::unit_x(), i) - edge_min;
            edge_delta_y[i] = get_edge(bounds.min + Vec2::unit_y() * ONE, i) - edge_min;
            edge[i] = edge_min * ONE / edge_delta_x[i].abs().max(1);// + ONE / 2;
            edge_delta_y[i] /= edge_delta_x[i].abs().max(1);
        }

        let mut row = unsafe { self.ptr_at(Vec2::new(0, bounds.min.y as usize)) };
        for y in bounds.min.y..bounds.max.y {
            let start = bounds.min.x + 1 + (0..N).fold(isize::MIN, |max, i| if edge_delta_x[i] > 0 { max.max(-edge[i]) } else { max }) / ONE;
            let end = bounds.min.x + 1 + (0..N).fold(isize::MAX, |min, i| if edge_delta_x[i] < 0 { min.min(edge[i]) } else { min }) / ONE;

            let start = (start.max(0) as usize).min(self.size().x);
            let end = (end.max(0) as usize).max(start).min(self.size().x);

            unsafe { write_u16_fast(row.offset(start as isize), end - start, px, true); }

            row = unsafe { row.offset(self.size().x as isize) };

            for i in 0..N {
                edge[i] += edge_delta_y[i];
            }
        }

        // for i in 0..N {
        //     self.line(verts[i], verts[(i + 1) % N], 0xFFFF);
        // }
    }
}

const FRAC: usize = 16;
const ONE: isize = 1 << FRAC;

// const RECIP_LEN: usize = 64;
// const RECIP_LUT: [isize; RECIP_LEN] = make_recip_lut();

// // Divisor must be between 1 and 256 (uninclusive)
// pub unsafe fn fast_div(x: isize, y: isize) -> isize {
//     RECIP_LUT.get_unchecked(y as usize) * x
// }

// const fn make_recip_lut() -> [isize; RECIP_LEN] {
//     let mut lut = [0; RECIP_LEN];

//     let mut i = 1;
//     while i < lut.len() {
//         lut[i] = ONE / i as isize;
//         i += 1;
//     }

//     lut
// }
