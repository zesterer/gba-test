use core::{
    convert::TryFrom,
    intrinsics::{likely, unlikely},
    mem::MaybeUninit,
};
use vek::*;
use crate::{*, mem::*};
use super::Framebuffer;

type Num = F32;

pub const fn num(x: f32) -> Num {
    use fixed::traits::Fixed;
    Num::from_bits((x * (1 << Num::FRAC_NBITS) as f32) as <Num as Fixed>::Bits)
}

#[derive(Copy, Clone)]
pub struct Vertex {
    pos: Vec2<Num>,
    depth: Num,
}

// #[derive(Copy, Clone)]
// pub struct TexCoord {
//     uv: Vec2<u8>,
// }

#[derive(Copy, Clone)]
#[repr(u8)]
enum FaceKind {
    FlatTri { posi: [u16; 3], col: Rgb<Num> },
    FlatQuad { posi: [u16; 4], col: Rgb<Num> },
    TextureTri { posi: [u16; 3], uvi: [u16; 3] },
}

impl FaceKind {
    pub fn pos_indices(&self) -> &[u16] {
        match self {
            FaceKind::FlatTri { posi, .. } => posi,
            FaceKind::FlatQuad { posi, .. } => posi,
            FaceKind::TextureTri { posi, .. } => posi,
        }
    }
}

impl From<([u16; 3], Rgb<Num>)> for FaceKind {
    fn from((posi, col): ([u16; 3], Rgb<Num>)) -> Self { FaceKind::FlatTri { posi, col } }
}

impl From<([u16; 4], Rgb<Num>)> for FaceKind {
    fn from((posi, col): ([u16; 4], Rgb<Num>)) -> Self { FaceKind::FlatQuad { posi, col } }
}

impl From<([u16; 3], [u16; 3])> for FaceKind {
    fn from((posi, uvi): ([u16; 3], [u16; 3])) -> Self { FaceKind::TextureTri { posi, uvi } }
}

#[derive(Copy, Clone)]
pub struct Face {
    norm: Vec3<Num>,
    kind: FaceKind,
    depth: Num,
    model_idx: u16,
}

#[derive(Copy, Clone)]
pub struct Model {
    mvp: Mat4<Num>,
    mv: Mat4<Num>,
}

const MAX_MODELS: usize = 64;
const MAX_VERTS: usize = 1024;
const MAX_FACES: usize = 512;

pub struct Scene {
    model_count: u16,
    models: [Model; MAX_MODELS],
    vert_count: u16,
    verts: [Vertex; MAX_VERTS],
    face_count: u16,
    faces: [Face; MAX_FACES],
    faces_sorted: [u16; MAX_FACES],
}

pub struct SceneState {
    pub proj: Mat4<Num>,
    pub view: Mat4<Num>,
    pub light_dir: Vec3<Num>,
    pub ambiance: Num,
    pub light_col: Rgb<Num>,
}

impl Scene {
    pub fn begin_frame(&mut self, state: SceneState) -> SceneFrame<'_> {
        // Reset from last frame
        self.models[0] = Model {
            mvp: Mat4::identity(),
            mv: Mat4::identity(),
        };
        self.model_count = 1; // First is identity
        self.vert_count = 0;
        self.face_count = 0;

        SceneFrame { scene: self, state }
    }
}

pub struct SceneFrame<'a> {
    scene: &'a mut Scene,
    state: SceneState,
}

/// Take a pre-transposed matrix and a point, transforming the point using the matrix
#[link_section = ".text_fast"]
#[inline(always)]
#[no_mangle]
pub extern fn transform_pos(m: Mat4<Num>, v: Vec3<Num>) -> Vec4<Num> {
    // (m.transposed().map(NumWrap) * Vec4::from_point(v).map(NumWrap)).xyz().map(|e| e.0)

    // TODO: Is this correct? If not, use above
    Vec4 {
        x: m.cols.x.xyz().dot(v) + m.cols.x.w,
        y: m.cols.y.xyz().dot(v) + m.cols.y.w,
        z: m.cols.z.xyz().dot(v) + m.cols.z.w,
        w: m.cols.w.xyz().dot(v) + m.cols.w.w,
    }
}

/// Take a pre-transposed matrix and a normal, transforming the normal using the matrix
#[link_section = ".text_fast"]
#[inline(always)]
#[no_mangle]
extern fn transform_norm(m: Mat4<Num>, v: Vec3<Num>) -> Vec3<Num> {
    // (m.transposed().map(NumWrap) * Vec4::from_direction(v).map(NumWrap)).xyz().map(|e| e.0)

    // TODO: Is this correct? If not, use above
    Vec3 {
        x: m.cols.x.xyz().dot(v),
        y: m.cols.y.xyz().dot(v),
        z: m.cols.z.xyz().dot(v),
    }
}

fn normalize(v: Vec3<F32>) -> Vec3<F32> {
    fn finvsqrt(x: f32) -> f32 {
        let y = f32::from_bits(0x5f375a86 - (x.to_bits() >> 1));
        y * (1.5 - ( x * 0.5 * y * y ))
    }
    (v * F32::from_num(finvsqrt(v.magnitude_squared().to_num::<f32>())))
    // (v * v.magnitude_squared().sqrt())
}

impl<'a> SceneFrame<'a> {
    // Model index is guaranteed to contiguously increasing.
    pub fn add_model(&mut self, trans: Mat4<Num>) -> u16 {
        let idx = self.scene.model_count;
        self.scene.model_count += 1;
        self.scene.models[idx as usize] = Model {
            mvp: apply4(apply4(self.state.proj, self.state.view), trans).transposed(),
            mv: apply4(self.state.view, trans).transposed(),
        };
        idx
    }

    // Vertex index is guaranteed to contiguously increasing.
    #[link_section = ".text_fast"]
    #[inline(never)]
    pub fn add_vert(&mut self, model_idx: u16, pos: Vec3<Num>) -> u16 {
        let idx = self.scene.vert_count;
        self.scene.vert_count += 1;
        let p = transform_pos(self.scene.models[model_idx as usize].mvp, pos);
        self.scene.verts[idx as usize] = Vertex {
            pos: {
                // LUT composed of multiple ranges, going from 0 to 1097. Ranges: [0..1 (1), 1..9 (8), 9..73 (64), 73..1097 (1024)]
                const LUT_SIZE: usize = 512;
                const RANGE_SIZE: usize = LUT_SIZE / 4;
                static LUT: [Num; LUT_SIZE] = {
                    let mut lut = [num(0.0); LUT_SIZE];
                    let mut j = 0;
                    while j < 4 {
                        let mut i = 0;
                        while i < RANGE_SIZE {
                            let x = [0.0001, 1.0, 9.0, 73.0][j] + [1.0, 8.0, 64.0, 1024.0][j] * (i as f32 / RANGE_SIZE as f32);
                            lut[j * RANGE_SIZE + i] = num(-1.0 / if x < 0.0001 { 0.0001 } else { x });
                            i += 1;
                        }
                        j += 1;
                    }
                    lut
                };

                #[link_section = ".text_fast"]
                #[inline(always)]
                #[no_mangle]
                unsafe extern fn lut_get(x: Num) -> Num {
                    let idx = if likely(x >= num(73.0)) {
                        (num((3 * RANGE_SIZE) as f32) + (x - num(73.0)) * (num(RANGE_SIZE as f32) / num(1024.0))).min(num(LUT_SIZE as f32 - 1.0))
                    } else if x >= num(9.0) {
                        num((2 * RANGE_SIZE) as f32) + (x - num(9.0)) * (num(RANGE_SIZE as f32) / num(64.0))
                    } else if x >= num(1.0) {
                        num((1 * RANGE_SIZE) as f32) + ((x - num(1.0)) * (num(RANGE_SIZE as f32) / num(8.0)))
                    } else {
                        num((0 * RANGE_SIZE) as f32) + (x * num(RANGE_SIZE as f32))
                    };

                    let idx = idx.wrapping_to_num::<usize>();
                    debug_assert!(idx < LUT_SIZE);
                    *LUT.get_unchecked(idx)
                }

                let w_factor = unsafe { lut_get(-p.w.min(num(0.0))) };

                p.xy() * w_factor
            },
            depth: p.z,
        };
        idx
    }

    #[link_section = ".text_fast"]
    #[inline(never)]
    pub fn add_convex<K: Into<FaceKind>>(&mut self, model_idx: u16, kind: K, norm: Vec3<Num>) {
        let kind = kind.into();
        let vert_count = kind.pos_indices().len();

        // Check all vertices. If *all* of them fall outside *any* of the clipping bounds, clip the polygon.
        // We test this by clamping the coordinates to the clipping range (-1, 1) in each axis for each vertex, and
        // then summing these clipped vectors. If the absolute of the sum in any axis is >= (number of vertices) then
        // we know that the vertices must all have been outside the clipping range for that axis.
        if kind
            .pos_indices()
            .iter()
            .fold(Vec3::<Num>::zero(), |axes, idx| axes + self.scene.verts[*idx as usize].pos.map(|e| e.max(num(-1.0)).min(num(1.0))))
            .map(|e| e.abs() >= Num::from_num(vert_count))
            .reduce_or()
        {
            return;
        }

        let idx = self.scene.face_count;
        self.scene.face_count += 1;
        self.scene.faces[idx as usize] = Face {
            norm: normalize(transform_norm(self.scene.models[model_idx as usize].mv, norm)),
            depth: kind
                .pos_indices()
                .iter()
                .map(|idx| self.scene.verts[*idx as usize].depth)
                // 12 allows us to use a mul instead of a div because it is the lcm of 3 and 4
                .sum::<Num>() * Num::from_num(12 / vert_count),
            kind,
            model_idx,
        };
    }

    pub fn add_flat_tri(&mut self, model_idx: u16, pos: [Vec3<Num>; 3], col: Rgb<Num>, norm: Vec3<Num>) {
        let kind = FaceKind::FlatTri { posi: pos.map(|p| self.add_vert(model_idx, p)), col };
        self.add_convex(model_idx, kind, norm);
    }

    pub fn add_flat_quad(&mut self, model_idx: u16, pos: [Vec3<Num>; 4], col: Rgb<Num>, norm: Vec3<Num>) {
        let kind = FaceKind::FlatQuad { posi: pos.map(|p| self.add_vert(model_idx, p)), col };
        self.add_convex(model_idx, kind, norm);
    }

    pub fn render(self, fb: &mut Framebuffer) {
        let Scene { vert_count, verts, face_count, faces, faces_sorted, .. } = self.scene;
        let verts = &verts[0..*vert_count as usize];
        let faces = &faces[0..*face_count as usize];
        let faces_sorted = &mut faces_sorted[0..*face_count as usize];
        let state = self.state;

        let scr_scale = (fb.size() / 2).map(|e| num(e as f32));
        let scr_offset = (fb.size() / 2).map(|e| num(e as f32));

        // Reset face indices for this frame
        for (i, face_id) in faces_sorted.iter_mut().enumerate() {
            *face_id = i as u16;
        }

        // Sort faces by depth
        faces_sorted.sort_unstable_by_key(|i| unsafe { faces.get_unchecked(*i as usize).depth });

        // Draw faces
        for &mut face_id in faces_sorted {
            let face = unsafe { &faces.get_unchecked(face_id as usize) };

            let flat_light = || {
                let diffuse = (face.norm.dot(-state.light_dir)).max(num(0.0)).min(num(1.0));
                state.light_col * diffuse.max(state.ambiance)
            };

            let flat_shading = |light: Rgb<Num>, col: Rgb<Num>| {
                let surf = (col * light).map(|e| (e * num(32.0)).to_num::<u16>().min(31));
                Color::from_rgb(surf.r, surf.g, surf.b).0
            };

            let get_vert_scr = |idx| (unsafe { verts.get_unchecked(idx as usize).pos } * scr_scale + scr_offset).map(|e| e.to_num());

            match face.kind {
                FaceKind::FlatTri { posi, col } => fb.tri(&posi.map(get_vert_scr), flat_shading(flat_light(), col)),
                FaceKind::FlatQuad { posi, col } => fb.convex_fast(&posi.map(get_vert_scr), flat_shading(flat_light(), col)),
                FaceKind::TextureTri { .. } => todo!(),
            }
        }
    }
}

#[link_section = ".heap"]
static mut SCENE: MaybeUninit<Scene> = MaybeUninit::uninit();

pub unsafe fn init() -> &'static mut Scene {
    // This is UB, probably. The types are `Copy` though, and this code isn't supposed to be portable.
    unsafe { SCENE.assume_init_mut() }
}
