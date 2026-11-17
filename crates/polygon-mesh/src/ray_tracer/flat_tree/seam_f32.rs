//! The GPU route's seam, `seam_at` in `seam.wgsl`, in Rust: what a hit on a glass face lying on
//! another glass part's face is, decided from the packed contact block (`gpu_contacts.rs`) in the
//! shader's order of operations, for the line toward the sun's mirror (`sun_line.rs`, whose walk
//! asks it at each hit) and for a test to hold against the CPU's seam (`path_tracer/glass_seam.rs`)
//! without a graphics card. A test pins the shader's text to these steps.
//!
//! **Pictures.** The pack decides one glass, which glass is denser and the interface between two
//! from the two faces' colors. In a scene with pictures ([`SCENE_FLAG_PICTURES`]) the seam also
//! reads the covering face's picture, at that face's own point under the hit
//! ([`cover_weights_f32`], the CPU's `barycentric_at` in `f32` on the face's packed corners), as
//! the CPU reads it. Where the covering part is there and either face's factor differs from 1,
//! the two surfaces are compared as the CPU compares them: one glass when the pack found the
//! materials the same apart from color ([`CONTACT_SAME_MATERIAL`]) and the tinted base colors
//! are equal, and an interface between two glasses takes the pack's record with its color the
//! two tinted colors multiplied. Elsewhere a factor of 1 leaves each surface as its color alone
//! makes it, so the pack's decisions are the CPU's. Packed texture coordinates mean something only
//! within their own face, so each face's picture is looked up on its own coordinates and only the
//! factors are compared.

use super::coverage_line::MAX_CROSSINGS_PER_LINE_F32;
use super::gpu_contacts::{
    CONTACT_COVER_DENSER, CONTACT_HIT_DENSER, CONTACT_NO_SEAM, CONTACT_SAME_GLASS,
    CONTACT_SAME_MATERIAL,
};
use super::gpu_view::{SCENE_FLAG_PICTURES, SCENE_FLAG_SEE_THROUGH};
use super::sun_line::{
    facing_f32, surface_at_f32, LineRecordF32, LineVerdictF32, PathPointF32, SunLineScene,
};
use super::traverse::FlatHit;
use super::SurfaceRecord;

/// The shader's `CONTACT_SKIP_TOLERANCES`.
const SKIP_TOLERANCES_F32: f32 = 2.0;

type V3 = [f32; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The one interface at a seam a vertex is on (`PathSeam` of kind `SEAM_INTERFACE`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamF32 {
    pub surface: SurfaceRecord,
    pub into_denser: bool,
    pub skip: f32,
    /// The part not there on the line that reached the seam, and whether its side is the one the
    /// hit face's normal points to.
    pub absent: Option<(u32, bool)>,
}

/// What `seam_at` found at a hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SeamAtF32 {
    None,
    Straight { skip: f32 },
    Interface(SeamF32),
    Ended,
}

/// `seam_cover_hit`'s weights: the CPU's `barycentric_at` in `f32`, the weights of `position` on
/// the triangle `corners`. The sub-triangle areas' ratios where they place the point inside; then
/// each edge's nearest point, taken where it rebuilds a point nearer `position` (ties keep the
/// earlier answer), so the weights are always on the triangle; a third each where nothing is
/// finite. The edges are three straight steps with their weights written out, as the shader's.
pub fn cover_weights_f32(corners: [V3; 3], position: V3) -> V3 {
    let [c0, c1, c2] = corners;
    let normal = cross(sub(c1, c0), sub(c2, c0));
    let normal_squared = dot(normal, normal);
    let rebuilt = |w: V3| {
        let at = std::array::from_fn(|axis| {
            c0[axis] * w[0] + c1[axis] * w[1] + c2[axis] * w[2] - position[axis]
        });
        dot(at, at)
    };
    let mut best: Option<(V3, f32)> = None;
    if normal_squared > 0.0 && normal_squared.is_finite() {
        let w0 = dot(normal, cross(sub(c1, position), sub(c2, position))) / normal_squared;
        let w1 = dot(normal, cross(sub(c2, position), sub(c0, position))) / normal_squared;
        let w2 = dot(normal, cross(sub(c0, position), sub(c1, position))) / normal_squared;
        let sum = w0 + w1 + w2;
        if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 && sum > 0.0 && sum.is_finite() {
            let weights = [w0 / sum, w1 / sum, w2 / sum];
            let distance = rebuilt(weights);
            if distance.is_finite() {
                best = Some((weights, distance));
            }
        }
    }
    let t01 = edge_along(c0, c1, position);
    let w01 = [1.0 - t01, t01, 0.0];
    let t12 = edge_along(c1, c2, position);
    let w12 = [0.0, 1.0 - t12, t12];
    let t20 = edge_along(c2, c0, position);
    let w20 = [t20, 0.0, 1.0 - t20];
    for weights in [w01, w12, w20] {
        let distance = rebuilt(weights);
        if distance.is_finite() && best.is_none_or(|(_, nearest)| distance < nearest) {
            best = Some((weights, distance));
        }
    }
    best.map_or([1.0 / 3.0; 3], |(weights, _)| weights)
}

/// `seam_edge_along`: how far along the edge from `start` to `end` the point nearest `position`
/// lies, in [0, 1]; 0 on an edge of no length.
fn edge_along(start: V3, end: V3, position: V3) -> f32 {
    let edge = sub(end, start);
    let edge_squared = dot(edge, edge);
    let mut t = 0.0;
    if edge_squared > 0.0 {
        t = (dot(sub(position, start), edge) / edge_squared).clamp(0.0, 1.0);
    }
    t
}

impl SunLineScene<'_> {
    /// `seam_cover_hit`: a hit on the packed triangle `cover` (its index in the tree's triangle
    /// buffer) at `position`, its weights [`cover_weights_f32`] on the triangle's corners, for
    /// the picture lookup, which reads nothing else of it; `None` for a triangle the tree lacks.
    fn cover_hit(&self, cover: u32, position: V3) -> Option<FlatHit> {
        let triangle = self.pack.tree().triangles().get(cover as usize)?;
        Some(FlatHit {
            t: 0.0,
            triangle: cover,
            source_index: triangle.source_index,
            barycentric: cover_weights_f32([triangle.v0, triangle.v1, triangle.v2], position),
            front_face: false,
        })
    }

    /// `seam_at`: the seam at `point` on the hit's packed face `triangle`, for a line along
    /// `direction` that found the hit part there when `here_present`. A pack whose flags say it
    /// has pictures but which lacks the entry's covering triangle (one no pack builds) has no seam
    /// there, before anything is drawn.
    pub fn seam_at(
        &self,
        point: &PathPointF32,
        triangle: u32,
        direction: V3,
        here_present: bool,
        line: &mut LineRecordF32,
        there: &mut dyn FnMut(f32) -> bool,
    ) -> SeamAtF32 {
        let contacts = self.pack.glass_contacts();
        let (first, entries, fully) = contacts.of_triangle_from(triangle as usize);
        let found = if fully {
            entries.first().map(|entry| (0, entry))
        } else {
            entries
                .iter()
                .enumerate()
                .find(|(_, entry)| entry.covers_f32(point.position))
        };
        let Some((offset, entry)) = found else {
            return SeamAtF32::None;
        };
        if entry.flags & CONTACT_NO_SEAM != 0 {
            return SeamAtF32::None;
        }
        let mut cover_hit = None;
        if self.flags() & SCENE_FLAG_PICTURES != 0 {
            let cover = contacts.covering_triangles.get(first + offset);
            cover_hit = cover.and_then(|&cover| self.cover_hit(cover, point.position));
            if cover_hit.is_none() {
                return SeamAtF32::None;
            }
        }
        let mut there_present = true;
        if self.flags() & SCENE_FLAG_SEE_THROUGH != 0 {
            let verdict = line.at_hit(self.pack, entry.part, there);
            if verdict == LineVerdictF32::Ended {
                return SeamAtF32::Ended;
            }
            there_present = verdict == LineVerdictF32::Stops;
        }
        let skip = SKIP_TOLERANCES_F32 * entry.tolerance;
        let leaving_here = facing_f32(point, direction).1;
        if !here_present && !there_present {
            return SeamAtF32::Straight { skip };
        }
        let floor = self.floor_surface();
        // The covering face's picture, where the covering part is there to show it.
        let mut there_picture = [1.0; 3];
        let mut pictured = false;
        if let (true, Some(hit)) = (there_present, &cover_hit) {
            there_picture = self.picture_at(hit);
            pictured = there_picture != [1.0; 3] || point.picture != [1.0; 3];
        }
        let mut same = entry.flags & CONTACT_SAME_GLASS != 0;
        let mut tint = [0.0; 3];
        if pictured && here_present {
            let here = surface_at_f32(self.pack, floor, point.part, point.color, point.picture);
            let there = surface_at_f32(self.pack, floor, entry.part, entry.color, there_picture);
            same = entry.flags & CONTACT_SAME_MATERIAL != 0 && here.base_color == there.base_color;
            tint = std::array::from_fn(|c| here.base_color[c] * there.base_color[c]);
        }
        if here_present && there_present && same {
            if line.crossings >= MAX_CROSSINGS_PER_LINE_F32 {
                return SeamAtF32::Ended;
            }
            line.crossings += 1;
            return SeamAtF32::Straight { skip };
        }
        let (surface, into_denser, absent) = if here_present && !there_present {
            (
                surface_at_f32(self.pack, floor, point.part, point.color, point.picture),
                !leaving_here,
                Some((entry.part, true)),
            )
        } else if !here_present && there_present {
            (
                surface_at_f32(self.pack, floor, entry.part, entry.color, there_picture),
                leaving_here,
                Some((point.part, false)),
            )
        } else {
            let denser = if leaving_here {
                entry.flags & CONTACT_COVER_DENSER != 0
            } else {
                entry.flags & CONTACT_HIT_DENSER != 0
            };
            let mut interface = entry.interface;
            if pictured {
                interface.base_color = tint;
            }
            (interface, denser, None)
        };
        SeamAtF32::Interface(SeamF32 {
            surface,
            into_denser,
            skip,
            absent,
        })
    }
}

/// `seam_absent_toward`.
pub(super) fn absent_toward(seam: &SeamF32, point: &PathPointF32, direction: V3) -> Option<u32> {
    let (part, on_normal_side) = seam.absent?;
    let toward_normal_side = dot(direction, point.geometric) >= 0.0;
    (toward_normal_side == on_normal_side).then_some(part)
}

/// `seam_origin`: past both faces of a seam of half width `skip`, on the side `direction`
/// leaves by.
pub(super) fn seam_origin_f32(point: &PathPointF32, direction: V3, skip: f32) -> V3 {
    let side = if dot(direction, point.geometric) >= 0.0 {
        skip
    } else {
        -skip
    };
    std::array::from_fn(|axis| point.position[axis] + point.geometric[axis] * side)
}
