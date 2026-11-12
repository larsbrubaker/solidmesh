//! Whether a hit on a packed face lies on the GPU route's analytic floor: the test the path loop
//! makes (`path_rests_on_floor` in `catcher.wgsl`) to give glass resting on the floor its own exit
//! event before the floor, as the CPU's `glass_rest` does for glass on any opaque face.
//! [`rests_on_floor_f32`] is the same equations in Rust, in the shader's order of operations, so a
//! test can check the GPU rule ray for ray against the CPU's decisions without a graphics card;
//! the constants are pinned against the shader's by a test that parses it.
//!
//! **The rule.** A face rests on the floor at a point when its outward normal points straight
//! down (cosine with down at least [`FLOOR_REST_ANTI_PARALLEL`], the contact rule's), the point
//! lies over the floor's square, and its height is within [`FLOOR_REST_ROUNDING_STEPS`] of the
//! larger of the two rounding steps (the face's own continuing-ray offset and the floor's) of the
//! floor's plane. It is fixed by the point and the face, never by the ray, so which of the face
//! and the floor a trace meets first never matters, and a face a modelled gap above the floor is
//! a real gap of air and stays one. The floor here is analytic, its rounding step taken at the
//! point, so the tolerance stays a few `f32` steps of the point's own coordinates whatever the
//! floor's size; the CPU's floor is a triangle, and its tolerance follows the triangle's corners
//! (`glass_contacts`), so over a CPU floor quad of half side 40,000 glass lifted 0.05 rests on
//! the CPU and is refused here.
//!
//! **Which hits the loop takes the rule at** ([`leaves_onto_floor_f32`]): a ray leaving a glass
//! part through the face (along its outward normal) where the glass is fully there. The analytic
//! floor is always there, so the glass's own opacity is the whole of the per-contact gate the CPU
//! applies to both faces; a part elsewhere being partly there changes nothing here. At opacity 1
//! the line's decision is a stop that draws no random number and counts no crossing, so taking
//! the rule there in place of that decision keeps every random draw where it was.
//!
//! **The rest comes before a seam here.** The GPU loop takes this rule first and then skips the
//! glass-on-glass seam at that hit; the CPU decides the seam first (`glass_rest`). The two differ
//! only at a hit that is both a glass face resting on the floor's plane and covered there by
//! another glass part's face, a glass part reaching up to the floor from below with glass on top:
//! there the GPU takes the rest and the CPU the seam.

use super::gpu_scene::{FaceRecord, PartRecord};
use crate::ray_tracer::flat_tree::SURFACE_HAS_GLASS;

/// How many rounding steps apart a face and the floor may be and still lie on each other: the
/// two carry their own rounding, so twice the larger covers both, as for glass on glass.
pub const FLOOR_REST_ROUNDING_STEPS: f32 = 2.0;

/// How close to straight down a face's normal must point to rest on the floor: about 0.08
/// degrees, the contact rule's.
pub const FLOOR_REST_ANTI_PARALLEL: f32 = 0.999_999;

/// How many `f32` steps of a floor point's coordinates a ray leaving the floor starts off it
/// (`PATH_FLOOR_OFFSET_STEPS` in `catcher.wgsl`).
pub const FLOOR_OFFSET_STEPS: f32 = 8.0;

/// How far a ray leaving the floor at `position` starts off it (`path_floor_offset`).
pub fn floor_offset_f32(position: [f32; 3]) -> f32 {
    (position[0].abs() + position[1].abs() + position[2].abs()) * f32::EPSILON * FLOOR_OFFSET_STEPS
}

/// Whether a hit at `position` on a face of outward normal `geometric` and continuing-ray offset
/// `ray_offset` lies on the floor `floor_plane` (center x, center y, height, half side), as
/// `path_rests_on_floor` decides it.
pub fn rests_on_floor_f32(
    position: [f32; 3],
    geometric: [f32; 3],
    ray_offset: f32,
    floor_plane: [f32; 4],
) -> bool {
    if geometric[2] > -FLOOR_REST_ANTI_PARALLEL {
        return false;
    }
    let from_center = [
        (position[0] - floor_plane[0]).abs(),
        (position[1] - floor_plane[1]).abs(),
    ];
    if from_center[0] > floor_plane[3] || from_center[1] > floor_plane[3] {
        return false;
    }
    let tolerance = FLOOR_REST_ROUNDING_STEPS * ray_offset.max(floor_offset_f32(position));
    (position[2] - floor_plane[2]).abs() <= tolerance
}

/// Whether the path loop takes a hit at `position` on `face` of `part`, reached along
/// `direction`, as glass leaving onto the floor `floor_plane`: the ray leaves through the face,
/// the part is fully there, the face rests on the floor and the part is glass, tested in that
/// order as the loop in `path.wgsl` tests them.
pub fn leaves_onto_floor_f32(
    direction: [f32; 3],
    position: [f32; 3],
    face: &FaceRecord,
    part: &PartRecord,
    floor_plane: [f32; 4],
) -> bool {
    let leaving = direction[0] * face.geometric[0]
        + direction[1] * face.geometric[1]
        + direction[2] * face.geometric[2]
        > 0.0;
    leaving
        && part.opacity >= 1.0
        && rests_on_floor_f32(position, face.geometric, face.ray_offset, floor_plane)
        && (part.surface.flags & SURFACE_HAS_GLASS) != 0
}
