//! The GPU route's coverage along one line, `path_coverage_along` in `catcher.wgsl`, in Rust:
//! [`coverage_along_f32`] walks a packed scene with the `f32` reference traversal
//! ([`super::FlatTree::trace_closest`]) in the shader's order of operations, so a test can hold
//! the card's coverage of a camera line against the CPU's `RenderScene::coverage_along` line for
//! line without a graphics card, the way `floor_rest.rs` mirrors the path loop's floor rules. A
//! test pins the shader's text to these steps.
//!
//! **The walk.** Each stretch is traced to its nearest face; before the line's end a fully there
//! part ends the line covered, a see-through part leaves what it does not cover the first time the
//! line meets it, and the line goes on from just past the face, along the same direction. A line
//! the caller ends (a camera line ended at the floor's plane, [`catcher_distance_f32`]) keeps that
//! end on every stretch. The first stretch is traced to the end; every later one to its next face
//! wherever that lies, since past a see-through part's base lying on another part's top the
//! stretch starts past both faces, inside the other part. Once a face lies more than its surface
//! offset past the end, or a stretch starts past it, the walk goes on to the line's last face,
//! keeping after the passed parts in `line_passed` the parts not passed before the end that it has
//! crossed an odd number of times (`flip_past`, `line_flip_past`); the end lies inside each part
//! kept there, which then counts as one met before the end does. `RenderScene::coverage_along`'s
//! module header states the rule and its limits. Each kept part and each passed part took a
//! crossing, so the shader's `line_passed`, as long as the crossing cap, holds them all.

use super::gpu_scene::GpuRenderScene;
use super::gpu_view::SCENE_FLAG_SEE_THROUGH;
use super::traverse::{FlatRay, FACE_BOTH};

/// The far limit of an unbounded line (`PATH_FAR` in `path.wgsl`).
pub const PATH_FAR_F32: f32 = 3.0e38;

/// The most surfaces of see-through parts one straight line goes through
/// (`MAX_CROSSINGS_PER_LINE` in `path.wgsl`).
pub const MAX_CROSSINGS_PER_LINE_F32: u32 = 512;

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// How far along a camera line from `origin` along `direction` it reaches the floor
/// `floor_plane` (center x, center y, height, half side) from above, or `None`:
/// `path_catcher_distance`, a floor behind an orthographic line's start counting.
pub fn catcher_distance_f32(
    origin: [f32; 3],
    direction: [f32; 3],
    floor_plane: [f32; 4],
    orthographic: bool,
) -> Option<f32> {
    // The shader's `!(z < 0)`: a NaN direction misses too.
    if direction[2].is_nan() || direction[2] >= 0.0 {
        return None;
    }
    let t = (floor_plane[2] - origin[2]) / direction[2];
    if t <= 0.0 && !orthographic {
        return None;
    }
    let at = [origin[0] + direction[0] * t, origin[1] + direction[1] * t];
    if (at[0] - floor_plane[0]).abs() > floor_plane[3]
        || (at[1] - floor_plane[1]).abs() > floor_plane[3]
    {
        return None;
    }
    Some(t)
}

/// How much of `origin_ray` (in `pack`'s coordinates, its face mask ignored) the parts of `pack`
/// cover, as `path_coverage_along` finds it.
pub fn coverage_along_f32(pack: &GpuRenderScene, origin_ray: &FlatRay) -> f32 {
    let direction = origin_ray.direction;
    let bounded = origin_ray.t_max < PATH_FAR_F32;
    let end: [f32; 3] =
        std::array::from_fn(|axis| origin_ray.origin[axis] + direction[axis] * origin_ray.t_max);
    let see_through = pack.view().flags & SCENE_FLAG_SEE_THROUGH != 0;
    let mut left = origin_ray.t_max;
    let mut ray = FlatRay {
        face_mask: FACE_BOTH,
        ..*origin_ray
    };
    // `line_passed`: the parts passed before the end, then the parts crossed an odd number of
    // times past it (`odd` of them).
    let mut line_passed: Vec<u32> = Vec::new();
    let mut passed_count = 0usize;
    let mut crossings = 0u32;
    let mut uncovered = 1.0f32;
    let mut past = false;
    let mut odd = 0usize;
    while let Some(hit) = pack.tree().trace_closest(&ray) {
        let triangle = pack.tree().triangles()[hit.triangle as usize];
        let face = pack.faces()[hit.source_index as usize];
        let [u, v, w] = hit.barycentric;
        let position: [f32; 3] = std::array::from_fn(|axis| {
            triangle.v0[axis] * u + triangle.v1[axis] * v + triangle.v2[axis] * w
        });
        past = past || (see_through && hit.t > left + face.ray_offset);
        if past {
            if crossings >= MAX_CROSSINGS_PER_LINE_F32 {
                return 1.0;
            }
            crossings += 1;
            if !line_passed[..passed_count].contains(&face.part) {
                odd = flip_past(&mut line_passed, passed_count, odd, face.part);
            }
        } else {
            let opacity = pack.parts()[face.part as usize].opacity;
            if opacity >= 1.0 {
                return 1.0;
            }
            if crossings >= MAX_CROSSINGS_PER_LINE_F32 {
                return 1.0;
            }
            crossings += 1;
            if !line_passed[..passed_count].contains(&face.part) {
                line_passed.push(face.part);
                passed_count += 1;
                uncovered *= 1.0 - opacity;
            }
        }
        let side = if dot(direction, face.geometric) >= 0.0 {
            face.ray_offset
        } else {
            -face.ray_offset
        };
        let origin: [f32; 3] =
            std::array::from_fn(|axis| position[axis] + face.geometric[axis] * side);
        ray = FlatRay {
            origin,
            direction,
            t_min: 0.0,
            t_max: PATH_FAR_F32,
            face_mask: FACE_BOTH,
        };
        if bounded && !past {
            let to_end: [f32; 3] = std::array::from_fn(|axis| end[axis] - origin[axis]);
            left = dot(to_end, direction);
            if left <= 0.0 {
                if !see_through {
                    return 1.0 - uncovered;
                }
                past = true;
            }
        }
    }
    for &part in &line_passed[passed_count..passed_count + odd] {
        let opacity = pack.parts()[part as usize].opacity;
        if opacity >= 1.0 {
            return 1.0;
        }
        uncovered *= 1.0 - opacity;
    }
    1.0 - uncovered
}

/// `line_flip_past`: flips whether `part` is among the `odd` parts held after the first
/// `passed_count` entries of `line_passed`, a part leaving giving its place to the last one, and
/// answers how many are held then.
fn flip_past(line_passed: &mut Vec<u32>, passed_count: usize, odd: usize, part: u32) -> usize {
    for index in passed_count..passed_count + odd {
        if line_passed[index] == part {
            line_passed[index] = line_passed[passed_count + odd - 1];
            line_passed.truncate(passed_count + odd - 1);
            return odd - 1;
        }
    }
    line_passed.push(part);
    odd + 1
}
