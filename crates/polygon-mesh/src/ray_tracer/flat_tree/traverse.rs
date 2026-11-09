//! The reference traversal over a flat tree's buffers, in `f32`, written so that it ports line
//! for line to WGSL: a loop over a fixed-size stack, no recursion, no pointers. Given a unit
//! direction and an origin within `COORDINATE_LIMIT` of zero (what [`FlatRay`] documents), no
//! value in it can become infinite or NaN from finite stored bounds: slab distances are at most
//! `2e18 * 1e18`, edge values are products of sheared coordinates, at most about `3.2e37`, and a
//! hit's distance is a weighted sum of corner depths, because the three edge values all share a
//! sign by the time they are divided, so each weight lies in `0..=1`. The module header in
//! `flat_tree` states the stack bound.
//!
//! The box test is the slab test on all four children of a node, with each child's interval
//! widened by [`SLAB_WIDEN`] so `f32` rounding in the subtractions and products can never reject
//! a child a ray really enters. A reciprocal direction would be infinite along an axis the ray
//! does not move in, and WGSL may assume no infinities, so along an axis whose direction is below
//! [`MIN_DIRECTION`] a slab is decided by whether the origin lies inside it, giving
//! `(-PARALLEL_SPAN, +PARALLEL_SPAN)` or a miss: no finite stand-in distance can be wrong about a
//! ray lying exactly on a slab's plane, where a reciprocal of 1e18 turns a tiny bound-to-origin
//! gap into a tiny exit distance.
//!
//! The triangle test is the ray-sheared watertight test: corners are translated to the ray's
//! origin and sheared so the ray runs along an axis, and three edge functions of the projected
//! corners say which side of each edge the ray falls on. Two triangles sharing an edge see the
//! same `f32` corners and compute the same products in the opposite order, so their edge values
//! here are exact negatives of each other and a ray cannot slip between them, with no epsilon on
//! the edges. A zero value counts as inside, so a ray exactly on an edge hits, as on the CPU. WGSL
//! may reassociate or fuse `a * b - c * d`, so `trace.wgsl` cannot promise those exact negatives:
//! it treats an edge value within a few epsilons of its products as zero, and it widens the
//! slabs more, for the 2.5 ULP division WGSL permits; its header states the budget for each.
//!
//! A hit is front-facing when the triangle's normal `(v1 - v0) x (v2 - v0)` faces the ray, the
//! same rule the triangle shapes use, and a ray's face mask selects which faces it reports.

use solidmesh_math::Ray;

use super::child_code;
use super::layout::{FlatNode, FlatTriangle};
use super::STACK_CAPACITY;

/// Report hits on front faces.
pub const FACE_FRONT: u32 = 1;
/// Report hits on back faces.
pub const FACE_BACK: u32 = 2;
/// Report both faces.
pub const FACE_BOTH: u32 = 3;

/// Added to each side of a child's slab interval, as a fraction of its distance. The distances
/// carry at most about 3 half-ulps of relative error each (the subtraction, the reciprocal and
/// the product), so 4 full `f32` epsilons is safely larger. `trace.wgsl` uses 8, because WGSL's
/// division may be off by 2.5 ULP where Rust's is correctly rounded.
const SLAB_WIDEN: f32 = 4.0 * f32::EPSILON;

/// A direction component below this magnitude means the ray runs parallel to that axis's slabs.
/// A reciprocal is then at most 1e18, so a slab distance stays finite for every coordinate the
/// build accepts (`COORDINATE_LIMIT` is 1e18, and 2e18 * 1e18 is far inside `f32`'s range).
const MIN_DIRECTION: f32 = 1e-18;

/// Stands in for "any distance along this axis" when a ray runs parallel to it: larger than every
/// finite slab distance (2e36 at the coordinate limit) and well inside `f32`'s range.
const PARALLEL_SPAN: f32 = 1e37;

/// A ray in the traversal's precision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlatRay {
    /// Within `COORDINATE_LIMIT` of zero on each axis; the traversal's arithmetic stays finite
    /// only for origins as well bounded as the corners the build accepts.
    pub origin: [f32; 3],
    /// A unit vector.
    pub direction: [f32; 3],
    /// Hits nearer than this are ignored.
    pub t_min: f32,
    /// Hits farther than this are ignored; finite, because WGSL may assume no infinities.
    pub t_max: f32,
    /// [`FACE_FRONT`], [`FACE_BACK`] or [`FACE_BOTH`].
    pub face_mask: u32,
}

impl FlatRay {
    /// `ray` narrowed: its origin and direction at the nearest `f32`, its window with an infinite
    /// end replaced by the largest finite `f32`, and its face type as a mask.
    pub fn from_ray(ray: &Ray) -> FlatRay {
        FlatRay {
            origin: [
                ray.origin.x as f32,
                ray.origin.y as f32,
                ray.origin.z as f32,
            ],
            direction: [
                ray.direction_normal.x as f32,
                ray.direction_normal.y as f32,
                ray.direction_normal.z as f32,
            ],
            t_min: finite_window_end(ray.min_distance_to_consider),
            t_max: finite_window_end(ray.max_distance_to_consider),
            face_mask: ray.intersection_type as u32,
        }
    }
}

/// `value` as a finite `f32`: the window's ends are compared with hit distances only, so the
/// largest finite `f32` stands in for no limit.
fn finite_window_end(value: f64) -> f32 {
    (value as f32).clamp(f32::MIN, f32::MAX)
}

/// A hit found by a flat traversal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlatHit {
    /// The distance along the ray.
    pub t: f32,
    /// The hit triangle's index in the triangle buffer.
    pub triangle: u32,
    /// The hit triangle's position in the list given to the build.
    pub source_index: u32,
    /// The weights of corners v0, v1 and v2 at the hit; they sum to 1.
    pub barycentric: [f32; 3],
    /// True when the ray struck the triangle's front face.
    pub front_face: bool,
}

/// The reciprocal of a direction component, finite by construction.
fn safe_inverse(direction: f32) -> f32 {
    let clamped = if direction.abs() < MIN_DIRECTION {
        if direction >= 0.0 {
            MIN_DIRECTION
        } else {
            -MIN_DIRECTION
        }
    } else {
        direction
    };
    1.0 / clamped
}

/// The distances at which a ray enters and leaves one slab `[min, max]` along one axis; entry
/// after exit when it never enters. A ray running parallel to the slab is inside or out for good.
fn slab(min: f32, max: f32, origin: f32, direction: f32, inverse: f32) -> (f32, f32) {
    if direction.abs() < MIN_DIRECTION {
        return if origin >= min && origin <= max {
            (-PARALLEL_SPAN, PARALLEL_SPAN)
        } else {
            (PARALLEL_SPAN, -PARALLEL_SPAN)
        };
    }
    let t0 = (min - origin) * inverse;
    let t1 = (max - origin) * inverse;
    (t0.min(t1), t0.max(t1))
}

/// The ray's shear and axis permutation for the triangle test.
struct Shear {
    /// The permuted axes: the ray runs along `kz` with `kx` and `ky` across it.
    kx: usize,
    ky: usize,
    kz: usize,
    sx: f32,
    sy: f32,
    sz: f32,
}

fn shear_of(direction: [f32; 3]) -> Shear {
    let mut kz = 0;
    if direction[1].abs() > direction[kz].abs() {
        kz = 1;
    }
    if direction[2].abs() > direction[kz].abs() {
        kz = 2;
    }
    let mut kx = (kz + 1) % 3;
    let mut ky = (kx + 1) % 3;
    // Keeps the projected winding the same when the ray runs against its major axis.
    if direction[kz] < 0.0 {
        std::mem::swap(&mut kx, &mut ky);
    }
    let sz = 1.0 / direction[kz];
    Shear {
        kx,
        ky,
        kz,
        sx: direction[kx] * sz,
        sy: direction[ky] * sz,
        sz,
    }
}

/// The hit of `ray` on `triangle` within `[ray.t_min, t_best]`, as `(t, barycentric, front)`.
fn hit_triangle(
    triangle: &FlatTriangle,
    ray: &FlatRay,
    shear: &Shear,
    t_best: f32,
) -> Option<(f32, [f32; 3], bool)> {
    let (kx, ky, kz) = (shear.kx, shear.ky, shear.kz);
    let a = [
        triangle.v0[0] - ray.origin[0],
        triangle.v0[1] - ray.origin[1],
        triangle.v0[2] - ray.origin[2],
    ];
    let b = [
        triangle.v1[0] - ray.origin[0],
        triangle.v1[1] - ray.origin[1],
        triangle.v1[2] - ray.origin[2],
    ];
    let c = [
        triangle.v2[0] - ray.origin[0],
        triangle.v2[1] - ray.origin[1],
        triangle.v2[2] - ray.origin[2],
    ];

    let ax = a[kx] - shear.sx * a[kz];
    let ay = a[ky] - shear.sy * a[kz];
    let bx = b[kx] - shear.sx * b[kz];
    let by = b[ky] - shear.sy * b[kz];
    let cx = c[kx] - shear.sx * c[kz];
    let cy = c[ky] - shear.sy * c[kz];

    let u = cx * by - cy * bx;
    let v = ax * cy - ay * cx;
    let w = bx * ay - by * ax;

    if (u < 0.0 || v < 0.0 || w < 0.0) && (u > 0.0 || v > 0.0 || w > 0.0) {
        return None;
    }

    let det = u + v + w;
    if det == 0.0 {
        return None;
    }

    let az = shear.sz * a[kz];
    let bz = shear.sz * b[kz];
    let cz = shear.sz * c[kz];
    // The weights first: `u * az` is cubic in the coordinates and would overflow at sizes the
    // build accepts, while u, v and w share a sign here so each weight is within 0..=1.
    let (u_weight, v_weight, w_weight) = (u / det, v / det, w / det);
    let t = u_weight * az + v_weight * bz + w_weight * cz;
    if !(t >= ray.t_min && t <= t_best) {
        return None;
    }

    let front_face = det > 0.0;
    let wanted = if front_face { FACE_FRONT } else { FACE_BACK };
    if ray.face_mask & wanted == 0 {
        return None;
    }

    Some((t, [u_weight, v_weight, w_weight], front_face))
}

/// The nearest hit of `ray`, walking `nodes` and `triangles`.
pub(super) fn trace_closest(
    nodes: &[FlatNode],
    triangles: &[FlatTriangle],
    ray: &FlatRay,
) -> Option<FlatHit> {
    trace(nodes, triangles, ray, false)
}

/// True when `ray` hits anything; stops at the first hit found.
pub(super) fn trace_any(nodes: &[FlatNode], triangles: &[FlatTriangle], ray: &FlatRay) -> bool {
    trace(nodes, triangles, ray, true).is_some()
}

fn trace(
    nodes: &[FlatNode],
    triangles: &[FlatTriangle],
    ray: &FlatRay,
    stop_at_first_hit: bool,
) -> Option<FlatHit> {
    let inverse = [
        safe_inverse(ray.direction[0]),
        safe_inverse(ray.direction[1]),
        safe_inverse(ray.direction[2]),
    ];
    let shear = shear_of(ray.direction);

    let mut stack_code = [0u32; STACK_CAPACITY];
    let mut stack_near = [0.0f32; STACK_CAPACITY];
    let mut stack_size = 1;
    stack_code[0] = 0;
    stack_near[0] = ray.t_min;

    let mut best: Option<FlatHit> = None;
    let mut t_best = ray.t_max;

    while stack_size > 0 {
        stack_size -= 1;
        let code = stack_code[stack_size];
        // A nearer hit may have been found since this entry was pushed.
        if stack_near[stack_size] > t_best {
            continue;
        }

        if child_code::is_leaf(code) {
            let first = child_code::first_triangle(code) as usize;
            let count = child_code::count(code) as usize;
            for offset in 0..count {
                let triangle = &triangles[first + offset];
                if let Some((t, barycentric, front_face)) =
                    hit_triangle(triangle, ray, &shear, t_best)
                {
                    t_best = t;
                    best = Some(FlatHit {
                        t,
                        triangle: (first + offset) as u32,
                        source_index: triangle.source_index,
                        barycentric,
                        front_face,
                    });
                    if stop_at_first_hit {
                        return best;
                    }
                }
            }
            continue;
        }

        let node = &nodes[code as usize];
        // The entered children, farthest first, so the nearest ends up on top of the stack.
        let mut order = [0u32; 4];
        let mut order_near = [0.0f32; 4];
        let mut order_count = 0;
        for slot in 0..4 {
            if node.child[slot] == child_code::EMPTY {
                continue;
            }
            let (near_x, far_x) = slab(
                node.min_x[slot],
                node.max_x[slot],
                ray.origin[0],
                ray.direction[0],
                inverse[0],
            );
            let (near_y, far_y) = slab(
                node.min_y[slot],
                node.max_y[slot],
                ray.origin[1],
                ray.direction[1],
                inverse[1],
            );
            let (near_z, far_z) = slab(
                node.min_z[slot],
                node.max_z[slot],
                ray.origin[2],
                ray.direction[2],
                inverse[2],
            );
            let near = near_x.max(near_y).max(near_z);
            let far = far_x.min(far_y).min(far_z);
            let near = near - near.abs() * SLAB_WIDEN;
            let far = far + far.abs() * SLAB_WIDEN;
            if near <= far && far >= ray.t_min && near <= t_best {
                // Insertion by descending distance; ties keep slot order, deterministically.
                let mut at = order_count;
                while at > 0 && order_near[at - 1] < near {
                    order[at] = order[at - 1];
                    order_near[at] = order_near[at - 1];
                    at -= 1;
                }
                order[at] = node.child[slot];
                order_near[at] = near;
                order_count += 1;
            }
        }
        for position in 0..order_count {
            stack_code[stack_size] = order[position];
            stack_near[stack_size] = order_near[position];
            stack_size += 1;
        }
    }

    best
}
