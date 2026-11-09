//! Point queries over a surface that keep one sheet of a thin wall from reading as the other.
//! A closed solid's two faces of a thin wall have opposite outward normals, so a query carries
//! the normal of the surface it is asked for and only triangles facing the same half-space
//! answer. Plain nearest-point would project a vertex through the wall onto the far sheet, and
//! read that far sheet as "on the surface" for the next check. The filter is the half-space
//! `dot > 0` rather than something tighter because a coarse triangle over a curved input can
//! legitimately tilt away from the local input normal; anything sharper would refuse good edits.

use crate::g3::dmesh3::DMesh3;
use crate::g3::dmesh_aabb_tree3::DMeshAABBTree3;
use crate::g3::tri_distance::distance_sqr_closest;
use crate::g3::vector3d::{cross, distance, dot, length, normalized, sub};

use super::engine::P;

/// Below this height over longest edge a triangle's normal is noise rather than a direction.
const NORMAL_MIN_HEIGHT_RATIO: f64 = 1e-6;
/// How many `f32` rounding steps a triangle's height must clear before its normal is trusted.
const HEIGHT_ROUNDING_STEPS: f64 = 8.0;

/// A triangle's unit normal, or `None` when it is too thin to have a meaningful one. Input
/// vertices are `f32`, so a triangle no taller than a few rounding steps of its own coordinates
/// has a normal made of rounding: far from the origin a thin CSG sliver would then read as a
/// false crease, or as facing the opposite wall.
pub(super) fn reliable_normal(a: P, b: P, c: P) -> Option<P> {
    let n = cross(sub(b, a), sub(c, a));
    let longest = distance(a, b).max(distance(b, c)).max(distance(c, a));
    if longest <= 0.0 || length(n) <= NORMAL_MIN_HEIGHT_RATIO * longest * longest {
        return None;
    }
    let magnitude = [a, b, c]
        .iter()
        .flat_map(|p| p.iter())
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let height = length(n) / longest;
    (height > HEIGHT_ROUNDING_STEPS * f64::from(f32::EPSILON) * magnitude).then(|| normalized(n))
}

/// A surface answering sheet-aware nearest-point queries.
pub(super) struct Sheets<'a> {
    tree: &'a DMeshAABBTree3<'a>,
    normals: Vec<Option<P>>,
}

impl<'a> Sheets<'a> {
    pub(super) fn new(tree: &'a DMeshAABBTree3<'a>) -> Self {
        let mesh: &DMesh3 = tree.mesh;
        let mut normals = vec![None; mesh.max_triangle_id().max(0) as usize];
        for t in mesh.triangle_indices() {
            let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
            normals[t as usize] = reliable_normal(a, b, c);
        }
        Self { tree, normals }
    }

    /// The surface being queried.
    pub(super) fn mesh(&self) -> &'a DMesh3 {
        self.tree.mesh
    }

    /// The nearest point on a triangle facing `normal`'s half-space, strictly within `reach`,
    /// and that triangle; `None` if no such triangle is that close.
    pub(super) fn nearest(&self, p: P, normal: P, reach: f64) -> Option<(P, i32)> {
        let accept = |t: i32| {
            self.normals
                .get(t as usize)
                .copied()
                .flatten()
                .is_some_and(|n| dot(n, normal) > 0.0)
        };
        let t = self.tree.find_nearest_triangle_where(p, reach, &accept);
        if t < 0 {
            return None;
        }
        let triangle = self.tree.mesh.get_tri_vertices(t as usize);
        Some((distance_sqr_closest(&triangle, p).1, t))
    }

    /// The distance from `p` to the same-facing sheet, or infinity if none is within `reach`.
    pub(super) fn distance(&self, p: P, normal: P, reach: f64) -> f64 {
        self.nearest(p, normal, reach)
            .map_or(f64::INFINITY, |(q, _)| distance(p, q))
    }
}
