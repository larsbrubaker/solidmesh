//! How the reducer (`reducer.rs`, whose child module this is) scores and places collapses on a
//! mesh carrying face textures or colours (`reducer_face_data.rs`). The seams there are refiner
//! constraints; this file adds the reducer's side of them:
//!
//! - **Placement.** A seam edge collapses to the quadric's minimum along the edge itself, so the
//!   kept point stays on the seam; an edge with a pinned end (a seam junction or end) or with
//!   exactly one seam end collapses onto that end; an edge joining two pinned vertices, or two
//!   seam vertices without being a seam, never collapses ([`Reducer::seam_refuses`]). A boundary
//!   edge with a pinned end (a seam reaching the boundary) collapses along the boundary onto it.
//!   The seam vertices' kinds come from `ReducerFaceData`'s dense arrays, not the constraint maps.
//! - **Seams keep their line.** Seam vertices' quadrics carry planes through each seam edge,
//!   perpendicular to its faces ([`Reducer::add_seam_planes`]), so cutting a bend in a seam - which
//!   moves the outline between two charts or colours - costs what the outline moves.
//! - **Texture-aware error.** An edge between two free vertices of a textured region is scored
//!   and placed by the sum of their attribute quadrics (`attribute_quadric.rs`), whose optimum
//!   gives the collapse point's UV as well as its position; an edge from a free vertex onto a
//!   seam vertex costs what the free vertex's attribute quadric says about the UVs its corners
//!   take there. Every other edge is scored by position, as on a bare mesh.

use super::super::attribute_quadric::{AttributeQuadric, Point5};
use super::super::quadric_error::QuadricError;
use super::super::reducer_face_data::{FREE, PINNED};
use super::super::vector3d::{add, cross, scale, sub, EPSILON};
use super::Reducer;

impl Reducer<'_, '_> {
    /// Edge `[a, b]`'s two seam kinds (`reducer_face_data.rs`), both free without face data.
    fn kinds(&self, a: i32, b: i32) -> (u8, u8) {
        match &self.face_data {
            Some(data) => (data.vertex_kind(a), data.vertex_kind(b)),
            None => (FREE, FREE),
        }
    }

    fn is_seam_edge(&self, eid: i32) -> bool {
        self.face_data
            .as_ref()
            .is_some_and(|data| data.is_seam_edge(eid))
    }

    /// Whether the seam rules forbid collapsing edge `[a, b]`: both ends pinned, or both on a
    /// seam with the edge itself not one.
    pub(super) fn seam_refuses(&self, eid: i32, a: i32, b: i32) -> bool {
        let (ka, kb) = self.kinds(a, b);
        if ka == PINNED && kb == PINNED {
            return true;
        }
        ka != FREE && kb != FREE && !self.is_seam_edge(eid)
    }

    /// Adds the seam-keeping planes to the seam vertices' quadrics: through every seam edge, one
    /// plane per face beside it, perpendicular to the face, weighted by the edge's squared
    /// length. A seam vertex that slides off the line its seam ran along - cutting a bend, which
    /// moves the outline between two charts or colours - then costs the squared distance it
    /// moves, as leaving the surface does.
    pub(super) fn add_seam_planes(&mut self) {
        let Some(data) = &self.face_data else {
            return;
        };
        let mesh = &*self.base.mesh;
        for eid in 0..mesh.max_edge_id() {
            if !mesh.is_edge(eid) || !data.is_seam_edge(eid) {
                continue;
            }
            let [a, b, t0, t1] = mesh.get_edge(eid);
            let va = mesh.vertex(a);
            let vb = mesh.vertex(b);
            let d = sub(vb, va);
            let len_sqr = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            if len_sqr <= 0.0 {
                continue;
            }
            for tid in [t0, t1] {
                let [p0, p1, p2] = mesh.get_tri_vertices(tid as usize);
                let face_normal = cross(sub(p1, p0), sub(p2, p0));
                let n = cross(d, face_normal);
                let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                if len <= EPSILON {
                    continue;
                }
                let n = scale(n, 1.0 / len);
                let plane = QuadricError::from_plane(n, va);
                let weight = SEAM_PLANE_WEIGHT * len_sqr;
                self.vert_quadrics[a as usize].add(weight, &plane);
                self.vert_quadrics[b as usize].add(weight, &plane);
            }
        }
    }

    /// Edge `[a, b]`'s collapse point, error and (from the attribute quadric) UV on a mesh with
    /// face data.
    pub(super) fn score_seamed_edge(
        &self,
        eid: i32,
        q: &QuadricError,
        a: i32,
        b: i32,
    ) -> ([f64; 3], f64, Option<[f64; 2]>) {
        let mesh = &*self.base.mesh;
        let (ka, kb) = self.kinds(a, b);
        let boundary_rule = self.have_boundary
            && self.preserve_boundary_shape
            && (mesh.is_boundary_edge(eid) || self.is_boundary_v(a) || self.is_boundary_v(b));
        if boundary_rule {
            // A boundary edge with a pinned end collapses onto that end.
            if mesh.is_boundary_edge(eid) && (ka == PINNED) != (kb == PINNED) {
                let p = mesh.vertex(if ka == PINNED { a } else { b });
                return (p, q.evaluate(p), None);
            }
            let p = self.optimal_point(eid, q, a, b);
            return (p, q.evaluate(p), None);
        }
        let Some(data) = &self.face_data else {
            let p = self.optimal_point(eid, q, a, b);
            return (p, q.evaluate(p), None);
        };
        let t0 = mesh.get_edge_t(eid)[0];
        if let Some(p) = self.seam_collapse_point(eid, q, a, b, (ka, kb)) {
            // A free vertex collapsing onto a seam vertex: its corners take the seam vertex's
            // UV on their side, which its attribute quadric prices.
            if (ka == FREE) != (kb == FREE) {
                let (r, k) = if ka == FREE { (a, b) } else { (b, a) };
                if let Some(qr) = data.attribute(r) {
                    let lifted = data.lifted_vertex(mesh, t0, k);
                    let err = self.vert_quadrics[k as usize].evaluate(p) + qr.evaluate(&lifted);
                    return (p, err, None);
                }
            }
            return (p, q.evaluate(p), None);
        }
        if self.minimize_quadric_position_error {
            if let (Some(qa), Some(qb)) = (data.attribute(a), data.attribute(b)) {
                let q5 = AttributeQuadric::sum(qa, qb);
                let x = q5.optimal_point().unwrap_or_else(|| {
                    cheapest_lifted(
                        &q5,
                        data.lifted_vertex(mesh, t0, a),
                        data.lifted_vertex(mesh, t0, b),
                    )
                });
                let lambda = data.lambda_of(t0);
                let p = self.project([x[0], x[1], x[2]]);
                let err = q5.evaluate(&[p[0], p[1], p[2], x[3], x[4]]);
                return (p, err, Some([x[3] / lambda, x[4] / lambda]));
            }
        }
        let p = self.optimal_point(eid, q, a, b);
        (p, q.evaluate(p), None)
    }

    /// Where a seam keeps edge `[ea, eb]`'s collapse point: a pinned end; on a seam edge, the
    /// quadric's minimum along the edge; the one end on a seam; on an edge joining two seam
    /// vertices (which may not collapse), the midpoint. `None` when neither end is on a seam.
    fn seam_collapse_point(
        &self,
        eid: i32,
        q: &QuadricError,
        ea: i32,
        eb: i32,
        (ka, kb): (u8, u8),
    ) -> Option<[f64; 3]> {
        let mesh = &*self.base.mesh;
        let va = mesh.vertex(ea);
        let vb = mesh.vertex(eb);
        if (ka == PINNED) != (kb == PINNED) {
            return Some(if ka == PINNED { va } else { vb });
        }
        if self.is_seam_edge(eid) {
            let t = quadric_minimum_along(q, va, vb);
            return Some([
                (1.0 - t) * va[0] + t * vb[0],
                (1.0 - t) * va[1] + t * vb[1],
                (1.0 - t) * va[2] + t * vb[2],
            ]);
        }
        match (ka != FREE, kb != FREE) {
            (false, false) => None,
            (true, false) => Some(va),
            (false, true) => Some(vb),
            (true, true) => Some(scale(add(va, vb), 0.5)),
        }
    }
}

/// How strongly a seam holds its line, relative to the surface: a seam vertex moving a distance
/// `d` off a seam edge of length `l` costs `SEAM_PLANE_WEIGHT * l^2 * d^2` per face beside it,
/// against about `l^2 / 2 * d^2` for leaving a face's plane.
const SEAM_PLANE_WEIGHT: f64 = 1.0;

/// The cheapest of two lifted ends and their midpoint under `q` (the first on ties), for a
/// quadric too flat to have one minimum.
fn cheapest_lifted(q: &AttributeQuadric, a: Point5, b: Point5) -> Point5 {
    let mut mid = [0.0; 5];
    for (k, m) in mid.iter_mut().enumerate() {
        *m = 0.5 * (a[k] + b[k]);
    }
    let mut best = (q.evaluate(&a), a);
    for candidate in [b, mid] {
        let err = q.evaluate(&candidate);
        if err < best.0 {
            best = (err, candidate);
        }
    }
    best.1
}

/// The parameter `t` in [0, 1] of the point on segment `from -> to` nearest `p`.
pub(super) fn parameter_along(from: [f64; 3], to: [f64; 3], p: [f64; 3]) -> f64 {
    let d = sub(to, from);
    let len_sqr = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if len_sqr <= 0.0 {
        return 1.0;
    }
    let w = sub(p, from);
    let t = (w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / len_sqr;
    if t.is_nan() {
        1.0
    } else {
        t.clamp(0.0, 1.0)
    }
}

/// The parameter `t` in [0, 1] minimizing quadric `q` at `a + t (b - a)`. Along the line the
/// error is `t^2 (d A d) + 2 t (d A a + b . d) + q(a)`, so the minimum is where its slope is zero;
/// a flat or downward line takes the cheaper end.
fn quadric_minimum_along(q: &QuadricError, a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub(b, a);
    let ad = [
        q.axx * d[0] + q.axy * d[1] + q.axz * d[2],
        q.axy * d[0] + q.ayy * d[1] + q.ayz * d[2],
        q.axz * d[0] + q.ayz * d[1] + q.azz * d[2],
    ];
    let curvature = d[0] * ad[0] + d[1] * ad[1] + d[2] * ad[2];
    let slope =
        (a[0] * ad[0] + a[1] * ad[1] + a[2] * ad[2]) + (q.bx * d[0] + q.by * d[1] + q.bz * d[2]);
    if curvature > 0.0 {
        let t = -slope / curvature;
        if t.is_nan() {
            0.5
        } else {
            t.clamp(0.0, 1.0)
        }
    } else if q.evaluate(b) < q.evaluate(a) {
        1.0
    } else {
        0.0
    }
}
