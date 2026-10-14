//! The pair-corner end-face field's construction and queries: the constructor, `of`, the field
//! queries and their exact candidate sets; the type lives in `pair_corner_end_face_field.rs`.
//! Every query is EXACT: the field is the max of
//! six linear pieces and `size - r` (r the distance off the round's axis), so its least over a
//! piece stands at a corner, where two pieces tie along an edge, or where three tie inside a
//! triangle, and its greatest at a corner or the point nearest the axis.

use vector_math::{cs_math, Vector3};

use super::{
    ChainDistanceField, FieldMaximum, FieldMinimum, LevelLineWork, PairCornerEndFaceField,
    ScalarField, WorkError,
};

impl PairCornerEndFaceField {
    /// The field from its parts; `of` reads one off a pair corner.
    pub fn new(
        axis_point: Vector3,
        axis_direction: Vector3,
        size: f64,
        planes: Vec<(Vector3, f64)>,
    ) -> Self {
        Self {
            axis_point,
            axis_direction,
            size,
            planes,
        }
    }

    /// The field for a round of `size` on the edge leaving `corner` along `edge_direction`, between
    /// faces of outward normals `shared` and `own` (both containing the edge), cutting the end face
    /// of outward normal `end_face` through the corner. Bounded to the corner's END REGION by four
    /// more linear pieces: a slab within `size` of the end face's plane and a span along the edge
    /// that holds everything of that plane in front of the round.
    pub fn of(
        corner: Vector3,
        edge_direction: Vector3,
        shared: Vector3,
        own: Vector3,
        end_face: Vector3,
        size: f64,
    ) -> Self {
        // The axis stands size inside both faces: o = a (shared + own) with o . n = -size for each n.
        let k = shared.dot(own);
        let offset = (shared + own) * (-size / (1.0 + k));
        let axis = corner + offset;
        let along = edge_direction.get_normal();
        let toward = |n: Vector3| {
            let back = offset * -1.0;
            let across = back - (n * back.dot(n));
            across.get_normal()
        };

        // -(q . m) = -(p - axis) . m, as m is square to the axis.
        let tangent = |m: Vector3| (m * -1.0, m.dot(axis));

        // g . (p - corner) - bound.
        let bound = |g: Vector3, bound: f64| (g, -g.dot(corner) - bound);

        // On the end face's plane a point in front of the round is at most |offset| from the edge
        // across it; its run along the edge is that times the plane's slope to the edge. A plane
        // nearly along the edge is capped.
        let normal = end_face.get_normal();
        let slope = along.dot(normal).abs();
        let span = (offset.length() * cs_math::max_f64(0.0, 1.0 - (slope * slope)).sqrt()
            / cs_math::max_f64(slope, 0.2))
            + size;

        Self::new(
            axis,
            along,
            size,
            vec![
                tangent(toward(shared)),
                tangent(toward(own)),
                bound(along, span),
                bound(along * -1.0, span),
                bound(normal, size),
                bound(normal * -1.0, size),
            ],
        )
    }

    /// The field, uncharged: `size - r`, raised by every linear piece.
    pub fn read(&self, point: Vector3) -> f64 {
        let mut value = self.size - self.across(point).length();
        for &(gradient, constant) in &self.planes {
            value = cs_math::max_f64(value, gradient.dot(point) + constant);
        }

        value
    }

    /// The least reading over `points` (strictly smaller displaces), or +infinity at `fallback`.
    pub fn least(&self, fallback: Vector3, points: &[Vector3]) -> FieldMinimum {
        let mut best = FieldMinimum::new(f64::INFINITY, fallback);
        for &at in points {
            let value = self.read(at);
            if value < best.value {
                best = FieldMinimum::new(value, at);
            }
        }

        best
    }

    /// The greatest reading over `points` above `at_least` (strictly greater displaces), or
    /// `at_least` at `fallback`.
    pub fn greatest(&self, at_least: f64, fallback: Vector3, points: &[Vector3]) -> FieldMaximum {
        let mut best = FieldMaximum::new(at_least, fallback);
        for &at in points {
            let value = self.read(at);
            if value > best.value {
                best = FieldMaximum::new(value, at);
            }
        }

        best
    }

    /// The three points a segment's greatest is read at: its ends and the extra one.
    pub fn pairs(a: Vector3, b: Vector3, extra: Vector3) -> [Vector3; 3] {
        [a, b, extra]
    }

    /// The point's offset from the axis, across it.
    pub fn across(&self, point: Vector3) -> Vector3 {
        let from = point - self.axis_point;
        from - (self.axis_direction * from.dot(self.axis_direction))
    }

    /// A segment's two ends and every point along it where two pieces tie - between those the one
    /// piece that is the max is linear or concave along the segment, so its least is at a break.
    pub fn segment_candidates(&self, a: Vector3, b: Vector3) -> Vec<Vector3> {
        let mut candidates = vec![a, b];
        for t in self.ties_along(a, b - a) {
            if t > 0.0 && t < 1.0 {
                candidates.push(a + ((b - a) * t));
            }
        }

        candidates
    }

    /// Every t at which two pieces tie on the line a + t d.
    pub fn ties_along(&self, a: Vector3, d: Vector3) -> Vec<f64> {
        let mut ties = Vec::new();
        let q0 = self.across(a);
        let dq = self.across(a + d) - q0;
        for (i, &(gi, ci)) in self.planes.iter().enumerate() {
            let l0 = gi.dot(a) + ci;
            let l1 = gi.dot(d);

            // Two linear pieces.
            for &(gj, cj) in &self.planes[i + 1..] {
                let slope = l1 - gj.dot(d);
                if slope.abs() > 1e-15 {
                    ties.push(((gj.dot(a) + cj) - l0) / slope);
                }
            }

            // size - |q| = l0 + t l1: |q| = c0 + t c1, squared (a root with c0 + t c1 < 0 is
            // extraneous, and harmless: it is only read).
            let c0 = self.size - l0;
            let c1 = -l1;
            ties.extend(Self::quadratic_roots(
                dq.dot(dq) - (c1 * c1),
                2.0 * (q0.dot(dq) - (c0 * c1)),
                q0.dot(q0) - (c0 * c0),
            ));
        }

        ties
    }

    /// Points of triangle abc where three pieces tie: three linear pieces at one point of its
    /// plane, or two linear pieces along a line of its plane where `size - r` meets them.
    pub fn triple_points(&self, a: Vector3, b: Vector3, c: Vector3) -> Vec<Vector3> {
        let mut points = Vec::new();
        let e1 = b - a;
        let e2 = c - a;
        let inside = |s: f64, t: f64| s >= -1e-12 && t >= -1e-12 && s + t <= 1.0 + 1e-12;

        // A linear piece in the triangle's coordinates: v + s gs + t gt.
        let local: Vec<(f64, f64, f64)> = self
            .planes
            .iter()
            .map(|&(g, constant)| (g.dot(a) + constant, g.dot(e1), g.dot(e2)))
            .collect();
        for i in 0..local.len() {
            for j in i + 1..local.len() {
                // L_i - L_j = 0: alpha s + beta t + gamma = 0.
                let alpha = local[i].1 - local[j].1;
                let beta = local[i].2 - local[j].2;
                let gamma = local[i].0 - local[j].0;
                let norm = (alpha * alpha) + (beta * beta);
                if norm < 1e-24 {
                    continue;
                }

                for k in j + 1..local.len() {
                    let alpha2 = local[i].1 - local[k].1;
                    let beta2 = local[i].2 - local[k].2;
                    let gamma2 = local[i].0 - local[k].0;
                    let determinant = (alpha * beta2) - (beta * alpha2);
                    if determinant.abs() > 1e-15 {
                        let s = ((-gamma * beta2) + (beta * gamma2)) / determinant;
                        let t = ((-alpha * gamma2) + (gamma * alpha2)) / determinant;
                        if inside(s, t) {
                            points.push(a + (e1 * s) + (e2 * t));
                        }
                    }
                }

                // The tie line: its point nearest the origin of (s, t), and its direction.
                let s0 = -alpha * gamma / norm;
                let t0 = -beta * gamma / norm;
                let origin = a + (e1 * s0) + (e2 * t0);
                let direction = (e1 * -beta) + (e2 * alpha);
                for u in self.ties_with_the_round(origin, direction, i) {
                    let s = s0 - (beta * u);
                    let t = t0 + (alpha * u);
                    if inside(s, t) {
                        points.push(origin + (direction * u));
                    }
                }
            }
        }

        points
    }

    /// Every t at which `size - r` ties linear piece `piece` on the line a + t d.
    pub fn ties_with_the_round(&self, a: Vector3, d: Vector3, piece: usize) -> Vec<f64> {
        let q0 = self.across(a);
        let dq = self.across(a + d) - q0;
        let (g, constant) = self.planes[piece];
        let c0 = self.size - (g.dot(a) + constant);
        let c1 = -g.dot(d);
        Self::quadratic_roots(
            dq.dot(dq) - (c1 * c1),
            2.0 * (q0.dot(dq) - (c0 * c1)),
            q0.dot(q0) - (c0 * c0),
        )
    }

    /// The point of triangle abc whose offset across the axis is `q`, or `None` when the
    /// triangle's projection is degenerate (its edges then hold every candidate) or does not hold
    /// q.
    pub fn on_triangle(&self, a: Vector3, b: Vector3, c: Vector3, q: Vector3) -> Option<Vector3> {
        let qa = self.across(a);
        let e1 = self.across(b) - qa;
        let e2 = self.across(c) - qa;
        let w = q - qa;
        let d11 = e1.dot(e1);
        let d12 = e1.dot(e2);
        let d22 = e2.dot(e2);
        let determinant = (d11 * d22) - (d12 * d12);
        if determinant <= 1e-18 * cs_math::max_f64(1.0, d11 * d22) {
            return None;
        }

        let s = ((d22 * w.dot(e1)) - (d12 * w.dot(e2))) / determinant;
        let t = ((d11 * w.dot(e2)) - (d12 * w.dot(e1))) / determinant;

        // In the plane of P as long as q is: the axis itself is.
        if s < 0.0 || t < 0.0 || s + t > 1.0 {
            return None;
        }

        Some(a + ((b - a) * s) + ((c - a) * t))
    }

    /// The real roots of a t^2 + b t + c, the larger first; the linear root when a vanishes.
    pub fn quadratic_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
        if a.abs() < 1e-15 {
            return if b.abs() > 1e-15 {
                vec![-c / b]
            } else {
                Vec::new()
            };
        }

        let discriminant = (b * b) - (4.0 * a * c);
        if discriminant < 0.0 {
            return Vec::new();
        }

        let root = discriminant.sqrt();
        vec![(-b + root) / (2.0 * a), (-b - root) / (2.0 * a)]
    }

    /// The point of segment pq nearest the axis, across it.
    fn nearest_the_axis(&self, p: Vector3, q: Vector3) -> Vector3 {
        let qp = self.across(p);
        let d = self.across(q) - qp;
        let dd = d.dot(d);
        let t = if dd > 0.0 {
            (-qp.dot(d) / dd).clamp(0.0, 1.0)
        } else {
            0.0
        };
        p + ((q - p) * t)
    }
}

impl ScalarField for PairCornerEndFaceField {
    fn as_chain_distance_field(&self) -> Option<&dyn super::ChainDistanceField> {
        Some(self)
    }

    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        LevelLineWork::charge(1)?;
        Ok(self.read(point))
    }
}

impl ChainDistanceField for PairCornerEndFaceField {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        LevelLineWork::charge(1)?;
        Ok(self.least(a, &self.segment_candidates(a, b)))
    }

    /// Exact. Inside a region where one piece is the max nothing is least - a linear piece falls
    /// toward the region's rim, and `size - r` is concave - so the least lies on the triangle's
    /// rim or on a curve where two pieces tie: the edges' candidates and the triple points.
    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        LevelLineWork::charge(1)?;
        let mut candidates = self.segment_candidates(a, b);
        candidates.extend(self.segment_candidates(b, c));
        candidates.extend(self.segment_candidates(c, a));
        candidates.extend(self.triple_points(a, b, c));
        Ok(self.least(a, &candidates))
    }

    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_along_segment_at_least(a, b, tolerance, f64::NEG_INFINITY)
    }

    /// Exact: the greatest of a max is the greatest of its pieces' greatest values. The linear
    /// pieces peak at an end; `size - r` peaks where the segment comes closest to the axis.
    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        _tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let extra = self.nearest_the_axis(a, b);
        Ok(self.greatest(at_least, a, &Self::pairs(a, b, extra)))
    }

    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_over_triangle_bounded(a, b, c, tolerance, f64::INFINITY, f64::NEG_INFINITY)
    }

    /// The corners, and the point of the triangle nearest the axis - on the axis itself when the
    /// projection holds it, else on an edge.
    fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        _tolerance: f64,
        _stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let mut points = vec![a, b, c];
        match self.on_triangle(a, b, c, Vector3::ZERO) {
            Some(nearest) => points.push(nearest),
            None => {
                for (p, q) in [(a, b), (b, c), (c, a)] {
                    points.push(self.nearest_the_axis(p, q));
                }
            }
        }

        Ok(self.greatest(at_least, a, &points))
    }
}
