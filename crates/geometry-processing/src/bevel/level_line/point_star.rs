//! A picked point and the faces around it, read for [`super::point_bevel`] (`Bevel/LevelLine/PointStar.cs`,
//! C# `internal sealed class PointStar`): the bevel's shape per unit of size (the fillet's
//! inscribed ball, the chamfer's plane) and its room (the size at which the ring first reaches a
//! feature edge - dihedral over the bend threshold - never a triangulation diagonal).
//!
//! The edge map is an `IndexMap` because C# walks its `Dictionary` in insertion order, and the
//! order reaches the answer: the neighbour and feature-end lists are summed and averaged in it.

use indexmap::IndexMap;
use vector_math::Vector3;

use super::point_bevel::PointBevelKind;

/// C# `PointStar`.
#[derive(Debug, Clone, PartialEq)]
pub struct PointStar {
    point: Vector3,
    face_normals: Vec<Vector3>,
    face_angles: Vec<f64>,
    neighbours: Vec<Vector3>,
    /// The far ends of the point's own feature edges.
    feature_ends: Vec<Vector3>,
    /// Every feature edge not touching the point.
    far_features: Vec<(Vector3, Vector3)>,
    /// The fillet ball's centre offset from the point per unit of radius, once
    /// [`PointStar::try_shape`] has read it.
    ball_offset: Vector3,
}

impl PointStar {
    /// C#'s constructor: the star of `vertex` read off the whole mesh.
    pub fn new(
        positions: &[Vector3],
        triangles: &[(i32, i32, i32)],
        vertex: i32,
        bend_threshold_degrees: f64,
    ) -> Self {
        let point = positions[vertex as usize];
        let at = |i: i32| positions[i as usize];
        let normals: Vec<Vector3> = triangles
            .iter()
            .map(|&(a, b, c)| (at(b) - at(a)).cross(at(c) - at(a)).get_normal())
            .collect();

        let mut face_normals = Vec::new();
        let mut face_angles = Vec::new();
        let mut edge_triangles: IndexMap<(i32, i32), Vec<usize>> = IndexMap::new();
        for (i, &(a, b, c)) in triangles.iter().enumerate() {
            for (u, w) in [(a, b), (b, c), (c, a)] {
                edge_triangles
                    .entry((u.min(w), u.max(w)))
                    .or_default()
                    .push(i);
            }

            if a == vertex || b == vertex || c == vertex {
                let corners = [a, b, c];
                let k = corners.iter().position(|&v| v == vertex).unwrap_or(0);
                let u = at(corners[(k + 1) % 3]) - point;
                let w = at(corners[(k + 2) % 3]) - point;
                face_normals.push(normals[i]);
                face_angles.push(u.cross(w).length().atan2(u.dot(w)));
            }
        }

        let bend = (bend_threshold_degrees * std::f64::consts::PI / 180.0).cos();
        let mut neighbours = Vec::new();
        let mut feature_ends = Vec::new();
        let mut far_features = Vec::new();
        for (&(low, high), on) in &edge_triangles {
            let feature = on.len() != 2 || normals[on[0]].dot(normals[on[1]]) < bend;
            if low == vertex || high == vertex {
                let other = at(if low == vertex { high } else { low });
                neighbours.push(other);
                if feature {
                    feature_ends.push(other);
                }
            } else if feature {
                far_features.push((at(low), at(high)));
            }
        }

        PointStar {
            point,
            face_normals,
            face_angles,
            neighbours,
            feature_ends,
            far_features,
            ball_offset: Vector3::ZERO,
        }
    }

    /// C# `HasSharpEdges`: true when a feature edge (dihedral over the bend threshold) meets the
    /// point.
    pub fn has_sharp_edges(&self) -> bool {
        !self.feature_ends.is_empty()
    }

    /// C# `BallOffset`: the fillet ball's centre offset from the point per unit of radius.
    pub fn ball_offset(&self) -> Vector3 {
        self.ball_offset
    }

    /// C# `TryShape`: the bevel's shape per unit of size, as `(per_size, inward)`, or the refusal in
    /// words. A fillet's `per_size` is the setback per unit of ball radius: the ball's centre offset
    /// c solves c . n = -1 over the distinct face planes (least squares - exact on a right cone or a
    /// cube corner, a fair approximation where no single ball touches every face), and the tangent
    /// ring stands sqrt(|c|^2 - 1) from the point. A chamfer's is the cut plane's depth per unit of
    /// setback along `inward`: through the setback points on the point's feature edges where it has
    /// three or more, else square to its mean normal.
    pub fn try_shape(&mut self, kind: PointBevelKind) -> Result<(f64, Vector3), String> {
        if kind == PointBevelKind::Fillet {
            let mut distinct: Vec<Vector3> = Vec::new();
            for &n in &self.face_normals {
                if !distinct.iter().any(|d| d.dot(n) > 1.0 - 1e-12) {
                    distinct.push(n);
                }
            }

            let rhs = distinct.iter().fold(Vector3::ZERO, |s, &n| s - n);
            match solve(&distinct, rhs) {
                Some(c) if c.length() > 1.0 => {
                    self.ball_offset = c;
                    return Ok(((c.length_squared() - 1.0).sqrt(), Vector3::ZERO));
                }
                _ => return Err("the faces around this point hold no ball".to_string()),
            }
        }

        let edges: Vec<Vector3> = self
            .feature_ends
            .iter()
            .map(|&e| (e - self.point).get_normal())
            .collect();
        if edges.len() >= 3 {
            let rhs = edges.iter().fold(Vector3::ZERO, |s, &e| s + e);
            if let Some(w) = solve(&edges, rhs) {
                if w.length() > 0.0 {
                    return Ok((1.0 / w.length(), w.get_normal()));
                }
            }
        }

        let mut mean = Vector3::ZERO;
        for i in 0..self.face_normals.len() {
            mean += self.face_normals[i] * self.face_angles[i];
        }

        if mean.length() == 0.0 {
            return Err("this point has no direction to cut square to".to_string());
        }

        let inward = -mean.get_normal();
        let per_size = average(
            self.neighbours
                .iter()
                .map(|&n| (n - self.point).get_normal().dot(inward)),
        );
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(perSize > 0)`, NaN included.
        if !(per_size > 0.0) {
            return Err("this point is not convex".to_string());
        }

        Ok((per_size, inward))
    }

    /// C# `Room`: the size at which the ring first reaches a feature - a far feature edge or the
    /// end of the point's own.
    pub fn room(&self, kind: PointBevelKind, per_size: f64, inward: Vector3) -> f64 {
        let mut reach = f64::INFINITY;
        if kind == PointBevelKind::Fillet {
            for &end in &self.feature_ends {
                reach = reach.min((end - self.point).length());
            }

            for &(a, b) in &self.far_features {
                reach = reach.min(distance_to_segment(self.point, a, b));
            }

            return reach / per_size;
        }

        // the plane is linear, so it reaches a segment first at one of its ends
        let ends = self
            .feature_ends
            .iter()
            .copied()
            .chain(self.far_features.iter().flat_map(|&(a, b)| [a, b]));
        for x in ends {
            let depth = (x - self.point).dot(inward);
            if depth > 0.0 {
                reach = reach.min(depth);
            }
        }

        reach / per_size
    }

    /// C# `RingReach`: how far from the point, per unit of size, the ring reaches at most - what
    /// another point's ring must stay clear of. A fillet's ring is the sphere of radius `per_size`.
    /// A chamfer's plane crosses each of the point's faces where the depth runs out, farthest along
    /// the edge least steep into the plane.
    pub fn ring_reach(&self, kind: PointBevelKind, per_size: f64, inward: Vector3) -> f64 {
        if kind == PointBevelKind::Fillet {
            return per_size;
        }

        // C#'s LINQ `Min` throws on an empty star; a point with no neighbours has no ring to
        // reach anything, which the infinity below says (divergence 5036).
        let least = self
            .neighbours
            .iter()
            .map(|&n| (n - self.point).get_normal().dot(inward))
            .fold(f64::NAN, |least: f64, d| {
                if least.is_nan() || d < least {
                    d
                } else {
                    least
                }
            });
        if least > 0.0 {
            per_size / least
        } else {
            f64::INFINITY
        }
    }
}

/// C# `Enumerable.Average` over doubles: the sum in order, over the count. An empty sequence is
/// NaN here (C# throws; divergence 5036), which the caller's `!(x > 0)` refuses.
fn average(values: impl Iterator<Item = f64>) -> f64 {
    let mut sum = 0.0;
    let mut count = 0usize;
    for value in values {
        sum += value;
        count += 1;
    }

    if count == 0 {
        f64::NAN
    } else {
        sum / count as f64
    }
}

/// C# `DistanceToSegment`: the 3D distance from `p` to the segment ab. Not agg's
/// `Vector3.DistanceToSegment`: that one measures across the segment against a perpendicular taken
/// with +Z, so it reads short in 3D.
fn distance_to_segment(p: Vector3, a: Vector3, b: Vector3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

/// C# `Solve`: least squares for x in rows . x = 1 (rhs = sum of rows) or the fillet's -1 (rhs =
/// -sum): (sum r r^T) x = rhs.
fn solve(rows: &[Vector3], rhs: Vector3) -> Option<Vector3> {
    let mut c0 = Vector3::ZERO;
    let mut c1 = Vector3::ZERO;
    let mut c2 = Vector3::ZERO;
    for &r in rows {
        c0 += r * r.x;
        c1 += r * r.y;
        c2 += r * r.z;
    }

    let det = c0.dot(c1.cross(c2));
    if det.abs() < 1e-12 {
        return None;
    }

    Some(
        Vector3::new(
            rhs.dot(c1.cross(c2)),
            c0.dot(rhs.cross(c2)),
            c0.dot(c1.cross(rhs)),
        ) / det,
    )
}
