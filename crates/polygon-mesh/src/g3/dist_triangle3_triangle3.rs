//! The triangle-triangle distance and the queries it is built from (geometry3Sharp's, after
//! WildMagic 5): triangle-triangle, segment-triangle, line-triangle and line-segment, with
//! [`Segment3d`]'s centre/direction/extent form and a complement basis for a direction.
//! The point-triangle query they fall back to is `tri_distance::distance_sqr_closest`.
//! `DMeshAABBTree3::find_nearest_triangles` (`dmesh_aabb_tree3_pairs.rs`) measures leaf pairs with
//! it, and `mesh_distance` reads the closest points of the pair it finds.
//!
//! Only the distances and closest points are computed; nothing reads barycentric outputs.

use super::tri_distance::distance_sqr_closest;
use super::vector3d::{add, cross, dot, length, scale, sub, EPSILON, ZERO_TOLERANCE};

/// g3 `Segment3d`: centre, unit direction and half-length.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Segment3d {
    pub(crate) center: [f64; 3],
    pub(crate) direction: [f64; 3],
    pub(crate) extent: f64,
}

impl Segment3d {
    /// The segment between two endpoints, in centre/direction/extent form.
    pub(crate) fn new(p0: [f64; 3], p1: [f64; 3]) -> Self {
        let center = scale(add(p0, p1), 0.5);
        let mut direction = sub(p1, p0);
        // Returns the length, or zeroes the vector and returns 0 at or under `MathUtil.Epsilon`.
        let mut len = length(direction);
        if len > EPSILON {
            let inv_length = 1.0 / len;
            direction = [
                direction[0] * inv_length,
                direction[1] * inv_length,
                direction[2] * inv_length,
            ];
        } else {
            len = 0.0;
            direction = [0.0; 3];
        }
        Self {
            center,
            direction,
            extent: 0.5 * len,
        }
    }

    pub(crate) fn p0(&self) -> [f64; 3] {
        sub(self.center, scale(self.direction, self.extent))
    }

    pub(crate) fn p1(&self) -> [f64; 3] {
        add(self.center, scale(self.direction, self.extent))
    }
}

/// g3 `Line3d`.
#[derive(Debug, Clone, Copy)]
struct Line3d {
    origin: [f64; 3],
    direction: [f64; 3],
}

/// The answer of a distance query: squared distance and the closest point on each operand.
#[derive(Debug, Clone, Copy)]
pub(crate) struct TriTriDistance {
    pub(crate) distance_squared: f64,
    pub(crate) triangle0_closest: [f64; 3],
    pub(crate) triangle1_closest: [f64; 3],
}

/// The squared distance between two triangles, with its closest points.
pub(crate) fn dist_triangle3_triangle3(
    triangle0: &[[f64; 3]; 3],
    triangle1: &[[f64; 3]; 3],
) -> TriTriDistance {
    let mut result = TriTriDistance {
        distance_squared: f64::MAX,
        triangle0_closest: [0.0; 3],
        triangle1_closest: [0.0; 3],
    };

    // Compare edges of triangle0 to the interior of triangle1.
    let mut sqr_dist = f64::MAX;
    let (mut i0, mut i1) = (2, 0);
    while i1 < 3 {
        let edge = Segment3d::new(triangle0[i0], triangle0[i1]);
        let query = dist_segment3_triangle3(&edge, triangle1);
        if query.distance_squared < sqr_dist {
            result.triangle0_closest = query.segment_closest;
            result.triangle1_closest = query.triangle_closest;
            sqr_dist = query.distance_squared;
            if sqr_dist <= ZERO_TOLERANCE {
                result.distance_squared = 0.0;
                return result;
            }
        }
        i0 = i1;
        i1 += 1;
    }

    // Compare edges of triangle1 to the interior of triangle0. Here the segment is triangle1's
    // edge, so its closest point is triangle1's and is stored as triangle1's; storing it the other
    // way would put both "closest points" on the wrong triangles.
    let (mut i0, mut i1) = (2, 0);
    while i1 < 3 {
        let edge = Segment3d::new(triangle1[i0], triangle1[i1]);
        let query = dist_segment3_triangle3(&edge, triangle0);
        if query.distance_squared < sqr_dist {
            result.triangle0_closest = query.triangle_closest;
            result.triangle1_closest = query.segment_closest;
            sqr_dist = query.distance_squared;
            if sqr_dist <= ZERO_TOLERANCE {
                result.distance_squared = 0.0;
                return result;
            }
        }
        i0 = i1;
        i1 += 1;
    }

    result.distance_squared = sqr_dist;
    result
}

/// What a segment or line query answers.
#[derive(Debug, Clone, Copy)]
struct SegmentTriangle {
    distance_squared: f64,
    segment_closest: [f64; 3],
    triangle_closest: [f64; 3],
}

fn dist_segment3_triangle3(segment: &Segment3d, triangle: &[[f64; 3]; 3]) -> SegmentTriangle {
    let line = Line3d {
        origin: segment.center,
        direction: segment.direction,
    };
    let query = dist_line3_triangle3(&line, triangle);
    let segment_param = query.line_param;

    if segment_param >= -segment.extent {
        if segment_param <= segment.extent {
            SegmentTriangle {
                distance_squared: query.distance_squared,
                segment_closest: query.line_closest,
                triangle_closest: query.triangle_closest,
            }
        } else {
            let segment_closest = segment.p1();
            let (sqr_dist, triangle_closest) = distance_sqr_closest(triangle, segment_closest);
            SegmentTriangle {
                distance_squared: sqr_dist,
                segment_closest,
                triangle_closest,
            }
        }
    } else {
        let segment_closest = segment.p0();
        let (sqr_dist, triangle_closest) = distance_sqr_closest(triangle, segment_closest);
        SegmentTriangle {
            distance_squared: sqr_dist,
            segment_closest,
            triangle_closest,
        }
    }
}

struct LineTriangle {
    distance_squared: f64,
    line_closest: [f64; 3],
    line_param: f64,
    triangle_closest: [f64; 3],
}

/// The normalized cross product (zero at or under `MathUtil.Epsilon`).
fn unit_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    super::vector3d::normalized(cross(a, b))
}

/// `u`, `v` completing the unit `w` to an orthonormal basis.
fn generate_complement_basis(w: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let mut u = [0.0; 3];
    let mut v = [0.0; 3];
    if w[0].abs() >= w[1].abs() {
        // W.x or W.z is the largest magnitude component, swap them
        let inv_length = 1.0 / (w[0] * w[0] + w[2] * w[2]).sqrt();
        u[0] = -w[2] * inv_length;
        u[1] = 0.0;
        u[2] = w[0] * inv_length;
        v[0] = w[1] * u[2];
        v[1] = w[2] * u[0] - w[0] * u[2];
        v[2] = -w[1] * u[0];
    } else {
        // W.y or W.z is the largest magnitude component, swap them
        let inv_length = 1.0 / (w[1] * w[1] + w[2] * w[2]).sqrt();
        u[0] = 0.0;
        u[1] = w[2] * inv_length;
        u[2] = -w[1] * inv_length;
        v[0] = w[1] * u[2] - w[2] * u[1];
        v[1] = -w[0] * u[2];
        v[2] = w[0] * u[1];
    }
    (u, v)
}

fn dist_line3_triangle3(line: &Line3d, triangle: &[[f64; 3]; 3]) -> LineTriangle {
    // Test if line intersects triangle.  If so, the squared distance is zero.
    let edge0 = sub(triangle[1], triangle[0]);
    let edge1 = sub(triangle[2], triangle[0]);
    let normal = unit_cross(edge0, edge1);
    let nd_d = dot(normal, line.direction);
    if nd_d.abs() > ZERO_TOLERANCE {
        // The line and triangle are not parallel, so the line intersects
        // the plane of the triangle.
        let diff = sub(line.origin, triangle[0]);
        let (u, v) = generate_complement_basis(line.direction);
        let ud_e0 = dot(u, edge0);
        let ud_e1 = dot(u, edge1);
        let ud_diff = dot(u, diff);
        let vd_e0 = dot(v, edge0);
        let vd_e1 = dot(v, edge1);
        let vd_diff = dot(v, diff);
        let inv_det = 1.0 / (ud_e0 * vd_e1 - ud_e1 * vd_e0);

        // Barycentric coordinates for the point of intersection.
        let b1 = (vd_e1 * ud_diff - ud_e1 * vd_diff) * inv_det;
        let b2 = (ud_e0 * vd_diff - vd_e0 * ud_diff) * inv_det;
        let b0 = 1.0 - b1 - b2;

        if b0 >= 0.0 && b1 >= 0.0 && b2 >= 0.0 {
            // Line parameter for the point of intersection.
            let dd_e0 = dot(line.direction, edge0);
            let dd_e1 = dot(line.direction, edge1);
            let dd_diff = dot(line.direction, diff);
            let line_param = b1 * dd_e0 + b2 * dd_e1 - dd_diff;

            // The intersection point is inside or on the triangle.
            return LineTriangle {
                distance_squared: 0.0,
                line_closest: add(line.origin, scale(line.direction, line_param)),
                line_param,
                triangle_closest: add(add(triangle[0], scale(edge0, b1)), scale(edge1, b2)),
            };
        }
    }

    // Either (1) the line is not parallel to the triangle and the point of
    // intersection of the line and the plane of the triangle is outside the
    // triangle or (2) the line and triangle are parallel.  Regardless, the
    // closest point on the triangle is on an edge of the triangle.  Compare
    // the line to all three edges of the triangle.
    let mut result = LineTriangle {
        distance_squared: f64::MAX,
        line_closest: [0.0; 3],
        line_param: 0.0,
        triangle_closest: [0.0; 3],
    };
    let (mut i0, mut i1) = (2, 0);
    while i1 < 3 {
        let segment = Segment3d::new(triangle[i0], triangle[i1]);
        let query = dist_line3_segment3(line, &segment);
        if query.distance_squared < result.distance_squared {
            result.line_closest = query.line_closest;
            result.triangle_closest = query.segment_closest;
            result.distance_squared = query.distance_squared;
            result.line_param = query.line_parameter;
        }
        i0 = i1;
        i1 += 1;
    }
    result
}

struct LineSegment {
    distance_squared: f64,
    line_closest: [f64; 3],
    line_parameter: f64,
    segment_closest: [f64; 3],
}

fn dist_line3_segment3(line: &Line3d, segment: &Segment3d) -> LineSegment {
    let diff = sub(line.origin, segment.center);
    let a01 = -dot(line.direction, segment.direction);
    let b0 = dot(diff, line.direction);
    let c = dot(diff, diff);
    let det = (1.0 - a01 * a01).abs();
    let (s0, s1, mut sqr_dist);

    if det >= ZERO_TOLERANCE {
        // The line and segment are not parallel.
        let b1 = -dot(diff, segment.direction);
        let raw_s1 = a01 * b0 - b1;
        let ext_det = segment.extent * det;

        if raw_s1 >= -ext_det {
            if raw_s1 <= ext_det {
                // Two interior points are closest, one on the line and one
                // on the segment.
                let inv_det = 1.0 / det;
                s0 = (a01 * b1 - b0) * inv_det;
                s1 = raw_s1 * inv_det;
                sqr_dist = s0 * (s0 + a01 * s1 + 2.0 * b0) + s1 * (a01 * s0 + s1 + 2.0 * b1) + c;
            } else {
                // The endpoint e1 of the segment and an interior point of
                // the line are closest.
                s1 = segment.extent;
                s0 = -(a01 * s1 + b0);
                sqr_dist = -s0 * s0 + s1 * (s1 + 2.0 * b1) + c;
            }
        } else {
            // The end point e0 of the segment and an interior point of the
            // line are closest.
            s1 = -segment.extent;
            s0 = -(a01 * s1 + b0);
            sqr_dist = -s0 * s0 + s1 * (s1 + 2.0 * b1) + c;
        }
    } else {
        // The line and segment are parallel.  Choose the closest pair so that
        // one point is at segment center.
        s1 = 0.0;
        s0 = -b0;
        sqr_dist = b0 * s0 + c;
    }

    // Account for numerical round-off errors.
    if sqr_dist < 0.0 {
        sqr_dist = 0.0;
    }

    LineSegment {
        distance_squared: sqr_dist,
        line_closest: add(line.origin, scale(line.direction, s0)),
        line_parameter: s0,
        segment_closest: add(segment.center, scale(segment.direction, s1)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rust_only_parallel_triangles_measure_their_gap() {
        let a = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let b = [[0.0, 0.0, 2.0], [1.0, 0.0, 2.0], [0.0, 1.0, 2.0]];
        let d = dist_triangle3_triangle3(&a, &b);
        assert!((d.distance_squared - 4.0).abs() < 1e-12);
        assert!(((d.triangle1_closest[2] - d.triangle0_closest[2]).abs() - 2.0).abs() < 1e-12);
    }

    /// The nearest pair is found from triangle1's edge (its bottom edge over triangle0's middle),
    /// and each closest point still lies on its own triangle.
    #[test]
    fn rust_only_closest_points_stay_on_their_own_triangles() {
        let a = [[-5.0, -5.0, 0.0], [5.0, -5.0, 0.0], [0.0, 5.0, 0.0]];
        let b = [[0.0, 0.0, 1.0], [0.2, 0.0, 1.0], [0.1, 0.0, 2.0]];
        let d = dist_triangle3_triangle3(&a, &b);
        assert!((d.distance_squared - 1.0).abs() < 1e-12);
        assert_eq!(d.triangle0_closest[2], 0.0);
        assert_eq!(d.triangle1_closest[2], 1.0);
    }

    #[test]
    fn rust_only_crossing_triangles_are_zero_apart() {
        let a = [[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]];
        let b = [[0.0, 0.0, -1.0], [0.0, 0.0, 1.0], [0.0, 2.0, 0.0]];
        assert_eq!(dist_triangle3_triangle3(&a, &b).distance_squared, 0.0);
    }
}
