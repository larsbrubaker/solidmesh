//! The triangle hit test `TriangleShape` and `MinimalTriangle` share: the plane from the
//! corners, the major axis the triangle is projected along, the projected bounds, the 2D
//! side-of-line test and the surface area. A point exactly on an edge counts as inside for both
//! projected windings, and a point within the plane hit's single-precision slack of an edge
//! counts as on it, so a click exactly on an edge two faces share never falls through the part.

use std::cmp::Ordering;

use vector_math::ieee_math::{max_f32, min_f32};
use vector_math::{IntersectionType, PlaneFloat, Ray, Vector2, Vector3, Vector3Float};

/// How many single-precision steps of the hit distance a hit may fall outside a triangle and
/// still count as on its edge (see `TriangleProjection::hit`).
const HIT_SLACK_FLOAT_STEPS: f64 = 4.0;

/// The projected axes for each major axis.
const X_MAPPING: [usize; 3] = [1, 0, 0];
const Y_MAPPING: [usize; 3] = [2, 2, 1];

/// The precomputed projection of a triangle: its major axis, projected bounds and plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct TriangleProjection {
    pub plane: PlaneFloat,
    pub major_axis: u8,
    /// `RectangleFloat` Left, Bottom, Right, Top.
    pub left: f32,
    pub bottom: f32,
    pub right: f32,
    pub top: f32,
}

/// A total order on doubles with NaN below everything, so a NaN normal component never wins
/// the major axis over a number.
fn nan_first_compare(a: f64, b: f64) -> Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => a.partial_cmp(&b).unwrap_or(Ordering::Equal),
    }
}

impl TriangleProjection {
    pub fn new(v0: Vector3, v1: Vector3, v2: Vector3, corners: [Vector3Float; 3]) -> Self {
        let plane_normal = (v1 - v0).cross(v2 - v0).get_normal();
        let distance_from_origin = v0.dot(plane_normal);
        let plane = PlaneFloat::new(
            Vector3Float::from_vector3(plane_normal),
            distance_from_origin as f32,
        );

        // `OrderBy(o => o.Value).Last()`: a stable sort, so the largest value wins and of
        // equal values the last axis does.
        let normal_lengths = [
            plane_normal.x.abs(),
            plane_normal.y.abs(),
            plane_normal.z.abs(),
        ];
        let mut major_axis = 0;
        for axis in 1..3 {
            if nan_first_compare(normal_lengths[axis], normal_lengths[major_axis]) != Ordering::Less
            {
                major_axis = axis;
            }
        }

        let mut projection = TriangleProjection {
            plane,
            major_axis: major_axis as u8,
            left: f32::MAX,
            bottom: f32::MAX,
            right: f32::MIN,
            top: f32::MIN,
        };
        let (x_axis, y_axis) = projection.axes();
        for corner in corners {
            projection.left = min_f32(corner[x_axis], projection.left);
            projection.right = max_f32(corner[x_axis], projection.right);
            projection.bottom = min_f32(corner[y_axis], projection.bottom);
            projection.top = max_f32(corner[y_axis], projection.top);
        }

        projection
    }

    /// The two axes the triangle is projected onto.
    pub fn axes(&self) -> (usize, usize) {
        (
            X_MAPPING[self.major_axis as usize],
            Y_MAPPING[self.major_axis as usize],
        )
    }

    /// The hit of `ray` on the triangle with `corners`: distance, position and normal, for the
    /// shape to build its `IntersectInfo` from.
    pub fn hit(
        &self,
        ray: &Ray,
        corners: &dyn Fn(usize) -> Vector3Float,
    ) -> Option<(f64, Vector3, Vector3)> {
        let (hit_plane, distance_to_hit, in_front) = self.plane.ray_hit_plane(ray);
        if !hit_plane {
            return None;
        }

        let wants =
            |face: IntersectionType| (ray.intersection_type as u8 & face as u8) == face as u8;
        let want_front_and_in_front = wants(IntersectionType::FrontFace) && in_front;
        let want_back_and_in_back = wants(IntersectionType::BackFace) && !in_front;
        if !(want_front_and_in_front || want_back_and_in_back) {
            return None;
        }

        let hit_position = ray.origin + ray.direction_normal * f64::from(distance_to_hit);
        // The plane hit is single precision (`PlaneFloat`), so the point is only
        // known to a few float steps of its distance along the ray. Two faces that share an edge
        // but project along different axes each place their own hit, and a ray onto the edge
        // can land that far outside both; within that slack the point counts as on the edge,
        // so a click exactly over such an edge (a pyramid's diagonal seen from above) hits.
        let tolerance =
            f64::from(distance_to_hit).abs() * f64::from(f32::EPSILON) * HIT_SLACK_FLOAT_STEPS;
        let have_hit_in_2d = match self.major_axis {
            0 => self.check_2d_hit(hit_position.y, hit_position.z, tolerance, corners),
            1 => self.check_2d_hit(hit_position.x, hit_position.z, tolerance, corners),
            _ => self.check_2d_hit(hit_position.x, hit_position.y, tolerance, corners),
        };

        if have_hit_in_2d {
            Some((
                f64::from(distance_to_hit),
                hit_position,
                Vector3::from_vector3_float(self.plane.normal()),
            ))
        } else {
            None
        }
    }

    fn check_2d_hit(
        &self,
        x: f64,
        y: f64,
        tolerance: f64,
        corners: &dyn Fn(usize) -> Vector3Float,
    ) -> bool {
        // check the bounding rect
        if x >= f64::from(self.left) - tolerance
            && x <= f64::from(self.right) + tolerance
            && y >= f64::from(self.bottom) - tolerance
            && y <= f64::from(self.top) + tolerance
        {
            let (x_axis, y_axis) = self.axes();
            let project = |i: usize| {
                let corner = corners(i);
                Vector2::new(f64::from(corner[x_axis]), f64::from(corner[y_axis]))
            };
            let vertex0 = project(0);
            let vertex1 = project(1);
            let vertex2 = project(2);
            let hit_position = Vector2::new(x, y);
            // Summing a ±1 sign per edge and asking for 3 or -3 would count a point exactly on
            // an edge (cross 0) for one winding only, so a ray onto an edge two triangles share
            // would miss both whenever their projection wound the other way. Instead
            // an edge only rules the point out when it lies strictly on the other side from
            // another edge: on an edge or corner, every triangle touching it reports the hit.
            // Each cross product is the point's distance from its edge times the edge's length;
            // within `tolerance` of the edge it counts as on it.
            let side = |a: Vector2, b: Vector2| {
                let cross = side_of_line(a, b, hit_position);
                if cross.abs() <= tolerance * (b - a).length() {
                    0.0
                } else {
                    cross
                }
            };
            let sides = [
                side(vertex0, vertex1),
                side(vertex1, vertex2),
                side(vertex2, vertex0),
            ];
            let on_side_one = sides.iter().any(|&cross| cross < 0.0);
            let on_side_two = sides.iter().any(|&cross| cross > 0.0);
            return !(on_side_one && on_side_two);
        }

        false
    }
}

/// The cross product whose sign says which side of a line a point is on: negative on one side
/// of the line `side_point0 -> side_point1`, positive on the other, zero on it. The line is
/// always measured from its lexicographically smaller end, so two triangles sharing an edge
/// (which run it in opposite directions) get exactly opposite values for the same point:
/// rounding can never put a point outside both.
pub(crate) fn side_of_line(
    side_point0: Vector2,
    side_point1: Vector2,
    test_position: Vector2,
) -> f64 {
    let reversed = (side_point1.x, side_point1.y) < (side_point0.x, side_point0.y);
    if reversed {
        -(test_position - side_point1).cross(side_point0 - side_point1)
    } else {
        (test_position - side_point0).cross(side_point1 - side_point0)
    }
}

/// Half the length of the summed corner cross products.
pub(crate) fn triangle_surface_area(corners: &dyn Fn(usize) -> Vector3Float) -> f64 {
    let mut accumulation = Vector3::ZERO;
    for first_index in 0..3 {
        let second_index = (first_index + 1) % 3;
        accumulation +=
            Vector3::from_vector3_float(corners(first_index).cross(corners(second_index)));
    }

    accumulation /= 2.0;
    accumulation.length()
}
