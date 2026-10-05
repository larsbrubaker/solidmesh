//! The triangle hit test `TriangleShape` and `MinimalTriangle` share (agg-sharp
//! `RayTracer/Primitive/Shapes/TriangleShape.cs` and `MinimalTriangle.cs` carry identical
//! copies): the plane from the corners, the major axis the triangle is projected along, the
//! projected bounds, the 2D side-of-line test and the surface area. The two C# copies differ in
//! one place, the side-of-line test (`< 0` versus `<= 0`), which [`EdgeRule`] keeps apart.

use std::cmp::Ordering;

use vector_math::cs_math::{max_f32, min_f32};
use vector_math::{IntersectionType, PlaneFloat, Ray, Vector2, Vector3, Vector3Float};

/// The projected axes for each major axis (C# `xMapping`, `yMapping`).
const X_MAPPING: [usize; 3] = [1, 0, 0];
const Y_MAPPING: [usize; 3] = [2, 2, 1];

/// Which side of an edge a point exactly on it counts as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EdgeRule {
    /// `TriangleShape`: cross `< 0` is side 1.
    Strict,
    /// `MinimalTriangle`: cross `<= 0` is side 1.
    Inclusive,
}

/// The precomputed projection of a triangle (C# `MajorAxis`, `boundsOnMajorAxis`, `Plane`).
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

/// C# `double.CompareTo`, which orders NaN below everything (the key `OrderBy` sorts on).
fn cs_compare(a: f64, b: f64) -> Ordering {
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
            if cs_compare(normal_lengths[axis], normal_lengths[major_axis]) != Ordering::Less {
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

    /// The hit of `ray` on the triangle with `corners`: distance, position and normal (C#
    /// `GetClosestIntersection` up to building the `IntersectInfo`).
    pub fn hit(
        &self,
        ray: &Ray,
        corners: &dyn Fn(usize) -> Vector3Float,
        rule: EdgeRule,
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
        let have_hit_in_2d = match self.major_axis {
            0 => self.check_2d_hit(hit_position.y, hit_position.z, corners, rule),
            1 => self.check_2d_hit(hit_position.x, hit_position.z, corners, rule),
            _ => self.check_2d_hit(hit_position.x, hit_position.y, corners, rule),
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
        corners: &dyn Fn(usize) -> Vector3Float,
        rule: EdgeRule,
    ) -> bool {
        // check the bounding rect
        if x >= f64::from(self.left)
            && x <= f64::from(self.right)
            && y >= f64::from(self.bottom)
            && y <= f64::from(self.top)
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
            let mut sum_of_line_sides = find_side_of_line(vertex0, vertex1, hit_position, rule);
            sum_of_line_sides += find_side_of_line(vertex1, vertex2, hit_position, rule);
            sum_of_line_sides += find_side_of_line(vertex2, vertex0, hit_position, rule);
            if sum_of_line_sides == -3 || sum_of_line_sides == 3 {
                return true;
            }
        }

        false
    }
}

/// 1 or -1 for the side of the line `side_point0 -> side_point1` that `test_position` is on
/// (C# `FindSideOfLine`).
pub(crate) fn find_side_of_line(
    side_point0: Vector2,
    side_point1: Vector2,
    test_position: Vector2,
    rule: EdgeRule,
) -> i32 {
    let cross = (test_position - side_point0).cross(side_point1 - side_point0);
    let on_side_one = match rule {
        EdgeRule::Strict => cross < 0.0,
        EdgeRule::Inclusive => cross <= 0.0,
    };
    if on_side_one {
        1
    } else {
        -1
    }
}

/// Half the length of the summed corner cross products (C# `GetSurfaceArea`).
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
