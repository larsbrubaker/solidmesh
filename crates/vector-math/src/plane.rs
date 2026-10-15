//! `Plane`: an f64 plane `normal . p = distance_from_origin`. `Frustum` is six of these (normals
//! pointing out); `PlaneFloat` (`plane_float.rs`) is the f32 variant.
//!
//! `normal` and `distance_from_origin` are public fields. The constructors are
//! `new(normal, distance)`, `from_points` (and `from_points_f32`) and `from_normal_and_point`.
//! `equals` compares within the default tolerances (distance .01, normal .0001);
//! `equals_within` takes them. `Plane` does not implement `PartialEq`, since a tolerance test
//! is not an equivalence. Multiple results come back in tuples;
//! `get_distance_to_line_intersection` takes a segment's two ends; `clip_line` shortens its
//! ends in place.

use crate::number_format::format_f64_max_decimals;
use crate::{AxisAlignedBoundingBox, Matrix4X4, Ray, Vector3, Vector3Float};
use std::fmt;
use std::ops::{Index, IndexMut};

/// A plane.
#[derive(Debug, Clone, Copy, Default)]
pub struct Plane {
    pub distance_from_origin: f64,
    pub normal: Vector3,
}

const TREAT_AS_ZERO: f64 = 0.000000001;
/// The distance tolerance of [`Plane::equals`].
const DEFAULT_DISTANCE_ERROR: f64 = 0.01;
/// The per-component normal tolerance of [`Plane::equals`].
const DEFAULT_NORMAL_ERROR: f64 = 0.0001;

impl Plane {
    /// A plane with the unit-length `plane_normal` at `distance_from_origin`.
    pub fn new(plane_normal: Vector3, distance_from_origin: f64) -> Self {
        Plane {
            normal: plane_normal.get_normal(),
            distance_from_origin,
        }
    }

    /// The plane through three points (normal by the right-hand rule).
    pub fn from_points(point0: Vector3, point1: Vector3, point2: Vector3) -> Self {
        let normal = (point1 - point0).cross(point2 - point0).get_normal();
        Plane {
            normal,
            distance_from_origin: normal.dot(point0),
        }
    }

    /// The plane through three f32 points (the normal is found in f32, then widened).
    pub fn from_points_f32(
        point0: Vector3Float,
        point1: Vector3Float,
        point2: Vector3Float,
    ) -> Self {
        let normal =
            Vector3::from_vector3_float((point1 - point0).cross(point2 - point0).get_normal());
        Plane {
            normal,
            distance_from_origin: normal.dot(Vector3::from_vector3_float(point0)),
        }
    }

    /// The plane with normal `plane_normal` through `point_on_plane`. The distance uses the
    /// normal as given, not the normalized one, so pass a unit normal.
    pub fn from_normal_and_point(plane_normal: Vector3, point_on_plane: Vector3) -> Self {
        Plane {
            normal: plane_normal.get_normal(),
            distance_from_origin: plane_normal.dot(point_on_plane),
        }
    }

    /// True when the box touches the plane, within `error`.
    pub fn crossed_by(&self, aabb: &AxisAlignedBoundingBox, error: f64) -> bool {
        // Convert AABB to center-extents representation
        let center = aabb.center();
        let delta_to_max = aabb.max_xyz - center; // Compute positive extents

        // Compute the projection interval radius of aabb onto L(t) = aabb.center + t * plane.normal
        let distance_to_max = delta_to_max.x * self.normal.x.abs()
            + delta_to_max.y * self.normal.y.abs()
            + delta_to_max.z * self.normal.z.abs();

        // Compute distance of box center from plane
        let distance_to_center = self.normal.dot(center) - self.distance_from_origin;

        // Intersection occurs when distance s falls within [-r,+r] interval
        distance_to_center.abs() <= distance_to_max + error
    }

    /// Within .01 in distance and .0001 per normal component.
    pub fn equals(&self, other_plane: &Plane) -> bool {
        self.equals_within(other_plane, DEFAULT_DISTANCE_ERROR, DEFAULT_NORMAL_ERROR)
    }

    /// True when the distances are strictly within `distance_error_value` and every normal
    /// component strictly within `normal_error_value`.
    pub fn equals_within(
        &self,
        other_plane: &Plane,
        distance_error_value: f64,
        normal_error_value: f64,
    ) -> bool {
        self.distance_from_origin < other_plane.distance_from_origin + distance_error_value
            && self.distance_from_origin > other_plane.distance_from_origin - distance_error_value
            && self
                .normal
                .equals_within(other_plane.normal, normal_error_value)
    }

    /// The signed distance of an f32 position (widened) from the plane.
    pub fn get_distance_from_plane_f32(&self, position_to_check: Vector3Float) -> f64 {
        self.get_distance_from_plane(Vector3::from_vector3_float(position_to_check))
    }

    /// The signed distance of `position_to_check` from the plane (positive on the normal side).
    pub fn get_distance_from_plane(&self, position_to_check: Vector3) -> f64 {
        let distance_to_position = self.normal.dot(position_to_check);
        distance_to_position - self.distance_from_origin
    }

    /// Clips the segment to the normal side of the plane; false if it is entirely behind.
    pub fn clip_line(&self, start_point: &mut Vector3, end_point: &mut Vector3) -> bool {
        let start_distance = self.get_distance_from_plane(*start_point);
        let end_distance = self.get_distance_from_plane(*end_point);

        if start_distance < 0.0 {
            if end_distance < 0.0 {
                // both points are behind the plane
                return false;
            }

            // the start point must be clipped
            // get the normal in the direction of the start point
            let line_direction = *start_point - *end_point;
            let line_length = line_direction.length();
            let line_normal = line_direction / line_length;
            let start_clip_ratio = start_distance / (end_distance - start_distance);
            *start_point += line_normal * (line_length * start_clip_ratio);
        } else if end_distance < 0.0 {
            // the end point must be clipped
            // get the normal in the direction of the start point
            let line_direction = *end_point - *start_point;
            let line_length = line_direction.length();
            let line_normal = line_direction / line_length;
            let end_clip_ratio = end_distance / (start_distance - end_distance);
            *end_point += line_normal * (line_length * end_clip_ratio);
        }

        // both points in front of the plane
        true
    }

    /// The plane moved by `matrix`: the normal through `transform_vector`, a point on the plane
    /// through `transform`.
    pub fn transform(&self, matrix: Matrix4X4) -> Plane {
        let plane_normal = self.normal;
        let distance_to_plane = self.distance_from_origin;

        let normal = plane_normal.transform_vector(matrix).get_normal();
        let point_on_plane = plane_normal * distance_to_plane;
        let point_on_transformed_plane = point_on_plane.transform(matrix);
        Plane {
            normal,
            distance_from_origin: normal.dot(point_on_transformed_plane),
        }
    }

    /// The distance along the ray to the plane (infinity when parallel) and whether the ray hits
    /// the front.
    pub fn get_distance_to_intersection(&self, ray: &Ray) -> (f64, bool) {
        let normal_dot_ray_direction = self.normal.dot(ray.direction_normal);
        if normal_dot_ray_direction < TREAT_AS_ZERO && normal_dot_ray_direction > -TREAT_AS_ZERO {
            // the ray is parallel to the plane
            return (f64::INFINITY, false);
        }

        let in_front = normal_dot_ray_direction < 0.0;

        (
            (self.distance_from_origin - self.normal.dot(ray.origin)) / normal_dot_ray_direction,
            in_front,
        )
    }

    /// The distance along a line to the plane (infinity when parallel).
    pub fn get_distance_to_line_intersection(
        &self,
        point_on_line: Vector3,
        line_direction: Vector3,
    ) -> f64 {
        let normal_dot_ray_direction = self.normal.dot(line_direction);
        if normal_dot_ray_direction < TREAT_AS_ZERO && normal_dot_ray_direction > -TREAT_AS_ZERO {
            // the ray is parallel to the plane
            return f64::INFINITY;
        }

        let plane_normal_dot_point_on_line = self.normal.dot(point_on_line);
        (self.distance_from_origin - plane_normal_dot_point_on_line) / normal_dot_ray_direction
    }

    /// Whether the ray hits the plane, the distance to the hit (infinity when not) and whether
    /// it hits the front.
    pub fn ray_hit_plane(&self, ray: &Ray) -> (bool, f64, bool) {
        let normal_dot_ray_direction = self.normal.dot(ray.direction_normal);
        if normal_dot_ray_direction < TREAT_AS_ZERO && normal_dot_ray_direction > -TREAT_AS_ZERO {
            // the ray is parallel to the plane
            return (false, f64::INFINITY, false);
        }

        let hit_front_of_plane = normal_dot_ray_direction < 0.0;

        let distance_to_ray_origin_from_origin = self.normal.dot(ray.origin);

        let distance_to_plane_from_ray_origin =
            self.distance_from_origin - distance_to_ray_origin_from_origin;

        let origin_in_front_of_plane = distance_to_plane_from_ray_origin < 0.0;
        let origin_and_hit_are_on_same_side = origin_in_front_of_plane == hit_front_of_plane;
        if !origin_and_hit_are_on_same_side {
            return (false, f64::INFINITY, hit_front_of_plane);
        }

        (
            true,
            distance_to_plane_from_ray_origin / normal_dot_ray_direction,
            hit_front_of_plane,
        )
    }

    /// Scales the normal to unit length, scaling the distance alike.
    pub fn normalize(&mut self) {
        let length = self.normal.length();
        self.normal /= length;
        self.distance_from_origin /= length;
    }

    /// Where the segment `start`-`end` crosses the plane, if it does (the miss position is
    /// positive infinity). An end lying on the plane is the crossing; otherwise the ends must
    /// be on opposite sides, judged by their signed distances from this plane.
    pub fn line_hit_plane(&self, start: Vector3, end: Vector3) -> (bool, Vector3) {
        let start_distance_from_plane = self.normal.dot(start) - self.distance_from_origin;
        if start_distance_from_plane == 0.0 {
            return (true, start);
        }

        let end_distance_from_plane = self.normal.dot(end) - self.distance_from_origin;
        if end_distance_from_plane == 0.0 {
            return (true, end);
        }

        if (start_distance_from_plane < 0.0 && end_distance_from_plane > 0.0)
            || (start_distance_from_plane > 0.0 && end_distance_from_plane < 0.0)
        {
            // The signed distance changes linearly along the segment, so the crossing is the
            // fraction of the segment where it reaches zero.
            let ratio_from_start =
                start_distance_from_plane / (start_distance_from_plane - end_distance_from_plane);
            return (true, start + (end - start) * ratio_from_start);
        }

        (false, Vector3::POSITIVE_INFINITY)
    }
}

/// 0..=2 are the normal's x, y, z; any other index is the distance.
impl Index<usize> for Plane {
    type Output = f64;
    fn index(&self, index: usize) -> &f64 {
        match index {
            0..=2 => &self.normal[index],
            _ => &self.distance_from_origin,
        }
    }
}

impl IndexMut<usize> for Plane {
    fn index_mut(&mut self, index: usize) -> &mut f64 {
        match index {
            0..=2 => &mut self.normal[index],
            _ => &mut self.distance_from_origin,
        }
    }
}

/// "D:d V:xX, yY, zZ" with each number in the `0.###` format.
impl fmt::Display for Plane {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "D:{} V:x{}, y{}, z{}",
            format_f64_max_decimals(self.distance_from_origin, 3),
            format_f64_max_decimals(self.normal.x, 3),
            format_f64_max_decimals(self.normal.y, 3),
            format_f64_max_decimals(self.normal.z, 3)
        )
    }
}
