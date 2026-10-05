//! `PlaneFloat`: the f32 variant of `Plane` (`plane.rs`), for mesh faces. Ports agg-sharp
//! `VectorMath/PlaneFloat.cs`. Arithmetic stays in f32 as in the C#; a `Ray`'s f64 origin and
//! direction are narrowed before use.
//!
//! Mapping notes: the C# properties have private setters, so the fields are private with
//! `normal()`/`distance_from_origin()` getters. `out` results come back in tuples;
//! `GetDistanceToIntersection(Vector3Float, Vector3Float)` is
//! `get_distance_to_line_intersection`.

use crate::{Ray, Vector3Float};

/// An f32 plane (C# `MatterHackers.VectorMath.PlaneFloat`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PlaneFloat {
    distance_from_origin: f32,
    normal: Vector3Float,
}

const TREAT_AS_ZERO: f32 = 0.000001;

impl PlaneFloat {
    /// A plane with the unit-length `plane_normal` at `distance_from_origin`.
    pub fn new(plane_normal: Vector3Float, distance_from_origin: f32) -> Self {
        PlaneFloat {
            normal: plane_normal.get_normal(),
            distance_from_origin,
        }
    }

    /// The plane through three points (normal by the right-hand rule).
    pub fn from_points(point0: Vector3Float, point1: Vector3Float, point2: Vector3Float) -> Self {
        let normal = (point1 - point0).cross(point2 - point0).get_normal();
        PlaneFloat {
            normal,
            distance_from_origin: normal.dot(point0),
        }
    }

    /// The distance of the plane from the origin along its normal.
    pub fn distance_from_origin(&self) -> f32 {
        self.distance_from_origin
    }

    /// The unit normal.
    pub fn normal(&self) -> Vector3Float {
        self.normal
    }

    /// The signed distance of `position_to_check` from the plane (positive on the normal side).
    pub fn get_distance_from_plane(&self, position_to_check: Vector3Float) -> f32 {
        let distance_to_point_from_origin = position_to_check.dot(self.normal);
        distance_to_point_from_origin - self.distance_from_origin
    }

    /// The distance along the ray to the plane (infinity when parallel) and whether the ray hits
    /// the front.
    pub fn get_distance_to_intersection(&self, ray: &Ray) -> (f32, bool) {
        let normal_dot_ray_direction = self
            .normal
            .dot(Vector3Float::from_vector3(ray.direction_normal));
        if normal_dot_ray_direction < TREAT_AS_ZERO && normal_dot_ray_direction > -TREAT_AS_ZERO {
            // the ray is parallel to the plane
            return (f32::INFINITY, false);
        }

        let in_front = normal_dot_ray_direction < 0.0;

        (
            (self.distance_from_origin - self.normal.dot(Vector3Float::from_vector3(ray.origin)))
                / normal_dot_ray_direction,
            in_front,
        )
    }

    /// The distance along a line to the plane (infinity when parallel).
    pub fn get_distance_to_line_intersection(
        &self,
        point_on_line: Vector3Float,
        line_direction: Vector3Float,
    ) -> f32 {
        let normal_dot_ray_direction = self.normal.dot(line_direction);
        if normal_dot_ray_direction < TREAT_AS_ZERO && normal_dot_ray_direction > -TREAT_AS_ZERO {
            // the ray is parallel to the plane
            return f32::INFINITY;
        }

        let plane_normal_dot_point_on_line = self.normal.dot(point_on_line);
        (self.distance_from_origin - plane_normal_dot_point_on_line) / normal_dot_ray_direction
    }

    /// Whether the ray hits the plane, the distance to the hit (infinity when not) and whether
    /// it hits the front.
    pub fn ray_hit_plane(&self, ray: &Ray) -> (bool, f32, bool) {
        let normal_dot_ray_direction = self
            .normal
            .dot(Vector3Float::from_vector3(ray.direction_normal));
        if normal_dot_ray_direction < TREAT_AS_ZERO && normal_dot_ray_direction > -TREAT_AS_ZERO {
            // the ray is parallel to the plane
            return (false, f32::INFINITY, false);
        }

        let hit_front_of_plane = normal_dot_ray_direction < 0.0;

        let distance_to_ray_origin_from_origin =
            self.normal.dot(Vector3Float::from_vector3(ray.origin));

        let distance_to_plane_from_ray_origin =
            self.distance_from_origin - distance_to_ray_origin_from_origin;

        let origin_in_front_of_plane = distance_to_plane_from_ray_origin < 0.0;
        let origin_and_hit_are_on_same_side = origin_in_front_of_plane == hit_front_of_plane;
        if !origin_and_hit_are_on_same_side {
            return (false, f32::INFINITY, hit_front_of_plane);
        }

        (
            true,
            distance_to_plane_from_ray_origin / normal_dot_ray_direction,
            hit_front_of_plane,
        )
    }
}
