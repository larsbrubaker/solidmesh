//! An infinite plane (agg-sharp `RayTracer/Primitive/Shapes/PlaneShape.cs`, with the
//! `BaseShape.cs` members it inherits). MatterCAD's drag handles trace their mouse rays against
//! these (the bed plane, and planes through the grabbed point). MatterCAD passes a `null`
//! material everywhere it builds one, so it keeps none (divergence 530).

use std::any::Any;

use vector_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

use super::base_shape::{leaf_get_crossing, leaf_get_touching};
use super::{IntersectInfo, RayTracerError, Traceable};

/// An infinite plane (C# `PlaneShape`).
#[derive(Debug, Clone, Copy)]
pub struct PlaneShape {
    pub plane: Plane,
}

impl PlaneShape {
    /// C# `PlaneShape(Plane plane, MaterialAbstract material)`.
    pub fn new(plane: Plane) -> Self {
        PlaneShape { plane }
    }

    /// C# `PlaneShape(Vector3 planeNormal, double distanceFromOrigin, MaterialAbstract material)`.
    pub fn from_normal_and_distance(plane_normal: Vector3, distance_from_origin: f64) -> Self {
        Self::new(Plane::new(plane_normal, distance_from_origin))
    }

    fn hit_at(&self, ray: &Ray, distance_to_hit: f64) -> IntersectInfo<'_> {
        IntersectInfo {
            closest_hit_object: Some(self),
            hit_type: IntersectionType::FrontFace,
            hit_position: ray.origin + ray.direction_normal * distance_to_hit,
            normal_at_hit: self.plane.normal,
            distance_to_hit,
            ..IntersectInfo::new()
        }
    }

    /// Like `get_closest_intersection`, but considers the ray's min and max distance instead.
    pub fn get_closest_intersection_within_ray_distance_range(
        &self,
        ray: &Ray,
    ) -> Option<IntersectInfo<'_>> {
        let (distance_to_hit, _in_front) = self.plane.get_distance_to_intersection(ray);
        // This will also reject infinity.
        if ray.min_distance_to_consider < distance_to_hit
            && distance_to_hit < ray.max_distance_to_consider
        {
            return Some(self.hit_at(ray, distance_to_hit));
        }

        None
    }
}

impl std::fmt::Display for PlaneShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The C# text says "Sphere"; kept as it is.
        write!(
            f,
            "Sphere {}x+{}y+{}z+{}=0)",
            self.plane.normal.x,
            self.plane.normal.y,
            self.plane.normal.z,
            self.plane.distance_from_origin
        )
    }
}

impl Traceable for PlaneShape {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        // C# BaseShape throws here; a leaf is never asked (see divergence 141).
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        f64::INFINITY
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::new(Vector3::NEGATIVE_INFINITY, Vector3::POSITIVE_INFINITY)
    }

    fn get_center(&self) -> Vector3 {
        self.get_axis_aligned_bounding_box().get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.get_center()[axis]
    }

    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        if self.get_axis_aligned_bounding_box().contains(sub_region) {
            results.push(self);
            return true;
        }

        false
    }

    fn contains(&self, position: Vector3) -> bool {
        self.get_axis_aligned_bounding_box()
            .contains_point(position)
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let (distance_to_hit, _in_front) = self.plane.get_distance_to_intersection(ray);
        if distance_to_hit > 0.0 {
            return Some(self.hit_at(ray, distance_to_hit));
        }

        None
    }

    fn get_intersect_cost(&self) -> f64 {
        350.0
    }

    fn intersection_iterator<'a>(
        &'a self,
        _ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        Err(RayTracerError::NotSupported(
            "PlaneShape.IntersectionIterator",
        ))
    }

    fn get_crossing(&self, plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Ok(leaf_get_crossing(self, plane))
    }

    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Ok(leaf_get_touching(self, position, error))
    }
}
