//! A traceable seen through a matrix, with its axis-to-world transform and the inverse. MatterCAD
//! wraps each object's cached mesh BVH in one of these with the object's matrix, so one tree serves
//! every placement: rays go into the child's space, hits come back out.

use std::any::Any;

use solidmesh_math::{AxisAlignedBoundingBox, Matrix4X4, Plane, Ray, Vector3};

use super::{IntersectInfo, RayTracerError, Traceable, TraceableRef};

/// A child traceable under a transform.
pub struct Transform {
    child: TraceableRef,
    axis_to_world: Matrix4X4,
    world_to_axis: Matrix4X4,
}

impl Transform {
    /// The child untransformed.
    pub fn new(root: TraceableRef) -> Self {
        Transform {
            child: root,
            axis_to_world: Matrix4X4::IDENTITY,
            world_to_axis: Matrix4X4::IDENTITY,
        }
    }

    /// The child moved by `transform`.
    pub fn with_matrix(root: TraceableRef, transform: Matrix4X4) -> Self {
        Transform {
            child: root,
            axis_to_world: transform,
            world_to_axis: transform.inverted(),
        }
    }

    /// The transformed traceable.
    pub fn child(&self) -> &TraceableRef {
        &self.child
    }

    /// The inverse of [`Traceable::axis_to_world`].
    pub fn world_to_axis(&self) -> Matrix4X4 {
        self.world_to_axis
    }

    fn get_local_space_ray(&self, ray: &Ray) -> Ray {
        // The inverse is computed for every ray; caching it would save that work.
        Ray::transform(ray, self.axis_to_world.inverted())
    }

    fn get_global_space_info<'a>(&self, local_info: IntersectInfo<'a>) -> IntersectInfo<'a> {
        let mut global_info = local_info;
        global_info.hit_position = local_info
            .hit_position
            .transform_position(self.axis_to_world);
        global_info.normal_at_hit = local_info
            .normal_at_hit
            .transform_vector(self.axis_to_world);
        global_info
    }
}

impl Traceable for Transform {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        Some(vec![self.child.as_ref()])
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        self.axis_to_world
    }

    fn get_surface_area(&self) -> f64 {
        self.child.get_surface_area()
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        let local_bounds = self.child.get_axis_aligned_bounding_box();
        local_bounds.new_transformed(self.axis_to_world)
    }

    fn get_center(&self) -> Vector3 {
        self.get_axis_aligned_bounding_box().get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.get_center()[axis]
    }

    /// The child's contained items, tested against `sub_region` untransformed (the region is
    /// not moved into the child's space).
    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        self.child.get_contained(results, sub_region);
        true
    }

    fn contains(&self, position: Vector3) -> bool {
        if self
            .get_axis_aligned_bounding_box()
            .contains_point(position)
        {
            let child_position = position.transform(self.world_to_axis);
            if self.child.contains(child_position) {
                return true;
            }
        }

        false
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let mut local_ray = self.get_local_space_ray(ray);
        let local_intersection = self.child.get_closest_intersection(&mut local_ray)?;
        Some(self.get_global_space_info(local_intersection))
    }

    fn get_intersect_cost(&self) -> f64 {
        self.child.get_intersect_cost()
    }

    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        let local_ray = self.get_local_space_ray(ray);
        Ok(self
            .child
            .intersection_iterator(&local_ray)?
            .into_iter()
            .map(|local_info| self.get_global_space_info(local_info))
            .collect())
    }

    fn get_crossing(&self, _plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Err(RayTracerError::NotSupported("Transform.GetCrossing"))
    }

    fn get_touching(
        &self,
        _position: Vector3,
        _error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Err(RayTracerError::NotSupported("Transform.GetTouching"))
    }
}
