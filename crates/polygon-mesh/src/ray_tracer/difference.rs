//! View-space CSG: one traceable with another cut away. Nothing is built; every ray collects
//! all front and back faces of both operands and [`IntersectInfo::subtract`] removes the spans
//! the subtract operand covers, so the first remaining face is the visible one.
//!
//! A difference has no tree of its own: it reports no children, identity, nothing contained,
//! no material and a [`RayTracerError::NotSupported`] color. A malformed hit list makes
//! [`Traceable::get_closest_intersection`] report no hit, because the trace path has no error
//! channel and a ray that cannot be resolved should show the background, not stop the render.

use std::any::Any;

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

use super::intersect_info_subtract::sort_on_distance;
use super::{IntersectInfo, MaterialRef, Primitive, RayTracerError, Traceable, TraceableRef};

/// `primary` minus `subtract`.
pub struct Difference {
    primary: TraceableRef,
    subtract: TraceableRef,
}

impl Difference {
    pub fn new(primary: TraceableRef, subtract: TraceableRef) -> Self {
        Difference { primary, subtract }
    }

    /// The shape cut from.
    pub fn primary(&self) -> &TraceableRef {
        &self.primary
    }

    /// The shape cut away.
    pub fn subtract(&self) -> &TraceableRef {
        &self.subtract
    }

    /// Every remaining face along `ray`, nearest first; `None` when the primary is not hit at
    /// all and `stop_when_primary_missed` is set (the subtract is not traced then).
    fn remaining_hits<'a>(
        &'a self,
        ray: &Ray,
        stop_when_primary_missed: bool,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        let mut check_front_and_backs = ray.clone();
        check_front_and_backs.intersection_type = IntersectionType::Both;
        let mut all_primary = self.primary.intersection_iterator(&check_front_and_backs)?;

        if all_primary.is_empty() && stop_when_primary_missed {
            // We did not hit the primary object.  We are done. The subtract object does not
            // mater.
            return Ok(all_primary);
        }

        sort_on_distance(&mut all_primary)?;

        // we hit the primary object, did we hit the subtract object before (within error)
        // hitting the primary.
        let mut all_subtract = self
            .subtract
            .intersection_iterator(&check_front_and_backs)?;

        if all_subtract.is_empty() {
            // we did not hit the subtract so return the primary
            return Ok(all_primary);
        }

        sort_on_distance(&mut all_subtract)?;

        let mut result = Vec::new();
        IntersectInfo::subtract(&all_primary, &all_subtract, &mut result)?;
        Ok(result)
    }
}

impl Traceable for Difference {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        self.primary.get_surface_area()
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        self.primary.get_axis_aligned_bounding_box()
    }

    fn get_center(&self) -> Vector3 {
        self.get_axis_aligned_bounding_box().get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.get_center()[axis]
    }

    fn get_contained<'a>(
        &'a self,
        _results: &mut Vec<&'a dyn Traceable>,
        _sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        false
    }

    fn contains(&self, position: Vector3) -> bool {
        self.get_axis_aligned_bounding_box()
            .contains_point(position)
            && (self.primary.contains(position) || self.subtract.contains(position))
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        self.remaining_hits(ray, true)
            .ok()
            .and_then(|hits| hits.first().copied())
    }

    fn get_intersect_cost(&self) -> f64 {
        self.primary.get_intersect_cost() + self.subtract.get_intersect_cost() / 2.0
    }

    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        self.remaining_hits(ray, false)
    }

    fn get_crossing(&self, _plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Err(RayTracerError::NotSupported("Difference.GetCrossing"))
    }

    fn get_touching(
        &self,
        _position: Vector3,
        _error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Err(RayTracerError::NotSupported("Difference.GetTouching"))
    }

    fn as_primitive(&self) -> Option<&dyn Primitive> {
        Some(self)
    }
}

impl Primitive for Difference {
    fn material(&self) -> Option<&MaterialRef> {
        None
    }

    fn get_uv(&self, _info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError> {
        Err(RayTracerError::NotSupported("Difference.GetUv"))
    }

    fn get_color(
        &self,
        _info: &IntersectInfo<'_>,
    ) -> Result<solidmesh_math::ColorF, RayTracerError> {
        // You should not get a color directly from a Difference.
        Err(RayTracerError::NotSupported("Difference.GetColor"))
    }
}
