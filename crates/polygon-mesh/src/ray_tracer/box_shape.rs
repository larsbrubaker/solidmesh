//! An axis-aligned solid box, with the shared leaf-shape queries of `base_shape`. Being
//! closed, it reports a front face where a ray enters and a back face where it leaves, which
//! is what [`super::Difference`] needs. The slab test is vector-math's [`Ray::intersect`].

use std::any::Any;

use vector_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Sign, Vector3};

use super::base_shape::{leaf_contains, leaf_get_contained, leaf_get_crossing, leaf_get_touching};
use super::{IntersectInfo, MaterialRef, Primitive, RayTracerError, Traceable};

/// A box from `min_xyz` to `max_xyz`.
#[derive(Debug, Clone)]
pub struct BoxShape {
    pub min_xyz: Vector3,
    pub max_xyz: Vector3,
    material: Option<MaterialRef>,
}

impl BoxShape {
    /// A box from `min_xyz` to `max_xyz` with `material`; an error when any max is below its
    /// min.
    pub fn new(
        min_xyz: Vector3,
        max_xyz: Vector3,
        material: Option<MaterialRef>,
    ) -> Result<Self, RayTracerError> {
        if max_xyz.x < min_xyz.x || max_xyz.y < min_xyz.y || max_xyz.z < min_xyz.z {
            return Err(RayTracerError::InvalidArgument(
                "All values of min must be less than all values in max.",
            ));
        }

        Ok(BoxShape {
            min_xyz,
            max_xyz,
            material,
        })
    }

    /// The face hit at `distance` on `axis`: you hit the side that is opposite your sign.
    fn face_hit(
        &self,
        ray: &Ray,
        hit_type: IntersectionType,
        distance: f64,
        axis: usize,
    ) -> IntersectInfo<'_> {
        let mut info = IntersectInfo::new();
        info.hit_type = hit_type;
        info.closest_hit_object = Some(self);
        info.hit_position = ray.origin + ray.direction_normal * distance;
        info.normal_at_hit[axis] = if ray.sign[axis] == Sign::Negative {
            1.0
        } else {
            -1.0
        };
        info.distance_to_hit = distance;
        info
    }
}

impl BoxShape {
    /// Replaces the material.
    pub fn set_material(&mut self, material: Option<MaterialRef>) {
        self.material = material;
    }
}

impl std::fmt::Display for BoxShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Box ({},{},{})-({},{},{})",
            self.min_xyz.x,
            self.min_xyz.y,
            self.min_xyz.z,
            self.max_xyz.x,
            self.max_xyz.y,
            self.max_xyz.z
        )
    }
}

impl Traceable for BoxShape {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        // A leaf has no children, and only items with children are asked; identity is the
        // honest answer for a shape in world space.
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        let front_and_back =
            (self.max_xyz.x - self.min_xyz.x) * (self.max_xyz.z - self.min_xyz.z) * 2.0;
        let left_and_right =
            (self.max_xyz.y - self.min_xyz.y) * (self.max_xyz.z - self.min_xyz.z) * 2.0;
        let top_and_bottom =
            (self.max_xyz.x - self.min_xyz.x) * (self.max_xyz.y - self.min_xyz.y) * 2.0;
        front_and_back + left_and_right + top_and_bottom
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::new(self.min_xyz, self.max_xyz)
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
        leaf_get_contained(self, results, sub_region)
    }

    fn contains(&self, position: Vector3) -> bool {
        leaf_contains(self, position)
    }

    /// The entry face for a front-face ray, the exit face for any other (so `Both` reports the
    /// exit).
    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let (hit, bounds_hit) = ray.intersect(&self.get_axis_aligned_bounding_box());
        if !hit {
            return None;
        }

        let (hit_type, distance, axis) = if ray.intersection_type == IntersectionType::FrontFace {
            (
                IntersectionType::FrontFace,
                bounds_hit.min_dist_found,
                bounds_hit.min_axis,
            )
        } else {
            (
                IntersectionType::BackFace,
                bounds_hit.max_dist_found,
                bounds_hit.max_axis,
            )
        };

        if distance > ray.min_distance_to_consider && distance < ray.max_distance_to_consider {
            if ray.is_shadow_ray {
                let mut info = IntersectInfo::new();
                info.hit_type = hit_type;
                return Some(info);
            }
            return Some(self.face_hit(ray, hit_type, distance, axis));
        }

        None
    }

    fn get_intersect_cost(&self) -> f64 {
        452.0
    }

    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        let mut hits = Vec::new();
        let (hit, bounds_hit) = ray.intersect(&self.get_axis_aligned_bounding_box());
        if hit {
            let wanted = ray.intersection_type as u8;
            if wanted & IntersectionType::FrontFace as u8 != 0 {
                hits.push(self.face_hit(
                    ray,
                    IntersectionType::FrontFace,
                    bounds_hit.min_dist_found,
                    bounds_hit.min_axis,
                ));
            }

            if wanted & IntersectionType::BackFace as u8 != 0 {
                hits.push(self.face_hit(
                    ray,
                    IntersectionType::BackFace,
                    bounds_hit.max_dist_found,
                    bounds_hit.max_axis,
                ));
            }
        }

        Ok(hits)
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

    fn as_primitive(&self) -> Option<&dyn Primitive> {
        Some(self)
    }
}

impl Primitive for BoxShape {
    fn material(&self) -> Option<&MaterialRef> {
        self.material.as_ref()
    }

    fn get_uv(&self, _info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError> {
        Err(RayTracerError::NotSupported("BoxShape.GetUv"))
    }
}
