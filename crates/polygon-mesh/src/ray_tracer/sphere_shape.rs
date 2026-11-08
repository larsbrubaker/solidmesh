//! A solid sphere: the classic ray-sphere test, front and back faces for
//! [`super::Difference`], and the latitude/longitude texture coordinates.

use std::any::Any;
use std::f64::consts::PI;

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

use super::base_shape::{leaf_contains, leaf_get_contained, leaf_get_crossing, leaf_get_touching};
use super::{IntersectInfo, MaterialRef, Primitive, RayTracerError, Traceable};

/// A sphere of `radius` around `position`.
#[derive(Debug, Clone)]
pub struct SphereShape {
    pub radius: f64,
    pub position: Vector3,
    material: Option<MaterialRef>,
}

/// The pieces of the ray-sphere test both trace paths share.
struct SphereRayTerms {
    distance_from_sphere_center_to_ray_origin: f64,
    length_from_ray_orgin_to_sphere_center_squared: f64,
    amount_squared: f64,
}

impl SphereShape {
    /// A sphere of `radius` around `position` with `material`.
    pub fn new(position: Vector3, radius: f64, material: Option<MaterialRef>) -> Self {
        SphereShape {
            radius,
            position,
            material,
        }
    }

    fn ray_terms(&self, ray: &Ray, radius_squared: f64) -> SphereRayTerms {
        let delta_from_shpere_center_to_ray_origin = ray.origin - self.position;
        // negative means the sphere is in front of the ray.
        let distance_from_sphere_center_to_ray_origin =
            delta_from_shpere_center_to_ray_origin.dot(ray.direction_normal);
        let length_from_ray_orgin_to_sphere_center_squared =
            delta_from_shpere_center_to_ray_origin.dot(delta_from_shpere_center_to_ray_origin);
        let length_from_ray_origin_to_near_edge_of_sphere_squared =
            length_from_ray_orgin_to_sphere_center_squared - radius_squared;
        let distance_from_sphere_center_to_ray_squared =
            distance_from_sphere_center_to_ray_origin * distance_from_sphere_center_to_ray_origin;
        SphereRayTerms {
            distance_from_sphere_center_to_ray_origin,
            length_from_ray_orgin_to_sphere_center_squared,
            amount_squared: distance_from_sphere_center_to_ray_squared
                - length_from_ray_origin_to_near_edge_of_sphere_squared,
        }
    }

    fn hit_at(
        &self,
        ray: &Ray,
        hit_type: IntersectionType,
        distance_to_hit: f64,
    ) -> IntersectInfo<'_> {
        let mut info = IntersectInfo::new();
        info.hit_type = hit_type;
        info.closest_hit_object = Some(self);
        info.distance_to_hit = distance_to_hit;
        info.hit_position = ray.origin + ray.direction_normal * info.distance_to_hit;
        let outward = (info.hit_position - self.position).get_normal();
        info.normal_at_hit = if hit_type == IntersectionType::BackFace {
            -outward
        } else {
            outward
        };
        info
    }
}

impl SphereShape {
    /// Replaces the material.
    pub fn set_material(&mut self, material: Option<MaterialRef>) {
        self.material = material;
    }
}

impl std::fmt::Display for SphereShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Sphere ({},{},{}) Radius: {}",
            self.position.x, self.position.y, self.position.z, self.radius
        )
    }
}

impl Traceable for SphereShape {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        // A leaf is never asked: the tree walk reads this only on items with children.
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        // 4πr², written with τ = 2π.
        2.0 * std::f64::consts::TAU * self.radius * self.radius
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        let (p, r) = (self.position, self.radius);
        AxisAlignedBoundingBox::new(
            Vector3::new(p.x - r, p.y - r, p.z - r),
            Vector3::new(p.x + r, p.y + r, p.z + r),
        )
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

    /// This implementation of intersect uses the fastest ray-sphere intersection algorithm I
    /// could find on the internet.
    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let radius_squared = self.radius * self.radius;
        let terms = self.ray_terms(ray, radius_squared);

        if terms.amount_squared > 0.0
            || (ray.intersection_type == IntersectionType::BackFace
                && terms.length_from_ray_orgin_to_sphere_center_squared < radius_squared)
        {
            // yes, that's it, we found the intersection!
            if ray.is_shadow_ray {
                let mut info = IntersectInfo::new();
                info.closest_hit_object = Some(self);
                info.hit_type = IntersectionType::FrontFace;
                return Some(info);
            }
            let distance_from_ray_origin_to_sphere_center =
                -terms.distance_from_sphere_center_to_ray_origin;

            let amount = terms.amount_squared.sqrt();
            if ray.intersection_type == IntersectionType::FrontFace {
                let distance_to_front_hit = distance_from_ray_origin_to_sphere_center - amount;
                if distance_to_front_hit > ray.max_distance_to_consider
                    || distance_to_front_hit < ray.min_distance_to_consider
                {
                    return None;
                }
                return Some(self.hit_at(ray, IntersectionType::FrontFace, distance_to_front_hit));
            }

            // check back faces
            let distance_to_back_hit = distance_from_ray_origin_to_sphere_center + amount;
            if distance_to_back_hit > ray.max_distance_to_consider
                || distance_to_back_hit < ray.min_distance_to_consider
            {
                return None;
            }
            return Some(self.hit_at(ray, IntersectionType::BackFace, distance_to_back_hit));
        }

        None
    }

    fn get_intersect_cost(&self) -> f64 {
        670.0
    }

    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        let mut hits = Vec::new();
        let radius_squared = self.radius * self.radius;
        let terms = self.ray_terms(ray, radius_squared);

        if terms.amount_squared > 0.0 {
            let distance_from_ray_origin_to_sphere_center =
                -terms.distance_from_sphere_center_to_ray_origin;
            let amount = terms.amount_squared.sqrt();
            let wanted = ray.intersection_type as u8;

            if wanted & IntersectionType::FrontFace as u8 != 0 {
                hits.push(self.hit_at(
                    ray,
                    IntersectionType::FrontFace,
                    distance_from_ray_origin_to_sphere_center - amount,
                ));
            }

            if wanted & IntersectionType::BackFace as u8 != 0 {
                hits.push(self.hit_at(
                    ray,
                    IntersectionType::BackFace,
                    distance_from_ray_origin_to_sphere_center + amount,
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

impl Primitive for SphereShape {
    fn material(&self) -> Option<&MaterialRef> {
        self.material.as_ref()
    }

    fn get_uv(&self, info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError> {
        let vn = Vector3::new(0.0, 1.0, 0.0).get_normal(); // north pole / up
        let ve = Vector3::new(0.0, 0.0, 1.0).get_normal(); // equator / sphere orientation
                                                           // points from center of sphere to intersection
        let vp = (info.hit_position - self.position).get_normal();

        let phi = libm::acos(-vp.dot(vn));
        let v = (phi * 2.0 / PI) - 1.0;

        let mut sinphi = ve.dot(vp) / libm::sin(phi);
        sinphi = sinphi.clamp(-1.0, 1.0);
        let theta = libm::acos(sinphi) * 2.0 / PI;

        let u = if vn.cross(ve).dot(vp) > 0.0 {
            theta
        } else {
            1.0 - theta
        };

        Ok((u, v))
    }
}
