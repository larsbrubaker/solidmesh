//! `Ray`: an origin, a direction and the distance window a hit must fall in, with the
//! precomputed reciprocal direction and per-axis signs the slab test uses. Ports agg-sharp
//! `VectorMath/Ray.cs` (and its `IntersectionType`). `Plane`/`PlaneFloat` intersect it; the
//! picking BVH (Phase 2) traces it.
//!
//! Mapping notes: the C# class becomes a `Clone` value with the same public fields (snake
//! case). The constructor's optional arguments are `new` (C# defaults: 0, +infinity,
//! `FrontFace`) and `with_limits`; the copy constructor is `Clone`. `Intersect(bounds, out ..)`
//! returns `(bool, RayBoundsHit)`. The static mutable `sameSurfaceOffset` is the constant
//! [`Ray::SAME_SURFACE_OFFSET`] (nothing in MatterCAD assigns it). `PerturbDirection(Random)`
//! takes the random source as a closure returning C# `NextDouble()` values.
//! `GetClosestIntersection` returns a [`RayHitInfo`] carrying the box that was hit; the C# never
//! returns null there (a miss has hit type `None`).

use crate::bvh::RayHitInfo;
use crate::{AxisAlignedBoundingBox, Matrix4X4, Vector3};

/// Which faces a ray reports hits on (C# `[Flags] enum IntersectionType`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum IntersectionType {
    None = 0,
    #[default]
    FrontFace = 1,
    BackFace = 2,
    Both = 3,
}

/// The sign of a direction component; the value indexes the box corner the slab test starts
/// from (C# `Ray.Sign`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum Sign {
    Negative = 1,
    #[default]
    Positive = 0,
}

/// The out values of [`Ray::intersect`]: the entry and exit distances and the axes of the
/// entry and exit faces.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RayBoundsHit {
    pub min_dist_found: f64,
    pub max_dist_found: f64,
    pub min_axis: usize,
    pub max_axis: usize,
}

/// A ray (C# `MatterHackers.VectorMath.Ray`).
#[derive(Debug, Clone, PartialEq)]
pub struct Ray {
    pub origin: Vector3,
    pub direction_normal: Vector3,
    pub min_distance_to_consider: f64,
    pub max_distance_to_consider: f64,
    pub one_over_direction: Vector3,
    pub is_shadow_ray: bool,
    pub intersection_type: IntersectionType,
    pub sign: [Sign; 3],
}

impl Ray {
    /// How far to step off a surface before tracing again (C# static `sameSurfaceOffset`).
    pub const SAME_SURFACE_OFFSET: f64 = 0.00001;

    /// A front-face ray considering every distance from 0 to infinity.
    pub fn new(origin: Vector3, direction_normal: Vector3) -> Self {
        Self::with_limits(
            origin,
            direction_normal,
            0.0,
            f64::INFINITY,
            IntersectionType::FrontFace,
        )
    }

    /// A ray with its distance window and face type.
    pub fn with_limits(
        origin: Vector3,
        direction_normal: Vector3,
        min_distance_to_consider: f64,
        max_distance_to_consider: f64,
        intersection_type: IntersectionType,
    ) -> Self {
        let one_over_direction = 1.0 / direction_normal;
        let sign_of = |v: f64| {
            if v < 0.0 {
                Sign::Negative
            } else {
                Sign::Positive
            }
        };
        Ray {
            origin,
            direction_normal,
            min_distance_to_consider,
            max_distance_to_consider,
            one_over_direction,
            is_shadow_ray: false,
            intersection_type,
            sign: [
                sign_of(one_over_direction.x),
                sign_of(one_over_direction.y),
                sign_of(one_over_direction.z),
            ],
        }
    }

    /// The ray moved by `matrix` (origin as a position, direction as a vector; the direction is
    /// not renormalized). `is_shadow_ray` resets to false, as in the C#.
    pub fn transform(ray: &Ray, matrix: Matrix4X4) -> Ray {
        let transformed_origin = ray.origin.transform_position(matrix);
        let transformed_directon = ray.direction_normal.transform_vector(matrix);
        Ray::with_limits(
            transformed_origin,
            transformed_directon,
            ray.min_distance_to_consider,
            ray.max_distance_to_consider,
            ray.intersection_type,
        )
    }

    /// The box corner (0 = min, 1 = max) the slab test enters through on `axis`, and the one it
    /// leaves through.
    fn slab_corners(&self, axis: usize) -> (usize, usize) {
        let entry = self.sign[axis] as usize;
        (entry, 1 - entry)
    }

    /// The slab test against `bounds`: true when the entry or exit distance is inside the
    /// window. The distances and face axes are filled in as far as the test got, as the C#
    /// `out` parameters are.
    pub fn intersect(&self, bounds: &AxisAlignedBoundingBox) -> (bool, RayBoundsHit) {
        let mut hit = RayBoundsHit::default();

        // we calculate distance to the intersection with the x planes of the box
        let (entry, exit) = self.slab_corners(0);
        hit.min_dist_found = (bounds.corner(entry).x - self.origin.x) * self.one_over_direction.x;
        hit.max_dist_found = (bounds.corner(exit).x - self.origin.x) * self.one_over_direction.x;

        // now find the distance to the y planes of the box
        let (entry, exit) = self.slab_corners(1);
        let min_dist_to_y = (bounds.corner(entry).y - self.origin.y) * self.one_over_direction.y;
        let max_dist_to_y = (bounds.corner(exit).y - self.origin.y) * self.one_over_direction.y;

        if (hit.min_dist_found > max_dist_to_y) || (min_dist_to_y > hit.max_dist_found) {
            return (false, hit);
        }

        if min_dist_to_y > hit.min_dist_found {
            hit.min_axis = 1;
            hit.min_dist_found = min_dist_to_y;
        }

        if max_dist_to_y < hit.max_dist_found {
            hit.max_axis = 1;
            hit.max_dist_found = max_dist_to_y;
        }

        // and finally the z planes
        let (entry, exit) = self.slab_corners(2);
        let min_dist_to_z = (bounds.corner(entry).z - self.origin.z) * self.one_over_direction.z;
        let max_dist_to_z = (bounds.corner(exit).z - self.origin.z) * self.one_over_direction.z;

        if (hit.min_dist_found > max_dist_to_z) || (min_dist_to_z > hit.max_dist_found) {
            return (false, hit);
        }

        if min_dist_to_z > hit.min_dist_found {
            hit.min_axis = 2;
            hit.min_dist_found = min_dist_to_z;
        }

        if max_dist_to_z < hit.max_dist_found {
            hit.max_axis = 2;
            hit.max_dist_found = max_dist_to_z;
        }

        let one_hit_is_within_limits = (hit.min_dist_found < self.max_distance_to_consider
            && hit.min_dist_found > self.min_distance_to_consider)
            || (hit.max_dist_found < self.max_distance_to_consider
                && hit.max_dist_found > self.min_distance_to_consider);
        (one_hit_is_within_limits, hit)
    }

    /// The hit on `bounds`: the entry face for a front-face ray, the exit face otherwise, when
    /// that distance is inside the window. A shadow ray stops once the hit type is known, with
    /// no object, position or distance filled in.
    pub fn get_closest_intersection(
        &self,
        bounds: &AxisAlignedBoundingBox,
    ) -> RayHitInfo<AxisAlignedBoundingBox> {
        let mut info = RayHitInfo::new();

        let (hit, found) = self.intersect(bounds);
        if hit {
            let (hit_type, dist_found) = if self.intersection_type == IntersectionType::FrontFace {
                (IntersectionType::FrontFace, found.min_dist_found)
            } else {
                // check back faces
                (IntersectionType::BackFace, found.max_dist_found)
            };

            if dist_found > self.min_distance_to_consider
                && dist_found < self.max_distance_to_consider
            {
                info.hit_type = hit_type;
                if self.is_shadow_ray {
                    return info;
                }

                info.closest_hit_object = Some(*bounds);
                info.hit_position = self.origin + self.direction_normal * dist_found;
                let mut normal_at_hit = Vector3::default();
                // you hit the side that is opposite your sign (the C# uses the entry face's
                // axis for back faces too)
                normal_at_hit[found.min_axis] = if self.sign[found.min_axis] == Sign::Negative {
                    1.0
                } else {
                    -1.0
                };
                info.normal_at_hit = normal_at_hit;
                info.distance_to_hit = dist_found;
            }
        }

        info
    }

    /// The slab test against `bounds`: true when the part of the ray inside the box overlaps the
    /// distance window (C# `Intersection`).
    pub fn intersection(&self, bounds: &AxisAlignedBoundingBox) -> bool {
        match self.slab_window(bounds) {
            Some((min_dist_found, max_dist_found)) => {
                (min_dist_found < self.max_distance_to_consider)
                    && (max_dist_found > self.min_distance_to_consider)
            }
            None => false,
        }
    }

    /// The entry/exit distances of `Intersection`'s slab test, `None` when a slab misses.
    fn slab_window(&self, bounds: &AxisAlignedBoundingBox) -> Option<(f64, f64)> {
        // we calculate distance to the intersection with the x planes of the box
        let (entry, exit) = self.slab_corners(0);
        let mut min_dist_found =
            (bounds.corner(entry).x - self.origin.x) * self.one_over_direction.x;
        let mut max_dist_found =
            (bounds.corner(exit).x - self.origin.x) * self.one_over_direction.x;

        // now find the distance to the y planes of the box
        let (entry, exit) = self.slab_corners(1);
        let min_dist_to_y = (bounds.corner(entry).y - self.origin.y) * self.one_over_direction.y;
        let max_dist_to_y = (bounds.corner(exit).y - self.origin.y) * self.one_over_direction.y;

        if (min_dist_found > max_dist_to_y) || (min_dist_to_y > max_dist_found) {
            return None;
        }

        if min_dist_to_y > min_dist_found {
            min_dist_found = min_dist_to_y;
        }

        if max_dist_to_y < max_dist_found {
            max_dist_found = max_dist_to_y;
        }

        // and finally the z planes
        let (entry, exit) = self.slab_corners(2);
        let min_dist_to_z = (bounds.corner(entry).z - self.origin.z) * self.one_over_direction.z;
        let max_dist_to_z = (bounds.corner(exit).z - self.origin.z) * self.one_over_direction.z;

        if (min_dist_found > max_dist_to_z) || (min_dist_to_z > max_dist_found) {
            return None;
        }

        if min_dist_to_z > min_dist_found {
            min_dist_found = min_dist_to_z;
        }

        if max_dist_to_z < max_dist_found {
            max_dist_found = max_dist_to_z;
        }

        Some((min_dist_found, max_dist_found))
    }

    /// Nudges the direction by up to 1e-5 per axis; `next_double` stands in for C#
    /// `Random.NextDouble()` (a value in [0, 1)). The reciprocal direction and signs are not
    /// updated, as in the C#.
    pub fn perturb_direction(&mut self, mut next_double: impl FnMut() -> f64) {
        self.direction_normal.x += 1e-5 * next_double();
        self.direction_normal.y += 1e-5 * next_double();
        self.direction_normal.z += 1e-5 * next_double();
    }
}
