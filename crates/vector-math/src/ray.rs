//! `Ray`: an origin, a direction and the distance window a hit must fall in, with the
//! precomputed reciprocal direction and per-axis signs the slab test uses, and
//! [`IntersectionType`]. `Plane`/`PlaneFloat` intersect it; the picking BVH traces it.
//!
//! The slab test counts touching as a hit: a ray that runs along a box face, or lies in it,
//! hits the box, on every axis alike (`Ray::touching_x_window` says how).
//!
//! A ray is a `Clone` value with public fields. `new` uses the usual window (0 to +infinity,
//! front faces) and `with_limits` sets it. `intersect` returns `(bool, RayBoundsHit)`.
//! [`Ray::SAME_SURFACE_OFFSET`] is a constant, since no caller needs another value.
//! `perturb_direction` takes the random source as a closure returning values in [0, 1).
//! `get_closest_intersection` always returns a [`RayHitInfo`] carrying the box that was hit; a
//! miss has hit type `None`.

use crate::bvh::RayHitInfo;
use crate::{AxisAlignedBoundingBox, Matrix4X4, Vector3};

/// Which faces a ray reports hits on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum IntersectionType {
    None = 0,
    #[default]
    FrontFace = 1,
    BackFace = 2,
    Both = 3,
}

/// The sign of a direction component; the value indexes the box corner the slab test starts
/// from.
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

/// A ray.
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
    /// How far to step off a surface before tracing again.
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
        let (one_over_direction, sign) = Self::slab_terms(direction_normal);
        Ray {
            origin,
            direction_normal,
            min_distance_to_consider,
            max_distance_to_consider,
            one_over_direction,
            is_shadow_ray: false,
            intersection_type,
            sign,
        }
    }

    /// The reciprocal direction and per-axis signs the slab test reads; they must be
    /// recomputed whenever the direction changes.
    fn slab_terms(direction_normal: Vector3) -> (Vector3, [Sign; 3]) {
        let one_over_direction = 1.0 / direction_normal;
        let sign_of = |v: f64| {
            if v < 0.0 {
                Sign::Negative
            } else {
                Sign::Positive
            }
        };
        (
            one_over_direction,
            [
                sign_of(one_over_direction.x),
                sign_of(one_over_direction.y),
                sign_of(one_over_direction.z),
            ],
        )
    }

    /// The ray moved by `matrix` (origin as a position, direction as a vector; the direction is
    /// not renormalized). The window, face type and shadow flag carry over: moving a ray into
    /// an object's space must not change what kind of query it is.
    pub fn transform(ray: &Ray, matrix: Matrix4X4) -> Ray {
        let transformed_origin = ray.origin.transform_position(matrix);
        let transformed_directon = ray.direction_normal.transform_vector(matrix);
        let mut transformed = Ray::with_limits(
            transformed_origin,
            transformed_directon,
            ray.min_distance_to_consider,
            ray.max_distance_to_consider,
            ray.intersection_type,
        );
        transformed.is_shadow_ray = ray.is_shadow_ray;
        transformed
    }

    /// The box corner (0 = min, 1 = max) the slab test enters through on `axis`, and the one it
    /// leaves through.
    fn slab_corners(&self, axis: usize) -> (usize, usize) {
        let entry = self.sign[axis] as usize;
        (entry, 1 - entry)
    }

    /// The x slab's entry and exit distances with touching counted as a hit: the window the
    /// slab tests start from when a plain x distance came out NaN.
    ///
    /// A ray that does not move on an axis (a zero direction component, so an infinite
    /// reciprocal) and starts exactly in one of that axis's box planes stays in the plane for its
    /// whole length, touching the box, so that plane bounds nothing. Its distance,
    /// `0 * infinity`, is undefined (NaN). On y and z that already gives the right answer: every
    /// comparison with a NaN is false, so the window simply does not take that plane's value. On
    /// x the distance is the window's starting value and the NaN survives to the end: the box
    /// reads as missed, or, when only the exit distance is NaN, as hit with an undefined exit
    /// distance. So each such x distance becomes the value the y and z planes behave as:
    /// -infinity entering, +infinity leaving.
    ///
    /// The repair applies only when every NaN x distance is such a `0 * infinity` (offset exactly
    /// 0, infinite reciprocal). When any comes from anything else (a NaN origin x, a NaN bound x,
    /// a NaN direction x, or `infinity - infinity`), both raw distances are kept, so the answer is
    /// what the plain products always gave. The y and z components are not looked at here: a ray
    /// in an x face plane whose y or z origin or direction is NaN has that axis ignored, as a ray
    /// off the face always had.
    ///
    /// The slab tests compute the plain x products first and come here only when one is NaN,
    /// through a cold out-of-line copy of themselves, so the common path stays the plain
    /// products. The copy is entered by returning its answer, which lets
    /// [`Ray::intersection`], the test a BVH makes at every node, stay a leaf with no stack
    /// frame.
    fn touching_x_window(&self, bounds: &AxisAlignedBoundingBox) -> (f64, f64) {
        let (entry, exit) = self.slab_corners(0);
        let inverse = self.one_over_direction.x;
        let entry_offset = bounds.corner(entry).x - self.origin.x;
        let exit_offset = bounds.corner(exit).x - self.origin.x;
        let min_dist = entry_offset * inverse;
        let max_dist = exit_offset * inverse;
        // True for a NaN distance that is not `0 * infinity`.
        let undefined_otherwise =
            |dist: f64, offset: f64| dist.is_nan() && !(offset == 0.0 && inverse.is_infinite());
        if undefined_otherwise(min_dist, entry_offset) || undefined_otherwise(max_dist, exit_offset)
        {
            return (min_dist, max_dist);
        }
        (
            if min_dist.is_nan() {
                f64::NEG_INFINITY
            } else {
                min_dist
            },
            if max_dist.is_nan() {
                f64::INFINITY
            } else {
                max_dist
            },
        )
    }

    /// The slab test against `bounds`: true when the entry or exit distance is inside the
    /// window. The distances and face axes are filled in as far as the test got, even on a
    /// miss.
    pub fn intersect(&self, bounds: &AxisAlignedBoundingBox) -> (bool, RayBoundsHit) {
        let mut hit = RayBoundsHit::default();

        // we calculate distance to the intersection with the x planes of the box
        let (entry, exit) = self.slab_corners(0);
        hit.min_dist_found = (bounds.corner(entry).x - self.origin.x) * self.one_over_direction.x;
        hit.max_dist_found = (bounds.corner(exit).x - self.origin.x) * self.one_over_direction.x;
        if hit.min_dist_found.is_nan() | hit.max_dist_found.is_nan() {
            return self.intersect_touching(bounds);
        }
        self.intersect_from_x(bounds, hit)
    }

    /// [`Ray::intersect`] after its x slab: the y and z slabs and the window test.
    #[inline(always)]
    fn intersect_from_x(
        &self,
        bounds: &AxisAlignedBoundingBox,
        mut hit: RayBoundsHit,
    ) -> (bool, RayBoundsHit) {
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

    /// [`Ray::intersect`] for a NaN x distance, through [`Ray::touching_x_window`].
    #[cold]
    #[inline(never)]
    fn intersect_touching(&self, bounds: &AxisAlignedBoundingBox) -> (bool, RayBoundsHit) {
        let (min_dist_found, max_dist_found) = self.touching_x_window(bounds);
        let hit = RayBoundsHit {
            min_dist_found,
            max_dist_found,
            ..RayBoundsHit::default()
        };
        self.intersect_from_x(bounds, hit)
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
                // The normal is the hit face's outward normal: the ray enters through the face
                // opposite its sign on the entry axis and leaves through the face matching its
                // sign on the exit axis.
                let mut normal_at_hit = Vector3::default();
                let entering_outward = |axis: usize| {
                    if self.sign[axis] == Sign::Negative {
                        1.0
                    } else {
                        -1.0
                    }
                };
                if hit_type == IntersectionType::FrontFace {
                    normal_at_hit[found.min_axis] = entering_outward(found.min_axis);
                } else {
                    normal_at_hit[found.max_axis] = -entering_outward(found.max_axis);
                }
                info.normal_at_hit = normal_at_hit;
                info.distance_to_hit = dist_found;
            }
        }

        info
    }

    /// The slab test against `bounds`: true when the part of the ray inside the box overlaps the
    /// distance window.
    pub fn intersection(&self, bounds: &AxisAlignedBoundingBox) -> bool {
        // we calculate distance to the intersection with the x planes of the box
        let (entry, exit) = self.slab_corners(0);
        let min_dist_found = (bounds.corner(entry).x - self.origin.x) * self.one_over_direction.x;
        let max_dist_found = (bounds.corner(exit).x - self.origin.x) * self.one_over_direction.x;
        if min_dist_found.is_nan() | max_dist_found.is_nan() {
            return self.intersection_touching(bounds);
        }
        self.window_overlaps(self.slab_window_from_x(bounds, min_dist_found, max_dist_found))
    }

    /// [`Ray::intersection`] for a NaN x distance, through [`Ray::touching_x_window`].
    #[cold]
    #[inline(never)]
    fn intersection_touching(&self, bounds: &AxisAlignedBoundingBox) -> bool {
        let (min_dist_found, max_dist_found) = self.touching_x_window(bounds);
        self.window_overlaps(self.slab_window_from_x(bounds, min_dist_found, max_dist_found))
    }

    /// True when the slab window overlaps the ray's distance window.
    #[inline(always)]
    fn window_overlaps(&self, window: Option<(f64, f64)>) -> bool {
        match window {
            Some((min_dist_found, max_dist_found)) => {
                (min_dist_found < self.max_distance_to_consider)
                    && (max_dist_found > self.min_distance_to_consider)
            }
            None => false,
        }
    }

    /// The entry/exit distances of [`Ray::intersection`]'s slab test from its x slab's, `None`
    /// when a slab misses.
    #[inline(always)]
    fn slab_window_from_x(
        &self,
        bounds: &AxisAlignedBoundingBox,
        mut min_dist_found: f64,
        mut max_dist_found: f64,
    ) -> Option<(f64, f64)> {
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

    /// Nudges the direction by up to 1e-5 per axis; `next_double` returns a random value in
    /// [0, 1). The slab test's reciprocal direction and signs follow the new direction.
    pub fn perturb_direction(&mut self, mut next_double: impl FnMut() -> f64) {
        self.direction_normal.x += 1e-5 * next_double();
        self.direction_normal.y += 1e-5 * next_double();
        self.direction_normal.z += 1e-5 * next_double();
        let (one_over_direction, sign) = Self::slab_terms(self.direction_normal);
        self.one_over_direction = one_over_direction;
        self.sign = sign;
    }
}
