//! [`RenderCamera`]: the rays through the image, perspective or orthographic, built from eye,
//! target and up. Screen coordinates are `WorldView`'s: bottom-left origin with y up, looking down
//! -Z of the view space with +Y up and x to the right. The field of view is not: `WorldView`'s is
//! always vertical, this one is on the shorter image side, so in portrait a view's vertical field
//! gives different rays; pass it `WorldView::h_fov_degrees` there to match the view.
//!
//! The field of view (or the orthographic extent) is measured across the **shorter** image side,
//! so changing the window's aspect ratio only ever adds view on the longer side and the object in
//! the middle never gets clipped by it.

use solidmesh_math::{IntersectionType, Ray, Vector3};

use crate::ray_tracer::RayTracerError;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Projection {
    /// `tan` of half the field of view on the shorter side.
    Perspective { half_tangent: f64 },
    /// Half the view's extent across the shorter side, in world units.
    Orthographic { half_extent: f64 },
}

/// A camera: eye position and orientation, and how the image spreads out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderCamera {
    eye: Vector3,
    forward: Vector3,
    right: Vector3,
    up: Vector3,
    projection: Projection,
}

impl RenderCamera {
    /// A perspective camera at `eye` looking at `target`, with `up` as near to the image's up as
    /// it can be, and `fov_degrees` across the shorter image side, in (0, 180).
    pub fn perspective(
        eye: Vector3,
        target: Vector3,
        up: Vector3,
        fov_degrees: f64,
    ) -> Result<Self, RayTracerError> {
        if !(fov_degrees > 0.0 && fov_degrees < 180.0) {
            return Err(RayTracerError::InvalidArgument(
                "a perspective camera's field of view must be between 0 and 180 degrees",
            ));
        }
        let half_tangent = libm::tan(fov_degrees.to_radians() / 2.0);
        Self::oriented(eye, target, up, Projection::Perspective { half_tangent })
    }

    /// An orthographic camera at `eye` looking at `target`, showing `shorter_side_extent` world
    /// units across the shorter image side, above 0. Rays start on the plane through `eye`.
    pub fn orthographic(
        eye: Vector3,
        target: Vector3,
        up: Vector3,
        shorter_side_extent: f64,
    ) -> Result<Self, RayTracerError> {
        if !(shorter_side_extent > 0.0 && shorter_side_extent.is_finite()) {
            return Err(RayTracerError::InvalidArgument(
                "an orthographic camera's extent must be finite and above 0",
            ));
        }
        let projection = Projection::Orthographic {
            half_extent: shorter_side_extent / 2.0,
        };
        Self::oriented(eye, target, up, projection)
    }

    fn oriented(
        eye: Vector3,
        target: Vector3,
        up: Vector3,
        projection: Projection,
    ) -> Result<Self, RayTracerError> {
        let toward = target - eye;
        if !eye.is_valid() || !toward.is_valid() || toward.length_squared() <= 0.0 {
            return Err(RayTracerError::InvalidArgument(
                "a camera's eye and target must be finite and apart",
            ));
        }
        let forward = toward.get_normal();
        let sideways = forward.cross(up);
        if !sideways.is_valid() || sideways.length_squared() <= 1e-24 {
            return Err(RayTracerError::InvalidArgument(
                "a camera's up must be finite and not along its view direction",
            ));
        }
        let right = sideways.get_normal();
        Ok(RenderCamera {
            eye,
            forward,
            right,
            up: right.cross(forward),
            projection,
        })
    }

    /// The ray through screen position (`x`, `y`) of a `width` by `height` image, in pixel units
    /// with the origin at the bottom-left corner of the bottom-left pixel; its direction is unit
    /// length and it reports hits on both faces.
    pub fn ray_through(&self, x: f64, y: f64, width: u32, height: u32) -> Ray {
        let (width, height) = (f64::from(width), f64::from(height));
        let shorter = width.min(height);
        // The image's center is the view axis; a unit of `horizontal` is half the shorter side.
        let horizontal = (2.0 * x - width) / shorter;
        let vertical = (2.0 * y - height) / shorter;
        let (origin, direction) = match self.projection {
            Projection::Perspective { half_tangent } => {
                let direction = self.forward
                    + self.right * (horizontal * half_tangent)
                    + self.up * (vertical * half_tangent);
                (self.eye, direction.get_normal())
            }
            Projection::Orthographic { half_extent } => {
                let origin = self.eye
                    + self.right * (horizontal * half_extent)
                    + self.up * (vertical * half_extent);
                (origin, self.forward)
            }
        };
        Ray::with_limits(
            origin,
            direction,
            0.0,
            f64::INFINITY,
            IntersectionType::Both,
        )
    }
}
