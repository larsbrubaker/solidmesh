//! `Quaternion`: an f64 rotation quaternion (derived from OpenTK). `Matrix4X4::get_rotation`
//! builds one from a matrix and `Matrix4X4::create_rotation_quaternion` turns one back; its
//! JSON shape is in `serde_impls.rs`.
//!
//! `x`/`y`/`z`/`w` are fields and `xyz()`/`set_xyz()` read and write the vector part.
//! Constructors: `from_xyz_w`, `from_matrix`, `from_directions` and `from_directions_f32`.
//! `normalized` (no zero check), `conjugated` and `to_axis_angle_parts` return new values;
//! `multiply` is also the `*` operator, and `+`/`-` are component-wise.

use crate::{Matrix4X4, Vector3, Vector3Float, Vector4};
use std::fmt;
use std::ops::{Add, Mul, Sub};

/// A quaternion of f64.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quaternion {
    /// The identity rotation.
    pub const IDENTITY: Quaternion = Quaternion::new(0.0, 0.0, 0.0, 1.0);

    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Quaternion { x, y, z, w }
    }

    /// The vector part `v` and scalar part `w`.
    pub fn from_xyz_w(v: Vector3, w: f64) -> Self {
        Quaternion::new(v.x, v.y, v.z, w)
    }

    /// The rotation a matrix holds.
    pub fn from_matrix(m: Matrix4X4) -> Self {
        m.get_rotation()
    }

    /// The rotation turning `starting_direction` to `ending_direction`. Opposite directions
    /// are nudged by 1e-7 in x so the rotation is defined.
    pub fn from_directions(starting_direction: Vector3, ending_direction: Vector3) -> Self {
        let mut ending_direction = ending_direction;
        if (ending_direction + starting_direction).length_squared() == 0.0 {
            ending_direction += Vector3::new(0.0000001, 0.0, 0.0);
        }

        let mut result = Quaternion::from_xyz_w(
            ending_direction.cross(starting_direction),
            (ending_direction.length().powi(2) * starting_direction.length().powi(2)).sqrt()
                + ending_direction.dot(starting_direction),
        );
        result.normalize();
        result
    }

    /// [`Quaternion::from_directions`] for f32 directions (the cross product, lengths and dot
    /// are computed in f32, the precision of the inputs, and widened).
    pub fn from_directions_f32(
        starting_direction: Vector3Float,
        ending_direction: Vector3Float,
    ) -> Self {
        let mut ending_direction = ending_direction;
        if (ending_direction + starting_direction).length_squared() == 0.0 {
            ending_direction += Vector3Float::new(0.0000001, 0.0, 0.0);
        }

        let ending_length = f64::from(ending_direction.length());
        let starting_length = f64::from(starting_direction.length());
        let mut result = Quaternion::from_xyz_w(
            Vector3::from_vector3_float(ending_direction.cross(starting_direction)),
            (ending_length.powi(2) * starting_length.powi(2)).sqrt()
                + f64::from(ending_direction.dot(starting_direction)),
        );
        result.normalize();
        result
    }

    /// The vector part.
    pub fn xyz(&self) -> Vector3 {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Sets the vector part.
    pub fn set_xyz(&mut self, value: Vector3) {
        self.x = value.x;
        self.y = value.y;
        self.z = value.z;
    }

    /// The axis (xyz) and angle (w, radians) of the rotation.
    pub fn to_axis_angle(&self) -> Vector4 {
        let mut axis_angle = Vector4::default();
        let mut q1 = *self;
        if q1.w > 1.0 {
            // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is
            // normalized
            q1.normalize();
        }

        axis_angle.w = 2.0 * libm::acos(q1.w);
        // assuming quaternion normalized then w is less than 1, so term always positive.
        let s = (1.0 - q1.w * q1.w).sqrt();
        if s < 0.001 {
            // test to avoid divide by zero, s is always positive due to sqrt
            // if s close to zero then direction of axis not important
            // if it is important that axis is normalized then replace with x=1; y=z=0;
            axis_angle.x = q1.x;
            axis_angle.y = q1.y;
            axis_angle.z = q1.z;
        } else {
            // normalize axis
            axis_angle.x = q1.x / s;
            axis_angle.y = q1.y / s;
            axis_angle.z = q1.z / s;
        }

        axis_angle
    }

    /// The axis and angle (radians) of the rotation.
    pub fn to_axis_angle_parts(&self) -> (Vector3, f64) {
        let result = self.to_axis_angle();
        (result.xyz(), result.w)
    }

    /// The length.
    pub fn length(&self) -> f64 {
        (self.w * self.w + self.xyz().length_squared()).sqrt()
    }

    /// The squared length.
    pub fn length_squared(&self) -> f64 {
        self.w * self.w + self.xyz().length_squared()
    }

    /// Scales to unit length; a zero quaternion is left unchanged.
    pub fn normalize(&mut self) {
        let length = self.length();
        if length != 0.0 {
            let scale = 1.0 / length;
            self.set_xyz(self.xyz() * scale);
            self.w *= scale;
        }
    }

    /// Negates the vector part in place.
    pub fn conjugate(&mut self) {
        self.set_xyz(-self.xyz());
    }

    /// `left * right`: apply `right`, then `left`.
    pub fn multiply(left: Quaternion, right: Quaternion) -> Quaternion {
        Quaternion::from_xyz_w(
            right.w * left.xyz() + left.w * right.xyz() + left.xyz().cross(right.xyz()),
            left.w * right.w - left.xyz().dot(right.xyz()),
        )
    }

    /// Every component scaled.
    pub fn multiply_scalar(quaternion: Quaternion, scale: f64) -> Quaternion {
        Quaternion::new(
            quaternion.x * scale,
            quaternion.y * scale,
            quaternion.z * scale,
            quaternion.w * scale,
        )
    }

    /// The conjugate.
    pub fn conjugated(q: Quaternion) -> Quaternion {
        Quaternion::from_xyz_w(-q.xyz(), q.w)
    }

    /// The inverse; a zero quaternion comes back unchanged.
    pub fn invert(q: Quaternion) -> Quaternion {
        let length_sq = q.length_squared();
        if length_sq != 0.0 {
            let i = 1.0 / length_sq;
            Quaternion::from_xyz_w(q.xyz() * -i, q.w * i)
        } else {
            q
        }
    }

    /// A unit-length copy (no zero check, so zero gives NaN).
    pub fn normalized(q: Quaternion) -> Quaternion {
        let scale = 1.0 / q.length();
        Quaternion::from_xyz_w(q.xyz() * scale, q.w * scale)
    }

    /// The rotation for Euler angles: x first, then y, then z.
    pub fn from_euler_angles(rotation: Vector3) -> Quaternion {
        let x_rotation = Quaternion::from_axis_angle(Vector3::UNIT_X, rotation.x);
        let y_rotation = Quaternion::from_axis_angle(Vector3::UNIT_Y, rotation.y);
        let z_rotation = Quaternion::from_axis_angle(Vector3::UNIT_Z, rotation.z);

        //return xRotation * yRotation * zRotation;
        z_rotation * y_rotation * x_rotation
    }

    /// A rotation of `angle` radians about `axis`; a zero axis gives the identity.
    pub fn from_axis_angle(axis: Vector3, angle: f64) -> Quaternion {
        if axis.length_squared() == 0.0 {
            return Quaternion::IDENTITY;
        }

        let mut result = Quaternion::IDENTITY;

        let angle = angle * 0.5;
        let mut axis = axis;
        axis.normalize();
        result.set_xyz(axis * libm::sin(angle));
        result.w = libm::cos(angle);

        Quaternion::normalized(result)
    }

    /// Spherical blend from `q1` (0) to `q2` (1); small angles blend linearly.
    pub fn slerp(q1: Quaternion, q2: Quaternion, blend: f64) -> Quaternion {
        // if either input is zero, return the other.
        if q1.length_squared() == 0.0 {
            if q2.length_squared() == 0.0 {
                return Quaternion::IDENTITY;
            }
            return q2;
        } else if q2.length_squared() == 0.0 {
            return q1;
        }

        let mut q2 = q2;
        let mut cos_half_angle = q1.w * q2.w + q1.xyz().dot(q2.xyz());

        if cos_half_angle >= 1.0 || cos_half_angle <= -1.0 {
            // angle = 0.0, so just return one input.
            return q1;
        } else if cos_half_angle < 0.0 {
            q2.set_xyz(-q2.xyz());
            q2.w = -q2.w;
            cos_half_angle = -cos_half_angle;
        }

        // The threshold is 0.99 in single precision, widened, which fixes where slerp switches.
        let (blend_a, blend_b) = if cos_half_angle < f64::from(0.99f32) {
            // do proper slerp for big angles
            let half_angle = libm::acos(cos_half_angle);
            let sin_half_angle = libm::sin(half_angle);
            let one_over_sin_half_angle = 1.0 / sin_half_angle;
            (
                libm::sin(half_angle * (1.0 - blend)) * one_over_sin_half_angle,
                libm::sin(half_angle * blend) * one_over_sin_half_angle,
            )
        } else {
            // do lerp if angle is really small.
            (1.0 - blend, blend)
        };

        let result = Quaternion::from_xyz_w(
            blend_a * q1.xyz() + blend_b * q2.xyz(),
            blend_a * q1.w + blend_b * q2.w,
        );
        if result.length_squared() > 0.0 {
            Quaternion::normalized(result)
        } else {
            Quaternion::IDENTITY
        }
    }
}

impl Add for Quaternion {
    type Output = Quaternion;
    fn add(self, right: Quaternion) -> Quaternion {
        Quaternion::from_xyz_w(self.xyz() + right.xyz(), self.w + right.w)
    }
}

impl Sub for Quaternion {
    type Output = Quaternion;
    fn sub(self, right: Quaternion) -> Quaternion {
        Quaternion::from_xyz_w(self.xyz() - right.xyz(), self.w - right.w)
    }
}

impl Mul for Quaternion {
    type Output = Quaternion;
    fn mul(self, right: Quaternion) -> Quaternion {
        Quaternion::multiply(self, right)
    }
}

impl Mul<f64> for Quaternion {
    type Output = Quaternion;
    fn mul(self, scale: f64) -> Quaternion {
        Quaternion::multiply_scalar(self, scale)
    }
}

impl Mul<Quaternion> for f64 {
    type Output = Quaternion;
    fn mul(self, quaternion: Quaternion) -> Quaternion {
        Quaternion::multiply_scalar(quaternion, self)
    }
}

/// "V: [x, y, z], W: w" (the vector part as `Vector3`'s text, w as
/// [`crate::number_format::format_f64`]).
impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "V: {}, W: {}",
            self.xyz(),
            crate::number_format::format_f64(self.w)
        )
    }
}

/// Hashes the component bits (see `float_hash`).
impl std::hash::Hash for Quaternion {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        crate::float_hash::hash_f64(self.x, state);
        crate::float_hash::hash_f64(self.y, state);
        crate::float_hash::hash_f64(self.z, state);
        crate::float_hash::hash_f64(self.w, state);
    }
}
