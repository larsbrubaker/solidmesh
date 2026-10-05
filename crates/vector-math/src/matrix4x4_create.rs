//! The `Matrix4X4` builders from agg-sharp `VectorMath/Matrix4x4.cs`: rotations, translation,
//! scale, orthographic and perspective projections, `LookAt`, `Frustum` and `Perspective`.
//! The matrix type itself is in `matrix4x4.rs`. Row vectors: translation goes in `row3`.
//!
//! Mapping notes (C# overloads): `CreateRotation(Vector3 radians)` is `create_rotation`;
//! `CreateRotation(Vector3 axis, double angle)` is `create_rotation_axis_angle`;
//! `CreateRotation(Quaternion)` is `create_rotation_quaternion`; `CreateRotation(Vector3,
//! Vector3)` is `create_rotation_between`; `CreateTranslation(Vector3)` is
//! `create_translation_vector`; `CreateScale(double)` is `create_scale_uniform` and
//! `CreateScale(Vector3)` `create_scale_vector`; the nine-double `LookAt` is
//! `look_at_components`. The `out` overloads compute the same as the value ones and are folded
//! in. `CreatePerspectiveFieldOfView`/`CreatePerspectiveOffCenter` throw
//! `ArgumentOutOfRangeException` in C# and return `Err(MathError::ArgumentOutOfRange)` here.

use crate::{MathError, Matrix4X4, Quaternion, Vector3, Vector4};

impl Matrix4X4 {
    /// A rotation of `angle` radians about `axis`, with the angle negated as the C# does (C#
    /// `CreateFromAxisAngle`; compare [`Matrix4X4::create_rotation_axis_angle`]).
    pub fn create_from_axis_angle(axis: Vector3, angle: f64) -> Matrix4X4 {
        let cos = (-angle).cos();
        let sin = (-angle).sin();
        let t = 1.0 - cos;

        let mut axis = axis;
        axis.normalize();

        Matrix4X4::new(
            t * axis.x * axis.x + cos,
            t * axis.x * axis.y - sin * axis.z,
            t * axis.x * axis.z + sin * axis.y,
            0.0,
            t * axis.x * axis.y + sin * axis.z,
            t * axis.y * axis.y + cos,
            t * axis.y * axis.z - sin * axis.x,
            0.0,
            t * axis.x * axis.z - sin * axis.y,
            t * axis.y * axis.z + sin * axis.x,
            t * axis.z * axis.z + cos,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        )
    }

    /// The rotation for Euler angles (x, then y, then z), through `Quaternion::from_euler_angles`.
    pub fn create_rotation(radians: Vector3) -> Matrix4X4 {
        Matrix4X4::create_rotation_quaternion(Quaternion::from_euler_angles(radians))
    }

    /// A rotation of `angle` radians about the x axis.
    pub fn create_rotation_x(angle: f64) -> Matrix4X4 {
        let cos = angle.cos();
        let sin = angle.sin();
        Matrix4X4::from_rows(
            Vector4::UNIT_X,
            Vector4::new(0.0, cos, sin, 0.0),
            Vector4::new(0.0, -sin, cos, 0.0),
            Vector4::UNIT_W,
        )
    }

    /// A rotation of `angle` radians about the y axis.
    pub fn create_rotation_y(angle: f64) -> Matrix4X4 {
        let cos = angle.cos();
        let sin = angle.sin();
        Matrix4X4::from_rows(
            Vector4::new(cos, 0.0, -sin, 0.0),
            Vector4::UNIT_Y,
            Vector4::new(sin, 0.0, cos, 0.0),
            Vector4::UNIT_W,
        )
    }

    /// A rotation of `angle` radians about the z axis.
    pub fn create_rotation_z(angle: f64) -> Matrix4X4 {
        let cos = angle.cos();
        let sin = angle.sin();
        Matrix4X4::from_rows(
            Vector4::new(cos, sin, 0.0, 0.0),
            Vector4::new(-sin, cos, 0.0, 0.0),
            Vector4::UNIT_Z,
            Vector4::UNIT_W,
        )
    }

    /// A rotation of `angle` radians about `axis` (C# `CreateRotation(Vector3, double)`).
    pub fn create_rotation_axis_angle(axis: Vector3, angle: f64) -> Matrix4X4 {
        let cos = angle.cos();
        let sin = angle.sin();
        let t = 1.0 - cos;

        let mut axis = axis;
        axis.normalize();

        Matrix4X4::from_rows(
            Vector4::new(
                t * axis.x * axis.x + cos,
                t * axis.x * axis.y - sin * axis.z,
                t * axis.x * axis.z + sin * axis.y,
                0.0,
            ),
            Vector4::new(
                t * axis.x * axis.y + sin * axis.z,
                t * axis.y * axis.y + cos,
                t * axis.y * axis.z - sin * axis.x,
                0.0,
            ),
            Vector4::new(
                t * axis.x * axis.z - sin * axis.y,
                t * axis.y * axis.z + sin * axis.x,
                t * axis.z * axis.z + cos,
                0.0,
            ),
            Vector4::UNIT_W,
        )
    }

    /// The rotation `q` holds (C# `CreateRotation(Quaternion)`, via its axis and angle).
    pub fn create_rotation_quaternion(q: Quaternion) -> Matrix4X4 {
        let (axis, angle) = q.to_axis_angle_parts();
        Matrix4X4::create_rotation_axis_angle(axis, angle)
    }

    /// The rotation turning `starting_direction` to `ending_direction`.
    pub fn create_rotation_between(
        starting_direction: Vector3,
        ending_direction: Vector3,
    ) -> Matrix4X4 {
        let q = Quaternion::from_directions(starting_direction, ending_direction);
        Matrix4X4::create_rotation_quaternion(q)
    }

    /// A translation.
    pub fn create_translation(x: f64, y: f64, z: f64) -> Matrix4X4 {
        let mut result = Matrix4X4::IDENTITY;
        result.row3 = Vector4::new(x, y, z, 1.0);
        result
    }

    /// A translation by `vector` (C# `CreateTranslation(Vector3)`).
    pub fn create_translation_vector(vector: Vector3) -> Matrix4X4 {
        Matrix4X4::create_translation(vector.x, vector.y, vector.z)
    }

    /// An orthographic projection centered on the view axis.
    pub fn create_orthographic(width: f64, height: f64, z_near: f64, z_far: f64) -> Matrix4X4 {
        Matrix4X4::create_orthographic_off_center(
            -width / 2.0,
            width / 2.0,
            -height / 2.0,
            height / 2.0,
            z_near,
            z_far,
        )
    }

    /// An orthographic projection of the given view box.
    pub fn create_orthographic_off_center(
        left: f64,
        right: f64,
        bottom: f64,
        top: f64,
        z_near: f64,
        z_far: f64,
    ) -> Matrix4X4 {
        let mut result = Matrix4X4::default();

        let inv_rl = 1.0 / (right - left);
        let inv_tb = 1.0 / (top - bottom);
        let inv_fn = 1.0 / (z_far - z_near);

        result.set_m11(2.0 * inv_rl);
        result.set_m22(2.0 * inv_tb);
        result.set_m33(-2.0 * inv_fn);

        result.set_m41(-(right + left) * inv_rl);
        result.set_m42(-(top + bottom) * inv_tb);
        result.set_m43(-(z_far + z_near) * inv_fn);
        result.set_m44(1.0);
        result
    }

    /// A perspective projection from a vertical field of view. Errors (C#
    /// `ArgumentOutOfRangeException`) name the offending parameter: "fovy" outside (0, pi],
    /// "aspect" <= 0, "zNear" <= 0 or >= `z_far`, "zFar" <= 0.
    pub fn create_perspective_field_of_view(
        fov_y_radians: f64,
        aspect_width_over_height: f64,
        z_near: f64,
        z_far: f64,
    ) -> Result<Matrix4X4, MathError> {
        if fov_y_radians <= 0.0 || fov_y_radians > std::f64::consts::PI {
            return Err(MathError::ArgumentOutOfRange("fovy"));
        }

        if aspect_width_over_height <= 0.0 {
            return Err(MathError::ArgumentOutOfRange("aspect"));
        }

        if z_near <= 0.0 {
            return Err(MathError::ArgumentOutOfRange("zNear"));
        }

        if z_far <= 0.0 {
            return Err(MathError::ArgumentOutOfRange("zFar"));
        }

        if z_near >= z_far {
            return Err(MathError::ArgumentOutOfRange("zNear"));
        }

        let y_max = z_near * (0.5 * fov_y_radians).tan();
        let y_min = -y_max;
        let x_min = y_min * aspect_width_over_height;
        let x_max = y_max * aspect_width_over_height;

        Matrix4X4::create_perspective_off_center(x_min, x_max, y_min, y_max, z_near, z_far)
    }

    /// A perspective projection of the given near-plane box. Errors name "zNear" (<= 0 or >=
    /// `z_far`) or "zFar" (<= 0).
    pub fn create_perspective_off_center(
        left: f64,
        right: f64,
        bottom: f64,
        top: f64,
        z_near: f64,
        z_far: f64,
    ) -> Result<Matrix4X4, MathError> {
        if z_near <= 0.0 {
            return Err(MathError::ArgumentOutOfRange("zNear"));
        }

        if z_far <= 0.0 {
            return Err(MathError::ArgumentOutOfRange("zFar"));
        }

        if z_near >= z_far {
            return Err(MathError::ArgumentOutOfRange("zNear"));
        }

        let x = (2.0 * z_near) / (right - left);
        let y = (2.0 * z_near) / (top - bottom);
        let a = (right + left) / (right - left);
        let b = (top + bottom) / (top - bottom);
        let c = -(z_far + z_near) / (z_far - z_near);
        let d = -(2.0 * z_far * z_near) / (z_far - z_near);

        Ok(Matrix4X4::new(
            x, 0.0, 0.0, 0.0, 0.0, y, 0.0, 0.0, a, b, c, -1.0, 0.0, 0.0, d, 0.0,
        ))
    }

    /// A uniform scale (C# `CreateScale(double)`).
    pub fn create_scale_uniform(scale: f64) -> Matrix4X4 {
        Matrix4X4::create_scale(scale, scale, scale)
    }

    /// A per-axis scale (C# `CreateScale(Vector3)`).
    pub fn create_scale_vector(scale: Vector3) -> Matrix4X4 {
        Matrix4X4::create_scale(scale.x, scale.y, scale.z)
    }

    /// A per-axis scale.
    pub fn create_scale(x: f64, y: f64, z: f64) -> Matrix4X4 {
        Matrix4X4::from_rows(
            Vector4::UNIT_X * x,
            Vector4::UNIT_Y * y,
            Vector4::UNIT_Z * z,
            Vector4::UNIT_W,
        )
    }

    /// A view matrix for a camera at `eye` looking at `target`, `up` being up.
    pub fn look_at(eye: Vector3, target: Vector3, up: Vector3) -> Matrix4X4 {
        // There are lots of examples of look at code on the internet that don't do all these
        // normalizes and also find the position through several dot products. The problem with
        // them is that they have a bit of error in that all the vectors aren't normal and need
        // to be.
        let z = Vector3::normalized(eye - target);
        let x = Vector3::normalized(up.cross(z));
        let y = Vector3::normalized(z.cross(x));

        let rot = Matrix4X4::from_rows(
            Vector4::new(x.x, y.x, z.x, 0.0),
            Vector4::new(x.y, y.y, z.y, 0.0),
            Vector4::new(x.z, y.z, z.z, 0.0),
            Vector4::UNIT_W,
        );

        let trans = Matrix4X4::create_translation_vector(-eye);

        trans * rot
    }

    /// [`Matrix4X4::look_at`] from components.
    #[allow(clippy::too_many_arguments)]
    pub fn look_at_components(
        eye_x: f64,
        eye_y: f64,
        eye_z: f64,
        target_x: f64,
        target_y: f64,
        target_z: f64,
        up_x: f64,
        up_y: f64,
        up_z: f64,
    ) -> Matrix4X4 {
        Matrix4X4::look_at(
            Vector3::new(eye_x, eye_y, eye_z),
            Vector3::new(target_x, target_y, target_z),
            Vector3::new(up_x, up_y, up_z),
        )
    }

    /// A perspective projection of the given near-plane box, without argument checks (C#
    /// `Frustum`).
    pub fn frustum(left: f64, right: f64, bottom: f64, top: f64, near: f64, far: f64) -> Matrix4X4 {
        let inv_rl = 1.0 / (right - left);
        let inv_tb = 1.0 / (top - bottom);
        let inv_fn = 1.0 / (far - near);
        Matrix4X4::from_rows(
            Vector4::new(2.0 * near * inv_rl, 0.0, 0.0, 0.0),
            Vector4::new(0.0, 2.0 * near * inv_tb, 0.0, 0.0),
            Vector4::new(
                (right + left) * inv_rl,
                (top + bottom) * inv_tb,
                -(far + near) * inv_fn,
                -1.0,
            ),
            Vector4::new(0.0, 0.0, -2.0 * far * near * inv_fn, 0.0),
        )
    }

    /// A perspective projection from a vertical field of view, without argument checks.
    pub fn perspective(fovy: f64, aspect: f64, near: f64, far: f64) -> Matrix4X4 {
        let y_max = near * (0.5 * fovy).tan();
        let y_min = -y_max;
        let x_min = y_min * aspect;
        let x_max = y_max * aspect;

        Matrix4X4::frustum(x_min, x_max, y_min, y_max, near, far)
    }
}
