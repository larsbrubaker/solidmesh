//! The matrix and quaternion transforms of the vector types, kept apart from the vector files
//! because they need `Matrix4X4` and `Quaternion`: the quaternion rotation of `Vector2` and
//! `Vector2Float`, the matrix and quaternion transforms of `Vector4`, and the position, vector,
//! normal and perspective transforms of `Vector3` and `Vector3Float`.
//!
//! Row vectors: a position is `p * M` with `row3` as the translation. `transform_vector`'s
//! column dot products are the same sums in the same order as a row-by-row form would take.
//! The quaternion transform on `Vector3` and `Vector4` is `transform_by_quaternion`; the array
//! forms are the free functions [`transform_array`], [`transform_array_by_quaternion`] and
//! [`transform_array_f32`]. `Vector3Float` has no position, perspective or quaternion
//! transform: nothing needs them.

use crate::{Matrix4X4, Quaternion, Vector2, Vector2Float, Vector3, Vector3Float, Vector4};

/// `quat * (v, 0) * quat^-1`, the full quaternion sandwich the 2D and 4D transforms use.
fn rotate_by_quaternion(v: Quaternion, quat: Quaternion) -> Quaternion {
    let i = Quaternion::invert(quat);
    let t = Quaternion::multiply(quat, v);
    Quaternion::multiply(t, i)
}

impl Vector2 {
    /// Rotates this vector (as (x, y, 0)) by a quaternion and keeps x and y.
    pub fn transform(self, quat: Quaternion) -> Vector2 {
        let v = rotate_by_quaternion(Quaternion::new(self.x, self.y, 0.0, 0.0), quat);
        Vector2::new(v.x, v.y)
    }
}

impl Vector2Float {
    /// Rotates this vector (widened, as (x, y, 0)) by a quaternion and keeps x and y, narrowed.
    pub fn transform(self, quat: Quaternion) -> Vector2Float {
        let v = rotate_by_quaternion(
            Quaternion::new(f64::from(self.x), f64::from(self.y), 0.0, 0.0),
            quat,
        );
        Vector2Float::new(v.x, v.y)
    }
}

impl Vector4 {
    /// `self * mat`.
    pub fn transform(self, mat: Matrix4X4) -> Vector4 {
        let vec = self;
        Vector4::new(
            vec.x * mat.row0.x + vec.y * mat.row1.x + vec.z * mat.row2.x + vec.w * mat.row3.x,
            vec.x * mat.row0.y + vec.y * mat.row1.y + vec.z * mat.row2.y + vec.w * mat.row3.y,
            vec.x * mat.row0.z + vec.y * mat.row1.z + vec.z * mat.row2.z + vec.w * mat.row3.z,
            vec.x * mat.row0.w + vec.y * mat.row1.w + vec.z * mat.row2.w + vec.w * mat.row3.w,
        )
    }

    /// `quat * self * quat^-1`.
    pub fn transform_by_quaternion(self, quat: Quaternion) -> Vector4 {
        let v = rotate_by_quaternion(Quaternion::new(self.x, self.y, self.z, self.w), quat);
        Vector4::new(v.x, v.y, v.z, v.w)
    }
}

impl Vector3 {
    /// Transforms a direction: the upper 3x3 only, no translation.
    pub fn transform_vector(self, mat: Matrix4X4) -> Vector3 {
        Vector3::new(
            self.dot(Vector3::from_vector4(mat.column0())),
            self.dot(Vector3::from_vector4(mat.column1())),
            self.dot(Vector3::from_vector4(mat.column2())),
        )
    }

    /// Transforms a normal by the inverse transpose of `mat` (inverts `mat`; use
    /// [`Vector3::transform_normal_inverse`] when the inverse is at hand).
    pub fn transform_normal(self, mat: Matrix4X4) -> Vector3 {
        let inverse = mat.inverted();
        self.transform_normal_inverse(inverse)
    }

    /// Transforms a normal by the transpose of `inv_mat`, the already-inverted matrix.
    pub fn transform_normal_inverse(self, inv_mat: Matrix4X4) -> Vector3 {
        Vector3::new(
            self.dot(Vector3::from_vector4(inv_mat.row0)),
            self.dot(Vector3::from_vector4(inv_mat.row1)),
            self.dot(Vector3::from_vector4(inv_mat.row2)),
        )
    }

    /// Transforms a position: the upper 3x3 plus the translation.
    pub fn transform_position(self, mat: Matrix4X4) -> Vector3 {
        Vector3::new(
            self.dot(Vector3::from_vector4(mat.column0())) + mat.row3.x,
            self.dot(Vector3::from_vector4(mat.column1())) + mat.row3.y,
            self.dot(Vector3::from_vector4(mat.column2())) + mat.row3.z,
        )
    }

    /// Transforms a position: `(x, y, z, 1) * mat` (the same sums as `transform_position`).
    pub fn transform(self, mat: Matrix4X4) -> Vector3 {
        let vec = self;
        Vector3::new(
            vec.x * mat.row0.x + vec.y * mat.row1.x + vec.z * mat.row2.x + mat.row3.x,
            vec.x * mat.row0.y + vec.y * mat.row1.y + vec.z * mat.row2.y + mat.row3.y,
            vec.x * mat.row0.z + vec.y * mat.row1.z + vec.z * mat.row2.z + mat.row3.z,
        )
    }

    /// Rotates by a quaternion.
    pub fn transform_by_quaternion(self, quat: Quaternion) -> Vector3 {
        // Since vec.W == 0, we can optimize quat * vec * quat^-1 as follows:
        // vec + 2.0 * cross(quat.xyz, cross(quat.xyz, vec) + quat.w * vec)
        let vec = self;
        let xyz = quat.xyz();
        let temp = xyz.cross(vec);
        let temp2 = vec * quat.w;
        let temp = temp + temp2;
        let temp = xyz.cross(temp);
        let temp = temp * 2.0;
        vec + temp
    }

    /// `(x, y, z, 1) * mat`, divided by the resulting w: a position through a projection, so
    /// the translation row applies.
    pub fn transform_perspective(self, mat: Matrix4X4) -> Vector3 {
        let v = Vector4::new(self.x, self.y, self.z, 1.0).transform(mat);
        Vector3::new(v.x / v.w, v.y / v.w, v.z / v.w)
    }
}

/// Transforms every position in place.
pub fn transform_array(vec_array: &mut [Vector3], mat: Matrix4X4) {
    for v in vec_array.iter_mut() {
        *v = v.transform(mat);
    }
}

/// Rotates every vector in place.
pub fn transform_array_by_quaternion(vec_array: &mut [Vector3], rotation_quaternion: Quaternion) {
    for v in vec_array.iter_mut() {
        *v = v.transform_by_quaternion(rotation_quaternion);
    }
}

impl Vector3Float {
    /// Transforms a direction: the upper 3x3 only (columns narrowed, dot products in f32).
    pub fn transform_vector(self, mat: Matrix4X4) -> Vector3Float {
        Vector3Float::new_f32(
            self.dot(Vector3Float::from_vector4(mat.column0())),
            self.dot(Vector3Float::from_vector4(mat.column1())),
            self.dot(Vector3Float::from_vector4(mat.column2())),
        )
    }

    /// Transforms a normal by the inverse transpose of `mat`.
    pub fn transform_normal(self, mat: Matrix4X4) -> Vector3Float {
        let mut mat = mat;
        mat.invert();
        self.transform_normal_inverse(mat)
    }

    /// Transforms a normal by the transpose of `inv_mat`, the already-inverted matrix.
    pub fn transform_normal_inverse(self, inv_mat: Matrix4X4) -> Vector3Float {
        Vector3Float::new_f32(
            self.dot(Vector3Float::from_vector4(inv_mat.row0)),
            self.dot(Vector3Float::from_vector4(inv_mat.row1)),
            self.dot(Vector3Float::from_vector4(inv_mat.row2)),
        )
    }

    /// Transforms a position: widened, transformed as `(x, y, z, 1) * mat`, narrowed.
    pub fn transform(self, mat: Matrix4X4) -> Vector3Float {
        let vec = Vector3::from_vector3_float(self);
        let v4 = Vector4::new(vec.x, vec.y, vec.z, 1.0).transform(mat);
        Vector3Float::from_vector3(v4.xyz())
    }
}

/// Transforms every position in place.
pub fn transform_array_f32(vec_array: &mut [Vector3Float], mat: Matrix4X4) {
    for v in vec_array.iter_mut() {
        *v = v.transform(mat);
    }
}
