//! `Matrix4X4`: a 4x4 f64 matrix, **row-major with row vectors** (a point transforms as
//! `p * M`, translation lives in `row3`, and `a * b` applies `a` first). Derived from OpenTK:
//! the struct, its element accessors, inversion, multiplication, equality, formatting and
//! hashing. The `create_*`/`look_at`/frustum/perspective builders are in
//! `matrix4x4_create.rs`; its JSON shape is in `serde_impls.rs`.
//!
//! Elements are `m11()`..`m44()` with `set_m11()`..`set_m44()`; `mult` is also the `*`
//! operator; `equals_within` compares with a tolerance; `to_string_with` takes the
//! per-component formatter (as `Vector4::to_string_with` does); the float and double array
//! getters return fixed arrays. There is no parser from text: nothing saves a matrix that way.
//!
//! Inverting a singular matrix (|det| below the smallest subnormal) returns the input matrix
//! unchanged, so a degenerate transform never fills a scene with NaN.

use crate::cs_math::max_f64;
use crate::{Quaternion, Vector3, Vector4, LONG_HASH_SEED};
use std::fmt;
use std::ops::{Index, IndexMut, Mul};

/// A 4x4 f64 matrix of four row vectors.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Matrix4X4 {
    pub row0: Vector4,
    pub row1: Vector4,
    pub row2: Vector4,
    pub row3: Vector4,
}

/// The smallest positive subnormal double.
const DOUBLE_EPSILON: f64 = f64::from_bits(1);

macro_rules! element_accessors {
    ($($get:ident, $set:ident, $row:ident, $c:ident;)*) => {
        $(
            #[doc = concat!("Reads element `", stringify!($get), "`.")]
            pub fn $get(&self) -> f64 {
                self.$row.$c
            }

            #[doc = concat!("Writes element `", stringify!($get), "`.")]
            pub fn $set(&mut self, value: f64) {
                self.$row.$c = value;
            }
        )*
    };
}

impl Matrix4X4 {
    /// The identity matrix.
    pub const IDENTITY: Matrix4X4 = Matrix4X4::from_rows(
        Vector4::UNIT_X,
        Vector4::UNIT_Y,
        Vector4::UNIT_Z,
        Vector4::UNIT_W,
    );

    /// A matrix from four rows.
    pub const fn from_rows(row0: Vector4, row1: Vector4, row2: Vector4, row3: Vector4) -> Self {
        Matrix4X4 {
            row0,
            row1,
            row2,
            row3,
        }
    }

    /// A matrix from its sixteen elements, row by row.
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        m00: f64,
        m01: f64,
        m02: f64,
        m03: f64,
        m10: f64,
        m11: f64,
        m12: f64,
        m13: f64,
        m20: f64,
        m21: f64,
        m22: f64,
        m23: f64,
        m30: f64,
        m31: f64,
        m32: f64,
        m33: f64,
    ) -> Self {
        Matrix4X4 {
            row0: Vector4::new(m00, m01, m02, m03),
            row1: Vector4::new(m10, m11, m12, m13),
            row2: Vector4::new(m20, m21, m22, m23),
            row3: Vector4::new(m30, m31, m32, m33),
        }
    }

    /// The first sixteen values, row by row; panics on fewer (a caller bug).
    pub fn from_slice(double16: &[f64]) -> Self {
        let d = double16;
        Self::new(
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11], d[12], d[13],
            d[14], d[15],
        )
    }

    /// The first sixteen values widened, row by row.
    pub fn from_slice_f32(float16: &[f32]) -> Self {
        let d: Vec<f64> = float16.iter().take(16).map(|&v| f64::from(v)).collect();
        Self::from_slice(&d)
    }

    /// The determinant, expanded term by term in a fixed order so the last bits never change.
    pub fn determinant(&self) -> f64 {
        let (r0, r1, r2, r3) = (self.row0, self.row1, self.row2, self.row3);
        r0.x * r1.y * r2.z * r3.w - r0.x * r1.y * r2.w * r3.z + r0.x * r1.z * r2.w * r3.y
            - r0.x * r1.z * r2.y * r3.w
            + r0.x * r1.w * r2.y * r3.z
            - r0.x * r1.w * r2.z * r3.y
            - r0.y * r1.z * r2.w * r3.x
            + r0.y * r1.z * r2.x * r3.w
            - r0.y * r1.w * r2.x * r3.z
            + r0.y * r1.w * r2.z * r3.x
            - r0.y * r1.x * r2.z * r3.w
            + r0.y * r1.x * r2.w * r3.z
            + r0.z * r1.w * r2.x * r3.y
            - r0.z * r1.w * r2.y * r3.x
            + r0.z * r1.x * r2.y * r3.w
            - r0.z * r1.x * r2.w * r3.y
            + r0.z * r1.y * r2.w * r3.x
            - r0.z * r1.y * r2.x * r3.w
            - r0.w * r1.x * r2.y * r3.z
            + r0.w * r1.x * r2.z * r3.y
            - r0.w * r1.y * r2.z * r3.x
            + r0.w * r1.y * r2.x * r3.z
            - r0.w * r1.z * r2.x * r3.y
            + r0.w * r1.z * r2.y * r3.x
    }

    /// The translation part, x/y/z of `row3`.
    pub fn position(&self) -> Vector3 {
        Vector3::from_vector4(self.row3)
    }

    /// `row3`.
    pub fn translation(&self) -> Vector4 {
        self.row3
    }

    /// The first column.
    pub fn column0(&self) -> Vector4 {
        Vector4::new(self.row0.x, self.row1.x, self.row2.x, self.row3.x)
    }

    /// The second column.
    pub fn column1(&self) -> Vector4 {
        Vector4::new(self.row0.y, self.row1.y, self.row2.y, self.row3.y)
    }

    /// The third column.
    pub fn column2(&self) -> Vector4 {
        Vector4::new(self.row0.z, self.row1.z, self.row2.z, self.row3.z)
    }

    /// The fourth column.
    pub fn column3(&self) -> Vector4 {
        Vector4::new(self.row0.w, self.row1.w, self.row2.w, self.row3.w)
    }

    element_accessors! {
        m11, set_m11, row0, x; m12, set_m12, row0, y; m13, set_m13, row0, z; m14, set_m14, row0, w;
        m21, set_m21, row1, x; m22, set_m22, row1, y; m23, set_m23, row1, z; m24, set_m24, row1, w;
        m31, set_m31, row2, x; m32, set_m32, row2, y; m33, set_m33, row2, z; m34, set_m34, row2, w;
        m41, set_m41, row3, x; m42, set_m42, row3, y; m43, set_m43, row3, z; m44, set_m44, row3, w;
    }

    /// The rotation this matrix holds, as a quaternion.
    pub fn get_rotation(&self) -> Quaternion {
        let mut rotation = Quaternion::IDENTITY;
        let tr = self.m11() + self.m22() + self.m33();
        if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0; // S=4*qw
            rotation.w = 0.25 * s;
            rotation.x = (self.m32() - self.m23()) / s;
            rotation.y = (self.m13() - self.m31()) / s;
            rotation.z = (self.m21() - self.m12()) / s;
        } else if (self.m11() > self.m22()) & (self.m11() > self.m33()) {
            let s = (1.0 + self.m11() - self.m22() - self.m33()).sqrt() * 2.0; // S=4*qx
            rotation.w = (self.m32() - self.m23()) / s;
            rotation.x = 0.25 * s;
            rotation.y = (self.m12() + self.m21()) / s;
            rotation.z = (self.m13() + self.m31()) / s;
        } else if self.m22() > self.m33() {
            let s = (1.0 + self.m22() - self.m11() - self.m33()).sqrt() * 2.0; // S=4*qy
            rotation.w = (self.m13() - self.m31()) / s;
            rotation.x = (self.m12() + self.m21()) / s;
            rotation.y = 0.25 * s;
            rotation.z = (self.m23() + self.m32()) / s;
        } else {
            let s = (1.0 + self.m33() - self.m11() - self.m22()).sqrt() * 2.0; // S=4*qz
            rotation.w = (self.m21() - self.m12()) / s;
            rotation.x = (self.m13() + self.m31()) / s;
            rotation.y = (self.m23() + self.m32()) / s;
            rotation.z = 0.25 * s;
        }

        rotation
    }

    /// Inverts this matrix in place.
    pub fn invert(&mut self) {
        *self = self.inverted();
    }

    /// The inverse. A singular matrix comes back unchanged (see the module header).
    pub fn inverted(&self) -> Matrix4X4 {
        // Original implementation can be found here:
        // https://github.com/dotnet/runtime/blob/79ae74f5ca5c8a6fe3a48935e85bd7374959c570/src/libraries/System.Private.CoreLib/src/System/Numerics/Matrix4x4.cs#L1556
        let mat = self;
        let (a, b, c, d) = (mat.m11(), mat.m21(), mat.m31(), mat.m41());
        let (e, f, g, h) = (mat.m12(), mat.m22(), mat.m32(), mat.m42());
        let (i, j, k, l) = (mat.m13(), mat.m23(), mat.m33(), mat.m43());
        let (m, n, o, p) = (mat.m14(), mat.m24(), mat.m34(), mat.m44());

        let kp_lo = k * p - l * o;
        let jp_ln = j * p - l * n;
        let jo_kn = j * o - k * n;
        let ip_lm = i * p - l * m;
        let io_km = i * o - k * m;
        let in_jm = i * n - j * m;

        let a11 = f * kp_lo - g * jp_ln + h * jo_kn;
        let a12 = -(e * kp_lo - g * ip_lm + h * io_km);
        let a13 = e * jp_ln - f * ip_lm + h * in_jm;
        let a14 = -(e * jo_kn - f * io_km + g * in_jm);

        let det = a * a11 + b * a12 + c * a13 + d * a14;

        if det.abs() < DOUBLE_EPSILON {
            // return the input matrix rather than throw an exception (release only)
            return *mat;
        }

        let inv_det = 1.0 / det;

        let row0 = Vector4::new(a11, a12, a13, a14) * inv_det;

        let row1 = Vector4::new(
            -(b * kp_lo - c * jp_ln + d * jo_kn),
            a * kp_lo - c * ip_lm + d * io_km,
            -(a * jp_ln - b * ip_lm + d * in_jm),
            a * jo_kn - b * io_km + c * in_jm,
        ) * inv_det;

        let gp_ho = g * p - h * o;
        let fp_hn = f * p - h * n;
        let fo_gn = f * o - g * n;
        let ep_hm = e * p - h * m;
        let eo_gm = e * o - g * m;
        let en_fm = e * n - f * m;

        let row2 = Vector4::new(
            b * gp_ho - c * fp_hn + d * fo_gn,
            -(a * gp_ho - c * ep_hm + d * eo_gm),
            a * fp_hn - b * ep_hm + d * en_fm,
            -(a * fo_gn - b * eo_gm + c * en_fm),
        ) * inv_det;

        let gl_hk = g * l - h * k;
        let fl_hj = f * l - h * j;
        let fk_gj = f * k - g * j;
        let el_hi = e * l - h * i;
        let ek_gi = e * k - g * i;
        let ej_fi = e * j - f * i;

        let row3 = Vector4::new(
            -(b * gl_hk - c * fl_hj + d * fk_gj),
            a * gl_hk - c * el_hi + d * ek_gi,
            -(a * fl_hj - b * el_hi + d * ej_fi),
            a * fk_gj - b * ek_gi + c * ej_fi,
        ) * inv_det;

        Matrix4X4::from_rows(row0, row1, row2, row3)
    }

    /// Transposes this matrix in place.
    pub fn transpose(&mut self) {
        *self = self.transposed();
    }

    /// The transpose.
    pub fn transposed(&self) -> Matrix4X4 {
        Matrix4X4::from_rows(
            self.column0(),
            self.column1(),
            self.column2(),
            self.column3(),
        )
    }

    /// False if any element is NaN or infinite.
    pub fn is_valid(&self) -> bool {
        self.row0.is_valid() && self.row1.is_valid() && self.row2.is_valid() && self.row3.is_valid()
    }

    /// `left * right`.
    pub fn mult(left: Matrix4X4, right: Matrix4X4) -> Matrix4X4 {
        let (l, r) = (left, right);
        let mut result = Matrix4X4::default();
        result
            .set_m11(l.m11() * r.m11() + l.m12() * r.m21() + l.m13() * r.m31() + l.m14() * r.m41());
        result
            .set_m12(l.m11() * r.m12() + l.m12() * r.m22() + l.m13() * r.m32() + l.m14() * r.m42());
        result
            .set_m13(l.m11() * r.m13() + l.m12() * r.m23() + l.m13() * r.m33() + l.m14() * r.m43());
        result
            .set_m14(l.m11() * r.m14() + l.m12() * r.m24() + l.m13() * r.m34() + l.m14() * r.m44());
        result
            .set_m21(l.m21() * r.m11() + l.m22() * r.m21() + l.m23() * r.m31() + l.m24() * r.m41());
        result
            .set_m22(l.m21() * r.m12() + l.m22() * r.m22() + l.m23() * r.m32() + l.m24() * r.m42());
        result
            .set_m23(l.m21() * r.m13() + l.m22() * r.m23() + l.m23() * r.m33() + l.m24() * r.m43());
        result
            .set_m24(l.m21() * r.m14() + l.m22() * r.m24() + l.m23() * r.m34() + l.m24() * r.m44());
        result
            .set_m31(l.m31() * r.m11() + l.m32() * r.m21() + l.m33() * r.m31() + l.m34() * r.m41());
        result
            .set_m32(l.m31() * r.m12() + l.m32() * r.m22() + l.m33() * r.m32() + l.m34() * r.m42());
        result
            .set_m33(l.m31() * r.m13() + l.m32() * r.m23() + l.m33() * r.m33() + l.m34() * r.m43());
        result
            .set_m34(l.m31() * r.m14() + l.m32() * r.m24() + l.m33() * r.m34() + l.m34() * r.m44());
        result
            .set_m41(l.m41() * r.m11() + l.m42() * r.m21() + l.m43() * r.m31() + l.m44() * r.m41());
        result
            .set_m42(l.m41() * r.m12() + l.m42() * r.m22() + l.m43() * r.m32() + l.m44() * r.m42());
        result
            .set_m43(l.m41() * r.m13() + l.m42() * r.m23() + l.m43() * r.m33() + l.m44() * r.m43());
        result
            .set_m44(l.m41() * r.m14() + l.m42() * r.m24() + l.m43() * r.m34() + l.m44() * r.m44());
        result
    }

    /// True when every element is strictly within `error_range` of `other`'s.
    pub fn equals_within(&self, other: Matrix4X4, error_range: f64) -> bool {
        self.row0.equals_within(other.row0, error_range)
            && self.row1.equals_within(other.row1, error_range)
            && self.row2.equals_within(other.row2, error_range)
            && self.row3.equals_within(other.row3, error_range)
    }

    /// True when both matrices move a set of test vectors to the same place, within
    /// `relative_error` of their lengths and directions, and have the same translation.
    pub fn are_transformations_equivalent(&self, matrix2: Matrix4X4, relative_error: f64) -> bool {
        let matrix1 = *self;
        let test_vectors = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 1.0),
            Vector3::new(1.0, 1.0, 1.0),
        ];

        for test_vector in test_vectors {
            let result1 = test_vector.transform(matrix1);
            let result2 = test_vector.transform(matrix2);

            let length1_sqrd = result1.length_squared();
            let length2_sqrd = result2.length_squared();
            let allowed_error_sqrd = length1_sqrd * relative_error * 2.0;

            if (length1_sqrd - length2_sqrd).abs() > allowed_error_sqrd {
                return false;
            }

            if length1_sqrd > DOUBLE_EPSILON && length2_sqrd > DOUBLE_EPSILON {
                let normalized1 = result1.get_normal();
                let normalized2 = result2.get_normal();
                let dot = normalized1.dot(normalized2);
                if dot < 1.0 - relative_error {
                    return false;
                }
            }
        }

        let translation1 = matrix1.translation();
        let translation2 = matrix2.translation();
        let translation_magnitude_sqrd =
            max_f64(translation1.length_squared(), translation2.length_squared());
        let translation_error_sqrd = translation_magnitude_sqrd * relative_error * 2.0;

        (translation1 - translation2).length_squared() <= translation_error_sqrd
    }

    /// The sixteen elements narrowed to f32, row by row.
    pub fn get_as_float_array(&self) -> [f32; 16] {
        let d = self.get_as_double_array();
        let mut contents = [0.0f32; 16];
        for (target, source) in contents.iter_mut().zip(d.iter()) {
            *target = *source as f32;
        }
        contents
    }

    /// The sixteen elements, row by row.
    pub fn get_as_double_array(&self) -> [f64; 16] {
        let (r0, r1, r2, r3) = (self.row0, self.row1, self.row2, self.row3);
        [
            r0.x, r0.y, r0.z, r0.w, r1.x, r1.y, r1.z, r1.w, r2.x, r2.y, r2.z, r2.w, r3.x, r3.y,
            r3.z, r3.w,
        ]
    }

    /// FNV-1a over the rows, starting from [`LONG_HASH_SEED`].
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// FNV-1a over the rows, starting from `hash`.
    pub fn get_long_hash_code_with(&self, hash: u64) -> u64 {
        let hash = self.row0.get_long_hash_code_with(hash);
        let hash = self.row1.get_long_hash_code_with(hash);
        let hash = self.row2.get_long_hash_code_with(hash);
        self.row3.get_long_hash_code_with(hash)
    }

    /// The four rows on separate lines (blank lines between), each element through `format`;
    /// the first three columns are left-padded with '0' to a common width.
    pub fn to_string_with(&self, format: impl Fn(f64) -> String) -> String {
        let rows = [self.row0, self.row1, self.row2, self.row3];
        let text: Vec<[String; 4]> = rows
            .iter()
            .map(|r| [format(r.x), format(r.y), format(r.z), format(r.w)])
            .collect();
        let width = |column: usize| text.iter().map(|r| r[column].len()).max().unwrap_or(0);
        let widths = [width(0), width(1), width(2)];
        let pad = |s: &str, w: usize| format!("{}{}", "0".repeat(w.saturating_sub(s.len())), s);
        let lines: Vec<String> = text
            .iter()
            .map(|r| {
                format!(
                    "{}, {}, {}, {}\n",
                    pad(&r[0], widths[0]),
                    pad(&r[1], widths[1]),
                    pad(&r[2], widths[2]),
                    r[3]
                )
            })
            .collect();
        format!("{}\n{}\n{}\n{}", lines[0], lines[1], lines[2], lines[3])
    }
}

/// `[row, column]`; a row outside 0..=3 is a caller bug and panics.
impl Index<(usize, usize)> for Matrix4X4 {
    type Output = f64;
    fn index(&self, (row, column): (usize, usize)) -> &f64 {
        match row {
            0 => &self.row0[column],
            1 => &self.row1[column],
            2 => &self.row2[column],
            3 => &self.row3[column],
            _ => panic!("Matrix4X4 row {row} out of range"),
        }
    }
}

impl IndexMut<(usize, usize)> for Matrix4X4 {
    fn index_mut(&mut self, (row, column): (usize, usize)) -> &mut f64 {
        match row {
            0 => &mut self.row0[column],
            1 => &mut self.row1[column],
            2 => &mut self.row2[column],
            3 => &mut self.row3[column],
            _ => panic!("Matrix4X4 row {row} out of range"),
        }
    }
}

impl Mul for Matrix4X4 {
    type Output = Matrix4X4;
    fn mul(self, right: Matrix4X4) -> Matrix4X4 {
        Matrix4X4::mult(self, right)
    }
}

/// "row0,  row1,  row2,  row3", each row as `Vector4`'s text.
impl fmt::Display for Matrix4X4 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{},  {},  {},  {}",
            self.row0, self.row1, self.row2, self.row3
        )
    }
}

/// Hashes the rows' component bits (see `float_hash`).
impl std::hash::Hash for Matrix4X4 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.row0.hash(state);
        self.row1.hash(state);
        self.row2.hash(state);
        self.row3.hash(state);
    }
}
