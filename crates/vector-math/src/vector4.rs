//! `Vector4`: a 4D f64 vector, including the FNV-1a helpers every vector's long hash uses.
//!
//! Adding, subtracting and scaling are the operators; `multiply_components`/
//! `divide_components` work per component; `normalized` returns a unit-length copy;
//! `to_string_with` takes the per-component formatter. The matrix and quaternion transforms
//! are in `vector_transforms.rs`.

use crate::number_format::{format_f64, parse_f64};
use crate::{long_hash, Vector2, Vector3, LONG_HASH_SEED};
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

/// A 4D vector of f64.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector4 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Vector4 {
    pub const UNIT_X: Vector4 = Vector4::new(1.0, 0.0, 0.0, 0.0);
    pub const UNIT_Y: Vector4 = Vector4::new(0.0, 1.0, 0.0, 0.0);
    pub const UNIT_Z: Vector4 = Vector4::new(0.0, 0.0, 1.0, 0.0);
    pub const UNIT_W: Vector4 = Vector4::new(0.0, 0.0, 0.0, 1.0);
    pub const ZERO: Vector4 = Vector4::new(0.0, 0.0, 0.0, 0.0);
    pub const ONE: Vector4 = Vector4::new(1.0, 1.0, 1.0, 1.0);
    /// `Marshal.SizeOf(new Vector4())`.
    pub const SIZE_IN_BYTES: i32 = 32;

    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Vector4 { x, y, z, w }
    }

    /// `(v.x, v.y, 0, 0)`.
    pub fn from_vector2(v: Vector2) -> Self {
        Vector4::new(v.x, v.y, 0.0, 0.0)
    }

    /// `(v.x, v.y, v.z, 0)`.
    pub fn from_vector3(v: Vector3) -> Self {
        Vector4::new(v.x, v.y, v.z, 0.0)
    }

    /// `(v.x, v.y, v.z, w)`.
    pub fn from_vector3_w(v: Vector3, w: f64) -> Self {
        Vector4::new(v.x, v.y, v.z, w)
    }

    /// Parses "x, y, z, w" (culture-invariant text). Unparsable parts are 0; missing parts stay 0.
    pub fn parse(s: &str) -> Vector4 {
        let mut result = Vector4::ZERO;
        let values: Vec<f64> = s.split(',').map(parse_f64).collect();
        for (i, value) in values.iter().take(4).enumerate() {
            result[i] = *value;
        }
        result
    }

    /// The length of the vector.
    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    /// The squared length.
    pub fn length_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    /// Scales this vector to unit length (NaN for the zero vector).
    pub fn normalize(&mut self) {
        let scale = 1.0 / self.length();
        self.x *= scale;
        self.y *= scale;
        self.z *= scale;
        self.w *= scale;
    }

    /// False if any component is NaN or infinite.
    pub fn is_valid(&self) -> bool {
        [self.x, self.y, self.z, self.w]
            .iter()
            .all(|c| !(c.is_nan() || c.is_infinite()))
    }

    /// Component-wise product.
    pub fn multiply_components(vector: Vector4, scale: Vector4) -> Vector4 {
        Vector4::new(
            vector.x * scale.x,
            vector.y * scale.y,
            vector.z * scale.z,
            vector.w * scale.w,
        )
    }

    /// Component-wise quotient.
    pub fn divide_components(vector: Vector4, scale: Vector4) -> Vector4 {
        Vector4::new(
            vector.x / scale.x,
            vector.y / scale.y,
            vector.z / scale.z,
            vector.w / scale.w,
        )
    }

    /// Component-wise minimum.
    pub fn min(a: Vector4, b: Vector4) -> Vector4 {
        Vector4::new(
            if a.x < b.x { a.x } else { b.x },
            if a.y < b.y { a.y } else { b.y },
            if a.z < b.z { a.z } else { b.z },
            if a.w < b.w { a.w } else { b.w },
        )
    }

    /// Component-wise maximum.
    pub fn max(a: Vector4, b: Vector4) -> Vector4 {
        Vector4::new(
            if a.x > b.x { a.x } else { b.x },
            if a.y > b.y { a.y } else { b.y },
            if a.z > b.z { a.z } else { b.z },
            if a.w > b.w { a.w } else { b.w },
        )
    }

    /// Each component limited to `[min, max]` (NaN passes through).
    pub fn clamp(vec: Vector4, min: Vector4, max: Vector4) -> Vector4 {
        let pick = |value: f64, min: f64, max: f64| {
            if value < min {
                min
            } else if value > max {
                max
            } else {
                value
            }
        };
        Vector4::new(
            pick(vec.x, min.x, max.x),
            pick(vec.y, min.y, max.y),
            pick(vec.z, min.z, max.z),
            pick(vec.w, min.w, max.w),
        )
    }

    /// A unit-length copy of `vec`.
    pub fn normalized(vec: Vector4) -> Vector4 {
        let scale = 1.0 / vec.length();
        Vector4::new(vec.x * scale, vec.y * scale, vec.z * scale, vec.w * scale)
    }

    /// Dot product.
    pub fn dot(left: Vector4, right: Vector4) -> f64 {
        left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w
    }

    /// Linear blend from `a` (0) to `b` (1).
    pub fn lerp(a: Vector4, b: Vector4, blend: f64) -> Vector4 {
        Vector4::new(
            blend * (b.x - a.x) + a.x,
            blend * (b.y - a.y) + a.y,
            blend * (b.z - a.z) + a.z,
            blend * (b.w - a.w) + a.w,
        )
    }

    /// `a + u * (b - a) + v * (c - a)`.
    pub fn bary_centric(a: Vector4, b: Vector4, c: Vector4, u: f64, v: f64) -> Vector4 {
        a + u * (b - a) + v * (c - a)
    }

    /// The x and y.
    pub fn xy(&self) -> Vector2 {
        Vector2::new(self.x, self.y)
    }

    /// Sets x and y.
    pub fn set_xy(&mut self, value: Vector2) {
        self.x = value.x;
        self.y = value.y;
    }

    /// The x, y and z.
    pub fn xyz(&self) -> Vector3 {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Sets x, y and z.
    pub fn set_xyz(&mut self, value: Vector3) {
        self.x = value.x;
        self.y = value.y;
        self.z = value.z;
    }

    /// "x, y, z, w" with each component written by `format`.
    pub fn to_string_with(&self, format: impl Fn(f64) -> String) -> String {
        format!(
            "{}, {}, {}, {}",
            format(self.x),
            format(self.y),
            format(self.z),
            format(self.w)
        )
    }

    /// FNV-1a over the bytes of one double.
    pub fn get_long_hash_code_of(data: f64, hash: u64) -> u64 {
        long_hash::of(data, hash)
    }

    /// FNV-1a over `bytes`.
    pub fn compute_hash(bytes: &[u8], hash: u64) -> u64 {
        long_hash::compute_hash(bytes, hash)
    }

    /// FNV-1a over the components, starting from [`LONG_HASH_SEED`].
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// FNV-1a over the components, starting from `hash`.
    pub fn get_long_hash_code_with(&self, hash: u64) -> u64 {
        let hash = long_hash::of(self.x, hash);
        let hash = long_hash::of(self.y, hash);
        let hash = long_hash::of(self.z, hash);
        long_hash::of(self.w, hash)
    }

    /// True when every component is strictly within `error_value` of `other`'s.
    pub fn equals_within(&self, other: Vector4, error_value: f64) -> bool {
        (self.x < other.x + error_value && self.x > other.x - error_value)
            && (self.y < other.y + error_value && self.y > other.y - error_value)
            && (self.z < other.z + error_value && self.z > other.z - error_value)
            && (self.w < other.w + error_value && self.w > other.w - error_value)
    }
}

/// Indexing: 0..=3 are x, y, z, w; reading any other index gives 0, writing one is a caller
/// bug and panics.
impl Index<usize> for Vector4 {
    type Output = f64;
    fn index(&self, index: usize) -> &f64 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            3 => &self.w,
            _ => &0.0,
        }
    }
}

impl IndexMut<usize> for Vector4 {
    fn index_mut(&mut self, index: usize) -> &mut f64 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            3 => &mut self.w,
            _ => panic!("Vector4 index {index} out of range"),
        }
    }
}

impl Add for Vector4 {
    type Output = Vector4;
    fn add(self, r: Vector4) -> Vector4 {
        Vector4::new(self.x + r.x, self.y + r.y, self.z + r.z, self.w + r.w)
    }
}

impl Sub for Vector4 {
    type Output = Vector4;
    fn sub(self, r: Vector4) -> Vector4 {
        Vector4::new(self.x - r.x, self.y - r.y, self.z - r.z, self.w - r.w)
    }
}

impl Neg for Vector4 {
    type Output = Vector4;
    fn neg(self) -> Vector4 {
        Vector4::new(-self.x, -self.y, -self.z, -self.w)
    }
}

impl Mul<f64> for Vector4 {
    type Output = Vector4;
    fn mul(self, s: f64) -> Vector4 {
        Vector4::new(self.x * s, self.y * s, self.z * s, self.w * s)
    }
}

impl Mul<Vector4> for f64 {
    type Output = Vector4;
    fn mul(self, v: Vector4) -> Vector4 {
        Vector4::new(v.x * self, v.y * self, v.z * self, v.w * self)
    }
}

/// Multiplies by the reciprocal (not a true division), so `v / s` equals `v * (1 / s)`.
impl Div<f64> for Vector4 {
    type Output = Vector4;
    fn div(self, scale: f64) -> Vector4 {
        let mult = 1.0 / scale;
        Vector4::new(self.x * mult, self.y * mult, self.z * mult, self.w * mult)
    }
}

impl AddAssign for Vector4 {
    fn add_assign(&mut self, r: Vector4) {
        *self = *self + r;
    }
}

impl SubAssign for Vector4 {
    fn sub_assign(&mut self, r: Vector4) {
        *self = *self - r;
    }
}

impl MulAssign<f64> for Vector4 {
    fn mul_assign(&mut self, s: f64) {
        *self = *self * s;
    }
}

impl DivAssign<f64> for Vector4 {
    fn div_assign(&mut self, s: f64) {
        *self = *self / s;
    }
}

/// "x, y, z, w" with each component in the default double text (`format_f64`); MatterCAD's
/// vector property editor reads this text back.
impl fmt::Display for Vector4 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_string_with(format_f64))
    }
}

/// Hashes the components' bits, with -0.0 and 0.0 alike (see `float_hash`).
impl std::hash::Hash for Vector4 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        crate::float_hash::hash_f64(self.x, state);
        crate::float_hash::hash_f64(self.y, state);
        crate::float_hash::hash_f64(self.z, state);
        crate::float_hash::hash_f64(self.w, state);
    }
}
