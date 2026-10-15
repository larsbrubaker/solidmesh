//! `Vector3`: a 3D f64 vector: arithmetic, dot and cross products, absolute value, the float
//! array form, text and hashing.
//!
//! Adding, subtracting and scaling are the operators; `multiply_components`/
//! `divide_components` work per component; `normalized` has no zero check, unlike the
//! in-place `normalize`; `component_max_value`/`component_min_value` return the largest and
//! smallest component and `component_max_scalar` raises each to at least a value;
//! `get_perpendicular_with` is perpendicular to this and another vector. The matrix and
//! quaternion transforms are in `vector_transforms.rs`.

use crate::cs_math::{max_f64, min_f64};
use crate::number_format::{format_f64_max_decimals, parse_f64};
use crate::{long_hash, math_helper, Vector2, Vector3Float, Vector4, LONG_HASH_SEED};
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

/// A 3D vector of f64.
///
/// Serializes in the shape saved files hold (`{"X":..,"Y":..,"Z":..}`); members missing when
/// reading keep their default (0).
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Vector3 {
    #[serde(rename = "X")]
    pub x: f64,
    #[serde(rename = "Y")]
    pub y: f64,
    #[serde(rename = "Z")]
    pub z: f64,
}

/// The epsilon of [`Vector3::collinear`].
const COLLINEAR_EPSILON: f64 = 0.000001;

impl Vector3 {
    pub const UNIT_X: Vector3 = Vector3::new(1.0, 0.0, 0.0);
    pub const UNIT_Y: Vector3 = Vector3::new(0.0, 1.0, 0.0);
    pub const UNIT_Z: Vector3 = Vector3::new(0.0, 0.0, 1.0);
    pub const ZERO: Vector3 = Vector3::new(0.0, 0.0, 0.0);
    pub const ONE: Vector3 = Vector3::new(1.0, 1.0, 1.0);
    pub const POSITIVE_INFINITY: Vector3 =
        Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    pub const NEGATIVE_INFINITY: Vector3 =
        Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    /// `Marshal.SizeOf(new Vector3())`.
    pub const SIZE_IN_BYTES: i32 = 24;

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3 { x, y, z }
    }

    /// `v` with the given z.
    pub fn from_vector2(v: Vector2, z: f64) -> Self {
        Vector3::new(v.x, v.y, z)
    }

    /// A `Vector3Float`, widened.
    pub fn from_vector3_float(v: Vector3Float) -> Self {
        Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z))
    }

    /// The first three values; panics on fewer (a caller bug).
    pub fn from_slice(double_array: &[f64]) -> Self {
        Vector3::new(double_array[0], double_array[1], double_array[2])
    }

    /// The x, y and z of a `Vector4`.
    pub fn from_vector4(v: Vector4) -> Self {
        Vector3::new(v.x, v.y, v.z)
    }

    /// Sets all three components.
    pub fn set(&mut self, x: f64, y: f64, z: f64) {
        self.x = x;
        self.y = y;
        self.z = z;
    }

    /// The length of the vector.
    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// The distance from this point to the segment `start`-`end`.
    pub fn distance_to_segment(&self, start: Vector3, end: Vector3) -> f64 {
        let segment_delta = end - start;
        let segment_length = segment_delta.length();
        if segment_length == 0.0 {
            return (*self - start).length();
        }

        let direction = segment_delta / segment_length;
        let along_segment = math_helper::clamp((*self - start).dot(direction), 0.0, segment_length);
        (*self - (start + direction * along_segment)).length()
    }

    /// The squared length.
    pub fn length_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// A unit-length copy (the zero vector stays zero).
    pub fn get_normal(&self) -> Vector3 {
        let mut temp = *self;
        temp.normalize();
        temp
    }

    /// Scales this vector to unit length; the zero vector is left unchanged.
    pub fn normalize(&mut self) {
        let length = self.length();
        if length != 0.0 {
            let scale = 1.0 / self.length();
            self.x *= scale;
            self.y *= scale;
            self.z *= scale;
        }
    }

    /// False if any component is NaN or infinite.
    pub fn is_valid(&self) -> bool {
        !(self.x.is_nan()
            || self.x.is_infinite()
            || self.y.is_nan()
            || self.y.is_infinite()
            || self.z.is_nan()
            || self.z.is_infinite())
    }

    /// `[x, y, z]`.
    pub fn to_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Component-wise product.
    pub fn multiply_components(vector: Vector3, scale: Vector3) -> Vector3 {
        Vector3::new(vector.x * scale.x, vector.y * scale.y, vector.z * scale.z)
    }

    /// Component-wise quotient.
    pub fn divide_components(vector: Vector3, scale: Vector3) -> Vector3 {
        Vector3::new(vector.x / scale.x, vector.y / scale.y, vector.z / scale.z)
    }

    /// Component-wise minimum.
    pub fn component_min(a: Vector3, b: Vector3) -> Vector3 {
        Vector3::new(
            if a.x < b.x { a.x } else { b.x },
            if a.y < b.y { a.y } else { b.y },
            if a.z < b.z { a.z } else { b.z },
        )
    }

    /// Parses "x, y, z" (culture-invariant text). Unparsable parts are 0; missing parts stay 0.
    pub fn parse(s: &str) -> Vector3 {
        let mut result = Vector3::ZERO;
        let values: Vec<f64> = s.split(',').map(parse_f64).collect();
        for (i, value) in values.iter().take(3).enumerate() {
            result[i] = *value;
        }
        result
    }

    /// Component-wise maximum.
    pub fn component_max(a: Vector3, b: Vector3) -> Vector3 {
        Vector3::new(
            if a.x > b.x { a.x } else { b.x },
            if a.y > b.y { a.y } else { b.y },
            if a.z > b.z { a.z } else { b.z },
        )
    }

    /// Each component raised to at least `b`.
    pub fn component_max_scalar(a: Vector3, b: f64) -> Vector3 {
        Vector3::new(
            if a.x > b { a.x } else { b },
            if a.y > b { a.y } else { b },
            if a.z > b { a.z } else { b },
        )
    }

    /// The shorter vector (`right` on a tie).
    pub fn min(left: Vector3, right: Vector3) -> Vector3 {
        if left.length_squared() < right.length_squared() {
            left
        } else {
            right
        }
    }

    /// The longer vector (`left` on a tie).
    pub fn max(left: Vector3, right: Vector3) -> Vector3 {
        if left.length_squared() >= right.length_squared() {
            left
        } else {
            right
        }
    }

    /// Each component limited to `[min, max]`.
    pub fn clamp(vec: Vector3, min: Vector3, max: Vector3) -> Vector3 {
        Vector3::new(
            clamp_component(vec.x, min.x, max.x),
            clamp_component(vec.y, min.y, max.y),
            clamp_component(vec.z, min.z, max.z),
        )
    }

    /// A unit-length copy of `vec` (NaN for the zero vector).
    pub fn normalized(vec: Vector3) -> Vector3 {
        let scale = 1.0 / vec.length();
        Vector3::new(vec.x * scale, vec.y * scale, vec.z * scale)
    }

    /// True if `a`, `b` and `c` lie on one line (epsilon 0.000001).
    pub fn collinear(a: Vector3, b: Vector3, c: Vector3) -> bool {
        Self::collinear_with_epsilon(a, b, c, COLLINEAR_EPSILON)
    }

    /// True if `a`, `b` and `c` lie on one line, within `epsilon`.
    pub fn collinear_with_epsilon(a: Vector3, b: Vector3, c: Vector3, epsilon: f64) -> bool {
        // Return true if a, b, and c all lie on the same line.
        (b - a).cross(c - a).length().abs() < epsilon
    }

    /// Component-wise absolute value.
    pub fn abs(self) -> Vector3 {
        Vector3::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    /// A vector perpendicular to this one; the zero vector has none and gives zero.
    pub fn get_perpendicular(&self) -> Vector3 {
        if self.x != 0.0 {
            Vector3::new(-(self.y + self.z) / self.x, 1.0, 1.0)
        } else if self.y != 0.0 {
            Vector3::new(1.0, -(self.x + self.z) / self.y, 1.0)
        } else if self.z != 0.0 {
            Vector3::new(1.0, 1.0, -(self.x + self.y) / self.z)
        } else {
            // the input vector has no length (no vector is perpendicular to it)
            Vector3::default()
        }
    }

    /// A vector perpendicular to both this and `b`; when the two are collinear with the
    /// origin, the cross product is taken about a far-away point instead.
    pub fn get_perpendicular_with(self, b: Vector3) -> Vector3 {
        let a = self;
        if !Self::collinear(a, b, Vector3::ZERO) {
            a.cross(b)
        } else {
            let z_one = Vector3::new(0.0, 0.0, 100000.0);
            if !Self::collinear(a, b, z_one) {
                (a - z_one).cross(b - z_one)
            } else {
                let x_one = Vector3::new(1000000.0, 0.0, 0.0);
                (a - x_one).cross(b - x_one)
            }
        }
    }

    /// Linear blend from `a` (0) to `b` (1); exactly `a` at 0 and exactly `b` at 1.
    pub fn lerp(a: Vector3, b: Vector3, blend: f64) -> Vector3 {
        if blend == 0.0 {
            return a;
        }

        if blend == 1.0 {
            return b;
        }

        Vector3::new(
            blend * (b.x - a.x) + a.x,
            blend * (b.y - a.y) + a.y,
            blend * (b.z - a.z) + a.z,
        )
    }

    /// `a + u * (b - a) + v * (c - a)`.
    pub fn bary_centric(a: Vector3, b: Vector3, c: Vector3, u: f64, v: f64) -> Vector3 {
        a + u * (b - a) + v * (c - a)
    }

    /// The angle between two vectors, in radians.
    pub fn calculate_angle(first: Vector3, second: Vector3) -> f64 {
        (first.dot(second) / (first.length() * second.length())).acos()
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

    /// FNV-1a over the components, starting from [`LONG_HASH_SEED`].
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// FNV-1a over the components, starting from `hash`.
    pub fn get_long_hash_code_with(&self, hash: u64) -> u64 {
        let hash = long_hash::of(self.x, hash);
        let hash = long_hash::of(self.y, hash);
        long_hash::of(self.z, hash)
    }

    /// True when every component is strictly within `error_value` of `other`'s.
    pub fn equals_within(&self, other: Vector3, error_value: f64) -> bool {
        (self.x < other.x + error_value && self.x > other.x - error_value)
            && (self.y < other.y + error_value && self.y > other.y - error_value)
            && (self.z < other.z + error_value && self.z > other.z - error_value)
    }

    /// The largest component (NaN wins, see `cs_math`).
    pub fn component_max_value(vector3: Vector3) -> f64 {
        max_f64(vector3.x, max_f64(vector3.y, vector3.z))
    }

    /// The smallest component (NaN wins, see `cs_math`).
    pub fn component_min_value(vector3: Vector3) -> f64 {
        min_f64(vector3.x, min_f64(vector3.y, vector3.z))
    }

    /// Dot product (`Vector3Ex.Dot`).
    pub fn dot(&self, right: Vector3) -> f64 {
        self.x * right.x + self.y * right.y + self.z * right.z
    }

    /// Cross product (`Vector3Ex.Cross`).
    pub fn cross(&self, right: Vector3) -> Vector3 {
        Vector3::new(
            self.y * right.z - self.z * right.y,
            self.z * right.x - self.x * right.z,
            self.x * right.y - self.y * right.x,
        )
    }
}

fn clamp_component(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// `[x0, y0, z0, x1, ...]` narrowed to f32 (`Vector3Ex.ToFloatArray(List<Vector3>)`).
pub fn to_float_array(list: &[Vector3]) -> Vec<f32> {
    let mut array = Vec::with_capacity(list.len() * 3);
    for v in list {
        array.push(v.x as f32);
        array.push(v.y as f32);
        array.push(v.z as f32);
    }
    array
}

/// Indexing: 0..=2 are x, y, z; reading any other index gives 0, writing one is a caller
/// bug and panics.
impl Index<usize> for Vector3 {
    type Output = f64;
    fn index(&self, index: usize) -> &f64 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => &0.0,
        }
    }
}

impl IndexMut<usize> for Vector3 {
    fn index_mut(&mut self, index: usize) -> &mut f64 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Vector3 index {index} out of range"),
        }
    }
}

impl Add for Vector3 {
    type Output = Vector3;
    fn add(self, right: Vector3) -> Vector3 {
        Vector3::new(self.x + right.x, self.y + right.y, self.z + right.z)
    }
}

impl Sub for Vector3 {
    type Output = Vector3;
    fn sub(self, right: Vector3) -> Vector3 {
        Vector3::new(self.x - right.x, self.y - right.y, self.z - right.z)
    }
}

impl Neg for Vector3 {
    type Output = Vector3;
    fn neg(self) -> Vector3 {
        Vector3::new(-self.x, -self.y, -self.z)
    }
}

/// Component-wise product.
impl Mul for Vector3 {
    type Output = Vector3;
    fn mul(self, vec_b: Vector3) -> Vector3 {
        Vector3::new(self.x * vec_b.x, self.y * vec_b.y, self.z * vec_b.z)
    }
}

impl Mul<f64> for Vector3 {
    type Output = Vector3;
    fn mul(self, scale: f64) -> Vector3 {
        Vector3::new(self.x * scale, self.y * scale, self.z * scale)
    }
}

impl Mul<Vector3> for f64 {
    type Output = Vector3;
    fn mul(self, vec: Vector3) -> Vector3 {
        Vector3::new(vec.x * self, vec.y * self, vec.z * self)
    }
}

impl Div<Vector3> for f64 {
    type Output = Vector3;
    fn div(self, vec: Vector3) -> Vector3 {
        Vector3::new(self / vec.x, self / vec.y, self / vec.z)
    }
}

/// Multiplies by the reciprocal (not a true division), so `v / s` equals `v * (1 / s)`.
impl Div<f64> for Vector3 {
    type Output = Vector3;
    fn div(self, scale: f64) -> Vector3 {
        let mult = 1.0 / scale;
        Vector3::new(self.x * mult, self.y * mult, self.z * mult)
    }
}

impl AddAssign for Vector3 {
    fn add_assign(&mut self, right: Vector3) {
        *self = *self + right;
    }
}

impl SubAssign for Vector3 {
    fn sub_assign(&mut self, right: Vector3) {
        *self = *self - right;
    }
}

impl MulAssign<f64> for Vector3 {
    fn mul_assign(&mut self, scale: f64) {
        *self = *self * scale;
    }
}

impl MulAssign<Vector3> for Vector3 {
    fn mul_assign(&mut self, vec_b: Vector3) {
        *self = *self * vec_b;
    }
}

impl DivAssign<f64> for Vector3 {
    fn div_assign(&mut self, scale: f64) {
        *self = *self / scale;
    }
}

/// "[x, y, z]" with each component in the `0.####` format, the form saved text holds.
impl fmt::Display for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}, {}, {}]",
            format_f64_max_decimals(self.x, 4),
            format_f64_max_decimals(self.y, 4),
            format_f64_max_decimals(self.z, 4)
        )
    }
}

/// Hashes the components' bits, with -0.0 and 0.0 alike (see `float_hash`).
impl std::hash::Hash for Vector3 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        crate::float_hash::hash_f64(self.x, state);
        crate::float_hash::hash_f64(self.y, state);
        crate::float_hash::hash_f64(self.z, state);
    }
}
