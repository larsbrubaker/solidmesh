//! `Vector3Float`: a 3D f32 vector (mesh vertices and normals). Ports agg-sharp
//! `VectorMath/Vector3Float.cs`: the struct and the non-matrix parts of `Vector3FloatEx`.
//!
//! Precision follows the C#: float expressions are computed in f32; `Math.Sqrt`/`Math.Acos`
//! widen to f64 and narrow the result.
//!
//! Mapping notes: static `Add`/`Subtract`/`Multiply(v, float)`/`Divide(v, float)` give the same
//! results as the operators and are the operators; the vector-scale overloads are
//! `multiply_components`/`divide_components`; `ComponentMin()`/`ComponentMax()` returning a
//! float are `component_min_value`/`component_max_value`; static `Normalize(v)` is
//! `normalized`. Not ported: the `Xy` setter and the Quaternion/perspective/position
//! transforms, which throw `NotImplementedException` in C#; `Transform(Matrix4X4)`,
//! `TransformVector` and `TransformNormal*` are in `vector_transforms.rs`.

use crate::cs_math::{max_f32, min_f32};
use crate::number_format::format_f32;
use crate::{long_hash, Vector2, Vector3, Vector4, LONG_HASH_SEED};
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

/// A 3D vector of f32 (C# `MatterHackers.VectorMath.Vector3Float`).
///
/// Serializes as Newtonsoft does (`{"X":..,"Y":..,"Z":..}`); members missing when reading keep
/// their default (0).
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Vector3Float {
    #[serde(rename = "X")]
    pub x: f32,
    #[serde(rename = "Y")]
    pub y: f32,
    #[serde(rename = "Z")]
    pub z: f32,
}

/// The default epsilon of C# `Vector3FloatEx.Collinear`.
const COLLINEAR_EPSILON: f32 = 0.000001;

impl Vector3Float {
    pub const UNIT_X: Vector3Float = Vector3Float::new_f32(1.0, 0.0, 0.0);
    pub const UNIT_Y: Vector3Float = Vector3Float::new_f32(0.0, 1.0, 0.0);
    pub const UNIT_Z: Vector3Float = Vector3Float::new_f32(0.0, 0.0, 1.0);
    pub const ZERO: Vector3Float = Vector3Float::new_f32(0.0, 0.0, 0.0);
    pub const ONE: Vector3Float = Vector3Float::new_f32(1.0, 1.0, 1.0);
    pub const POSITIVE_INFINITY: Vector3Float =
        Vector3Float::new_f32(f32::INFINITY, f32::INFINITY, f32::INFINITY);
    pub const NEGATIVE_INFINITY: Vector3Float =
        Vector3Float::new_f32(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    /// `Marshal.SizeOf(new Vector3Float())`.
    pub const SIZE_IN_BYTES: i32 = 12;

    /// Narrows each value to f32 (C# `Vector3Float(double, double, double)`).
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self::new_f32(x as f32, y as f32, z as f32)
    }

    /// C# `Vector3Float(float, float, float)`.
    pub const fn new_f32(x: f32, y: f32, z: f32) -> Self {
        Vector3Float { x, y, z }
    }

    /// `v` with the given z, narrowed (C# `Vector3Float(Vector2 v, double z = 0)`).
    pub fn from_vector2(v: Vector2, z: f64) -> Self {
        Self::new(v.x, v.y, z)
    }

    /// The first three values (C# `Vector3Float(float[])`; panics on fewer, where C# throws).
    pub fn from_slice(float_array: &[f32]) -> Self {
        Self::new_f32(float_array[0], float_array[1], float_array[2])
    }

    /// The x, y and z of a `Vector4`, narrowed.
    pub fn from_vector4(v: Vector4) -> Self {
        Self::new(v.x, v.y, v.z)
    }

    /// A `Vector3`, narrowed.
    pub fn from_vector3(position: Vector3) -> Self {
        Self::new(position.x, position.y, position.z)
    }

    /// This vector widened to a `Vector3`.
    pub fn as_vector3(&self) -> Vector3 {
        Vector3::from_vector3_float(*self)
    }

    /// The length (squared length summed in f32, square root in f64, narrowed).
    pub fn length(&self) -> f32 {
        f64::from(self.x * self.x + self.y * self.y + self.z * self.z).sqrt() as f32
    }

    /// The squared length, in f32.
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// A unit-length copy (NaN for the zero vector, as in C#).
    pub fn get_normal(&self) -> Vector3Float {
        let mut temp = *self;
        temp.normalize();
        temp
    }

    /// Scales this vector to unit length (NaN for the zero vector, as in C#).
    pub fn normalize(&mut self) {
        let scale = 1.0f32 / self.length();
        self.x *= scale;
        self.y *= scale;
        self.z *= scale;
    }

    /// `[x, y, z]`.
    pub fn to_array(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    /// The x and y, widened (C# property `Xy` getter).
    pub fn xy(&self) -> Vector2 {
        Vector2::new(f64::from(self.x), f64::from(self.y))
    }

    /// FNV-1a over the components (each widened to double, as C# passes them), starting from
    /// [`LONG_HASH_SEED`].
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// FNV-1a over the components (each widened to double), starting from `hash`.
    pub fn get_long_hash_code_with(&self, hash: u64) -> u64 {
        let hash = long_hash::of(f64::from(self.x), hash);
        let hash = long_hash::of(f64::from(self.y), hash);
        long_hash::of(f64::from(self.z), hash)
    }

    /// True when every component is strictly within `error_value` of `other`'s, compared in
    /// double (C# `Equals(Vector3Float, double)`).
    pub fn equals_within(&self, other: Vector3Float, error_value: f64) -> bool {
        let within = |a: f32, b: f32| {
            let (a, b) = (f64::from(a), f64::from(b));
            a < b + error_value && a > b - error_value
        };
        within(self.x, other.x) && within(self.y, other.y) && within(self.z, other.z)
    }

    /// Half the parallelogram area of `self`, `b`, `c`: the triangle's area (`GetArea`).
    pub fn get_area(self, b: Vector3Float, c: Vector3Float) -> f32 {
        (b - self).cross(c - self).length() / 2.0
    }

    /// Component-wise product.
    pub fn multiply_components(vector: Vector3Float, scale: Vector3Float) -> Vector3Float {
        Self::new_f32(vector.x * scale.x, vector.y * scale.y, vector.z * scale.z)
    }

    /// Component-wise quotient.
    pub fn divide_components(vector: Vector3Float, scale: Vector3Float) -> Vector3Float {
        Self::new_f32(vector.x / scale.x, vector.y / scale.y, vector.z / scale.z)
    }

    /// The smallest component (C# `Math.Min` semantics: NaN wins).
    pub fn component_min_value(self) -> f32 {
        min_f32(self.x, min_f32(self.y, self.z))
    }

    /// Component-wise minimum.
    pub fn component_min(self, b: Vector3Float) -> Vector3Float {
        Self::new_f32(
            if self.x < b.x { self.x } else { b.x },
            if self.y < b.y { self.y } else { b.y },
            if self.z < b.z { self.z } else { b.z },
        )
    }

    /// The largest component (C# `Math.Max` semantics: NaN wins).
    pub fn component_max_value(self) -> f32 {
        max_f32(self.x, max_f32(self.y, self.z))
    }

    /// Component-wise maximum.
    pub fn component_max(self, b: Vector3Float) -> Vector3Float {
        Self::new_f32(
            if self.x > b.x { self.x } else { b.x },
            if self.y > b.y { self.y } else { b.y },
            if self.z > b.z { self.z } else { b.z },
        )
    }

    /// The shorter vector (`right` on a tie).
    pub fn min(left: Vector3Float, right: Vector3Float) -> Vector3Float {
        if left.length_squared() < right.length_squared() {
            left
        } else {
            right
        }
    }

    /// The longer vector (`left` on a tie).
    pub fn max(left: Vector3Float, right: Vector3Float) -> Vector3Float {
        if left.length_squared() >= right.length_squared() {
            left
        } else {
            right
        }
    }

    /// Each component limited to `[min, max]`.
    pub fn clamp(vec: Vector3Float, min: Vector3Float, max: Vector3Float) -> Vector3Float {
        let pick = |v: f32, lo: f32, hi: f32| {
            if v < lo {
                lo
            } else if v > hi {
                hi
            } else {
                v
            }
        };
        Self::new_f32(
            pick(vec.x, min.x, max.x),
            pick(vec.y, min.y, max.y),
            pick(vec.z, min.z, max.z),
        )
    }

    /// A unit-length copy of `vec` (C# static `Normalize`).
    pub fn normalized(vec: Vector3Float) -> Vector3Float {
        let scale = 1.0f32 / vec.length();
        Self::new_f32(vec.x * scale, vec.y * scale, vec.z * scale)
    }

    /// Dot product, in f32.
    pub fn dot(&self, right: Vector3Float) -> f32 {
        self.x * right.x + self.y * right.y + self.z * right.z
    }

    /// Cross product, in f32.
    pub fn cross(&self, right: Vector3Float) -> Vector3Float {
        Self::new_f32(
            self.y * right.z - self.z * right.y,
            self.z * right.x - self.x * right.z,
            self.x * right.y - self.y * right.x,
        )
    }

    /// True if `a`, `b` and `c` lie on one line (C#'s default epsilon, 0.000001f).
    pub fn collinear(a: Vector3Float, b: Vector3Float, c: Vector3Float) -> bool {
        Self::collinear_with_epsilon(a, b, c, COLLINEAR_EPSILON)
    }

    /// True if `a`, `b` and `c` lie on one line, within `epsilon`.
    pub fn collinear_with_epsilon(
        a: Vector3Float,
        b: Vector3Float,
        c: Vector3Float,
        epsilon: f32,
    ) -> bool {
        // Return true if a, b, and c all lie on the same line.
        (b - a).cross(c - a).length().abs() < epsilon
    }

    /// Linear blend from `a` (0) to `b` (1), in f32.
    pub fn lerp(a: Vector3Float, b: Vector3Float, blend: f32) -> Vector3Float {
        Self::new_f32(
            blend * (b.x - a.x) + a.x,
            blend * (b.y - a.y) + a.y,
            blend * (b.z - a.z) + a.z,
        )
    }

    /// `a + u * (b - a) + v * (c - a)`, in f32.
    pub fn bary_centric(
        a: Vector3Float,
        b: Vector3Float,
        c: Vector3Float,
        u: f32,
        v: f32,
    ) -> Vector3Float {
        a + u * (b - a) + v * (c - a)
    }

    /// The angle between two vectors, in radians (ratio in f32, `acos` in f64, narrowed).
    pub fn calculate_angle(self, second: Vector3Float) -> f32 {
        f64::from(self.dot(second) / (self.length() * second.length())).acos() as f32
    }
}

/// `[x0, y0, z0, x1, ...]` (`Vector3FloatEx.ToFloatArray(List<Vector3Float>)`).
pub fn to_float_array_f32(list: &[Vector3Float]) -> Vec<f32> {
    let mut array = Vec::with_capacity(list.len() * 3);
    for v in list {
        array.push(v.x);
        array.push(v.y);
        array.push(v.z);
    }
    array
}

/// C# indexer: 0..=2 are x, y, z; reading any other index gives 0, writing one panics (the C#
/// setter throws).
impl Index<usize> for Vector3Float {
    type Output = f32;
    fn index(&self, index: usize) -> &f32 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => &0.0,
        }
    }
}

impl IndexMut<usize> for Vector3Float {
    fn index_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Vector3Float index {index} out of range"),
        }
    }
}

impl Add for Vector3Float {
    type Output = Vector3Float;
    fn add(self, r: Vector3Float) -> Vector3Float {
        Self::new_f32(self.x + r.x, self.y + r.y, self.z + r.z)
    }
}

impl Sub for Vector3Float {
    type Output = Vector3Float;
    fn sub(self, r: Vector3Float) -> Vector3Float {
        Self::new_f32(self.x - r.x, self.y - r.y, self.z - r.z)
    }
}

impl Neg for Vector3Float {
    type Output = Vector3Float;
    fn neg(self) -> Vector3Float {
        Self::new_f32(-self.x, -self.y, -self.z)
    }
}

/// Component-wise product (C# `operator *(Vector3Float, Vector3Float)`).
impl Mul for Vector3Float {
    type Output = Vector3Float;
    fn mul(self, b: Vector3Float) -> Vector3Float {
        Self::new_f32(self.x * b.x, self.y * b.y, self.z * b.z)
    }
}

/// The scale is narrowed to f32 first, then multiplied in f32.
impl Mul<f64> for Vector3Float {
    type Output = Vector3Float;
    fn mul(self, scale: f64) -> Vector3Float {
        let scale = scale as f32;
        Self::new_f32(self.x * scale, self.y * scale, self.z * scale)
    }
}

impl Mul<Vector3Float> for f32 {
    type Output = Vector3Float;
    fn mul(self, vec: Vector3Float) -> Vector3Float {
        Vector3Float::new_f32(vec.x * self, vec.y * self, vec.z * self)
    }
}

impl Div<Vector3Float> for f32 {
    type Output = Vector3Float;
    fn div(self, vec: Vector3Float) -> Vector3Float {
        Vector3Float::new_f32(self / vec.x, self / vec.y, self / vec.z)
    }
}

/// Multiplies by the f32 reciprocal of the narrowed scale, as the C# operator does.
impl Div<f64> for Vector3Float {
    type Output = Vector3Float;
    fn div(self, scale: f64) -> Vector3Float {
        let mult = 1.0f32 / scale as f32;
        Self::new_f32(self.x * mult, self.y * mult, self.z * mult)
    }
}

impl AddAssign for Vector3Float {
    fn add_assign(&mut self, r: Vector3Float) {
        *self = *self + r;
    }
}

impl SubAssign for Vector3Float {
    fn sub_assign(&mut self, r: Vector3Float) {
        *self = *self - r;
    }
}

impl MulAssign<f64> for Vector3Float {
    fn mul_assign(&mut self, scale: f64) {
        *self = *self * scale;
    }
}

impl DivAssign<f64> for Vector3Float {
    fn div_assign(&mut self, scale: f64) {
        *self = *self / scale;
    }
}

/// "[x, y, z]" with each component as C# `float.ToString()`. The C# interpolates with the
/// current culture; this always writes InvariantCulture text, which differs only under a
/// culture whose number symbols differ.
impl fmt::Display for Vector3Float {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}, {}, {}]",
            format_f32(self.x),
            format_f32(self.y),
            format_f32(self.z)
        )
    }
}

/// C# `GetHashCode`: the components' bits, with -0.0 and 0.0 alike (see `float_hash`).
impl std::hash::Hash for Vector3Float {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        crate::float_hash::hash_f32(self.x, state);
        crate::float_hash::hash_f32(self.y, state);
        crate::float_hash::hash_f32(self.z, state);
    }
}
