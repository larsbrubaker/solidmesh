//! `Vector2Float`: a 2D f32 vector.
//!
//! Precision is fixed per operation: pure float arithmetic is computed in f32, and where a
//! double enters (`multiply`/`divide` by a double, `atan2`, `sqrt`) the f32 values are widened
//! first and the result narrowed after, so results are the same on every platform.
//!
//! Adding, subtracting and the f32 scaling are the operators; `multiply`/`divide` scale by a
//! double, computing in double; `multiply_components`/`divide_components` work per component;
//! `rotated` and `normalized` return new values; `bary_centric_f64` takes double weights and
//! computes in double. The quaternion transform is in `vector_transforms.rs`.

use crate::math_helper::{self, MathError};
use crate::number_format::format_f32;
use crate::{Vector2, Vector3, Vector3Float};
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

/// A 2D vector of f32.
///
/// Serializes in the shape saved files hold (`{"X":..,"Y":..}`); members missing when
/// reading keep their default (0).
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Vector2Float {
    #[serde(rename = "X")]
    pub x: f32,
    #[serde(rename = "Y")]
    pub y: f32,
}

impl Vector2Float {
    pub const UNIT_X: Vector2Float = Vector2Float { x: 1.0, y: 0.0 };
    pub const UNIT_Y: Vector2Float = Vector2Float { x: 0.0, y: 1.0 };
    pub const ZERO: Vector2Float = Vector2Float { x: 0.0, y: 0.0 };
    pub const ONE: Vector2Float = Vector2Float { x: 1.0, y: 1.0 };
    /// `Marshal.SizeOf(new Vector2Float())`.
    pub const SIZE_IN_BYTES: i32 = 8;

    /// Narrows both values to f32 (the only constructor taking components).
    pub fn new(x: f64, y: f64) -> Self {
        Vector2Float {
            x: x as f32,
            y: y as f32,
        }
    }

    /// The x and y of a `Vector3`, narrowed.
    pub fn from_vector3(vector: Vector3) -> Self {
        Self::new(vector.x, vector.y)
    }

    /// A `Vector2`, narrowed.
    pub fn from_vector2(vector: Vector2) -> Self {
        Self::new(vector.x, vector.y)
    }

    /// The x and y of a `Vector3Float`.
    pub fn from_vector3_float(vector: Vector3Float) -> Self {
        Vector2Float {
            x: vector.x,
            y: vector.y,
        }
    }

    /// The angle swept from `start_position` to `end_position` around this point, each angle
    /// measured in `[0, Tau)`.
    pub fn get_delta_angle(&self, start_position: Vector2Float, end_position: Vector2Float) -> f64 {
        let start_position = start_position - *self;
        let mut start_angle = libm::atan2(f64::from(start_position.y), f64::from(start_position.x));
        start_angle = if start_angle < 0.0 {
            start_angle + math_helper::TAU
        } else {
            start_angle
        };

        let end_position = end_position - *self;
        let mut end_angle = libm::atan2(f64::from(end_position.y), f64::from(end_position.x));
        end_angle = if end_angle < 0.0 {
            end_angle + math_helper::TAU
        } else {
            end_angle
        };

        end_angle - start_angle
    }

    /// The length (squared length summed in f32, square root in f64, narrowed).
    pub fn length(&self) -> f32 {
        f64::from(self.x * self.x + self.y * self.y).sqrt() as f32
    }

    /// The squared length, summed in f32 and widened.
    pub fn length_squared(&self) -> f64 {
        f64::from(self.x * self.x + self.y * self.y)
    }

    /// Rotates this vector in place by `radians`.
    pub fn rotate(&mut self, radians: f64) {
        *self = Self::rotated(*self, radians);
    }

    /// `atan2(y, x)`.
    pub fn get_angle(&self) -> f64 {
        libm::atan2(f64::from(self.y), f64::from(self.x))
    }

    /// `get_angle` brought into `[0, Tau]`.
    pub fn get_angle_0_to_2pi(&self) -> Result<f64, MathError> {
        math_helper::range_0_to_tau(self.get_angle())
    }

    /// The perpendicular to the right.
    pub fn get_perpendicular_right(&self) -> Vector2Float {
        Vector2Float {
            x: self.y,
            y: -self.x,
        }
    }

    /// The perpendicular to the left.
    pub fn get_perpendicular_left(&self) -> Vector2Float {
        Vector2Float {
            x: -self.y,
            y: self.x,
        }
    }

    /// A unit-length copy (NaN for the zero vector).
    pub fn get_normal(&self) -> Vector2Float {
        let mut temp = *self;
        temp.normalize();
        temp
    }

    /// Scales this vector to unit length (NaN for the zero vector).
    pub fn normalize(&mut self) {
        let scale = 1.0f32 / self.length();
        self.x *= scale;
        self.y *= scale;
    }

    /// Each component times a double, computed in double and narrowed.
    pub fn multiply(vector: Vector2Float, scale: f64) -> Vector2Float {
        Self::new(f64::from(vector.x) * scale, f64::from(vector.y) * scale)
    }

    /// Divides by a double: `multiply` by `1 / scale`.
    pub fn divide(vector: Vector2Float, scale: f64) -> Vector2Float {
        Self::multiply(vector, 1.0 / scale)
    }

    /// Component-wise product.
    pub fn multiply_components(vector: Vector2Float, scale: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: vector.x * scale.x,
            y: vector.y * scale.y,
        }
    }

    /// Component-wise quotient.
    pub fn divide_components(vector: Vector2Float, scale: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: vector.x / scale.x,
            y: vector.y / scale.y,
        }
    }

    /// Component-wise minimum.
    pub fn min(a: Vector2Float, b: Vector2Float) -> Vector2Float {
        Self::component_min(a, b)
    }

    /// Component-wise maximum.
    pub fn max(a: Vector2Float, b: Vector2Float) -> Vector2Float {
        Self::component_max(a, b)
    }

    /// Each component limited to `[min, max]`.
    pub fn clamp(vec: Vector2Float, min: Vector2Float, max: Vector2Float) -> Vector2Float {
        let pick = |v: f32, lo: f32, hi: f32| {
            if v < lo {
                lo
            } else if v > hi {
                hi
            } else {
                v
            }
        };
        Vector2Float {
            x: pick(vec.x, min.x, max.x),
            y: pick(vec.y, min.y, max.y),
        }
    }

    /// A unit-length copy of `vec`.
    pub fn normalized(vec: Vector2Float) -> Vector2Float {
        let scale = 1.0f32 / vec.length();
        Vector2Float {
            x: vec.x * scale,
            y: vec.y * scale,
        }
    }

    /// Dot product, computed in f32 and widened.
    pub fn dot(left: Vector2Float, right: Vector2Float) -> f64 {
        f64::from(left.x * right.x + left.y * right.y)
    }

    /// The z of the 3D cross product, computed in f32 and widened.
    pub fn cross(left: Vector2Float, right: Vector2Float) -> f64 {
        f64::from(left.x * right.y - left.y * right.x)
    }

    /// `to_rotate` rotated by `radians` (cos and sin narrowed to f32 first).
    pub fn rotated(to_rotate: Vector2Float, radians: f64) -> Vector2Float {
        let cos = libm::cos(radians) as f32;
        let sin = libm::sin(radians) as f32;
        Vector2Float {
            x: to_rotate.x * cos - to_rotate.y * sin,
            y: to_rotate.y * cos + to_rotate.x * sin,
        }
    }

    /// Linear blend from `a` (0) to `b` (1), in f32.
    pub fn lerp(a: Vector2Float, b: Vector2Float, blend: f32) -> Vector2Float {
        Vector2Float {
            x: blend * (b.x - a.x) + a.x,
            y: blend * (b.y - a.y) + a.y,
        }
    }

    /// `a + u * (b - a) + v * (c - a)`, in f32.
    pub fn bary_centric(
        a: Vector2Float,
        b: Vector2Float,
        c: Vector2Float,
        u: f32,
        v: f32,
    ) -> Vector2Float {
        a + u * (b - a) + v * (c - a)
    }

    /// The barycentric blend with double weights: each scaled difference computed in double and
    /// narrowed before it is added.
    pub fn bary_centric_f64(
        a: Vector2Float,
        b: Vector2Float,
        c: Vector2Float,
        u: f64,
        v: f64,
    ) -> Vector2Float {
        let mut result = a;
        let temp = Self::multiply(b - a, u);
        result += temp;
        let temp = Self::multiply(c - a, v);
        result + temp
    }

    /// Component-wise minimum.
    pub fn component_min(a: Vector2Float, b: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: if a.x < b.x { a.x } else { b.x },
            y: if a.y < b.y { a.y } else { b.y },
        }
    }

    /// Component-wise maximum.
    pub fn component_max(a: Vector2Float, b: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: if a.x > b.x { a.x } else { b.x },
            y: if a.y > b.y { a.y } else { b.y },
        }
    }

    /// True when every component is strictly within `error_range` of `other`'s, compared in
    /// double.
    pub fn equals_within(&self, other: Vector2Float, error_range: f64) -> bool {
        let (x, y) = (f64::from(self.x), f64::from(self.y));
        let (ox, oy) = (f64::from(other.x), f64::from(other.y));
        (x < ox + error_range && x > ox - error_range)
            && (y < oy + error_range && y > oy - error_range)
    }
}

/// Indexing: 0 is x, 1 is y; reading any other index gives 0, writing one is a caller
/// bug and panics.
impl Index<usize> for Vector2Float {
    type Output = f32;
    fn index(&self, index: usize) -> &f32 {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => &0.0,
        }
    }
}

impl IndexMut<usize> for Vector2Float {
    fn index_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Vector2Float index {index} out of range"),
        }
    }
}

impl Add for Vector2Float {
    type Output = Vector2Float;
    fn add(self, r: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: self.x + r.x,
            y: self.y + r.y,
        }
    }
}

impl Sub for Vector2Float {
    type Output = Vector2Float;
    fn sub(self, r: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: self.x - r.x,
            y: self.y - r.y,
        }
    }
}

impl Neg for Vector2Float {
    type Output = Vector2Float;
    fn neg(self) -> Vector2Float {
        Vector2Float {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl Mul<f32> for Vector2Float {
    type Output = Vector2Float;
    fn mul(self, f: f32) -> Vector2Float {
        Vector2Float {
            x: self.x * f,
            y: self.y * f,
        }
    }
}

impl Mul<Vector2Float> for f32 {
    type Output = Vector2Float;
    fn mul(self, vec: Vector2Float) -> Vector2Float {
        vec * self
    }
}

/// Multiplies by the f32 reciprocal (not a true division), so `v / s` equals `v * (1 / s)`.
impl Div<f32> for Vector2Float {
    type Output = Vector2Float;
    fn div(self, f: f32) -> Vector2Float {
        let mult = 1.0f32 / f;
        Vector2Float {
            x: self.x * mult,
            y: self.y * mult,
        }
    }
}

impl Div<Vector2Float> for f32 {
    type Output = Vector2Float;
    fn div(self, vec: Vector2Float) -> Vector2Float {
        Vector2Float {
            x: self / vec.x,
            y: self / vec.y,
        }
    }
}

impl AddAssign for Vector2Float {
    fn add_assign(&mut self, r: Vector2Float) {
        *self = *self + r;
    }
}

impl SubAssign for Vector2Float {
    fn sub_assign(&mut self, r: Vector2Float) {
        *self = *self - r;
    }
}

impl MulAssign<f32> for Vector2Float {
    fn mul_assign(&mut self, f: f32) {
        *self = *self * f;
    }
}

impl DivAssign<f32> for Vector2Float {
    fn div_assign(&mut self, f: f32) {
        *self = *self / f;
    }
}

/// "(x, y)" with each component in the default float text (`format_f32`).
impl fmt::Display for Vector2Float {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", format_f32(self.x), format_f32(self.y))
    }
}

/// Hashes the components' bits, with -0.0 and 0.0 alike (see `float_hash`).
impl std::hash::Hash for Vector2Float {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        crate::float_hash::hash_f32(self.x, state);
        crate::float_hash::hash_f32(self.y, state);
    }
}
