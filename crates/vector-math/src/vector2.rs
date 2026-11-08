//! `Vector2`: a 2D f64 vector and its text conversion; the polyline helpers are in
//! `vector2_ex.rs`.
//!
//! Adding, subtracting and scaling are the operators; `multiply_components`/
//! `divide_components` work per component; `get_rotated` rotates; `dot`/`cross` are methods;
//! `equals_within` compares with a tolerance. The quaternion transform is in
//! `vector_transforms.rs`.

use crate::math_helper::{self, MathError};
use crate::number_format::{format_f64, parse_f64};
use crate::{long_hash, Vector3, Vector3Float, LONG_HASH_SEED};
use std::fmt;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

/// A 2D vector of f64.
///
/// Serializes in the shape saved files hold (`{"X":..,"Y":..}`); members missing when
/// reading keep their default (0).
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Vector2 {
    #[serde(rename = "X")]
    pub x: f64,
    #[serde(rename = "Y")]
    pub y: f64,
}

impl Vector2 {
    pub const POSITIVE_INFINITY: Vector2 = Vector2::new(f64::INFINITY, f64::INFINITY);
    pub const NEGATIVE_INFINITY: Vector2 = Vector2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    pub const UNIT_X: Vector2 = Vector2::new(1.0, 0.0);
    pub const UNIT_Y: Vector2 = Vector2::new(0.0, 1.0);
    pub const ZERO: Vector2 = Vector2::new(0.0, 0.0);
    pub const ONE: Vector2 = Vector2::new(1.0, 1.0);
    /// `Marshal.SizeOf(new Vector2())`.
    pub const SIZE_IN_BYTES: i32 = 16;

    pub const fn new(x: f64, y: f64) -> Self {
        Vector2 { x, y }
    }

    /// The x and y of a `Vector3`.
    pub fn from_vector3(vector: Vector3) -> Self {
        Vector2::new(vector.x, vector.y)
    }

    /// The x and y of a `Vector3Float`, widened.
    pub fn from_vector3_float(vector: Vector3Float) -> Self {
        Vector2::new(f64::from(vector.x), f64::from(vector.y))
    }

    /// FNV-1a over the components, starting from [`LONG_HASH_SEED`].
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// FNV-1a over the components, starting from `hash`.
    pub fn get_long_hash_code_with(&self, hash: u64) -> u64 {
        let hash = long_hash::of(self.x, hash);
        long_hash::of(self.y, hash)
    }

    /// The angle swept from `start_position` to `end_position` around this point, each angle
    /// measured in `[0, Tau)`.
    pub fn get_delta_angle(&self, start_position: Vector2, end_position: Vector2) -> f64 {
        let start_position = start_position - *self;
        let mut start_angle = libm::atan2(start_position.y, start_position.x);
        start_angle = if start_angle < 0.0 {
            start_angle + math_helper::TAU
        } else {
            start_angle
        };

        let end_position = end_position - *self;
        let mut end_angle = libm::atan2(end_position.y, end_position.x);
        end_angle = if end_angle < 0.0 {
            end_angle + math_helper::TAU
        } else {
            end_angle
        };

        end_angle - start_angle
    }

    /// The length of the vector.
    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// The distance to `p`.
    pub fn distance(&self, p: Vector2) -> f64 {
        (*self - p).length()
    }

    /// The squared length.
    pub fn length_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    /// Rotates this vector in place by `radians`.
    pub fn rotate(&mut self, radians: f64) {
        *self = self.get_rotated(radians);
    }

    /// This vector rotated by `radians`.
    pub fn get_rotated(&self, radians: f64) -> Vector2 {
        let cos = libm::cos(radians);
        let sin = libm::sin(radians);
        Vector2::new(self.x * cos - self.y * sin, self.y * cos + self.x * sin)
    }

    /// `atan2(y, x)`.
    pub fn get_angle(&self) -> f64 {
        libm::atan2(self.y, self.x)
    }

    /// `get_angle` brought into `[0, Tau]`.
    pub fn get_angle_0_to_2pi(&self) -> Result<f64, MathError> {
        math_helper::range_0_to_tau(self.get_angle())
    }

    /// The perpendicular to the right.
    pub fn get_perpendicular_right(&self) -> Vector2 {
        Vector2::new(self.y, -self.x)
    }

    /// The perpendicular to the left.
    pub fn get_perpendicular_left(&self) -> Vector2 {
        Vector2::new(-self.y, self.x)
    }

    /// A unit-length copy (NaN for the zero vector).
    pub fn get_normal(&self) -> Vector2 {
        let mut temp = *self;
        temp.normalize();
        temp
    }

    /// Scales this vector to unit length (NaN for the zero vector).
    pub fn normalize(&mut self) {
        let scale = 1.0 / self.length();
        self.x *= scale;
        self.y *= scale;
    }

    /// False if any component is NaN or infinite.
    pub fn is_valid(&self) -> bool {
        !(self.x.is_nan() || self.x.is_infinite() || self.y.is_nan() || self.y.is_infinite())
    }

    /// Component-wise product.
    pub fn multiply_components(vector: Vector2, scale: Vector2) -> Vector2 {
        Vector2::new(vector.x * scale.x, vector.y * scale.y)
    }

    /// Component-wise quotient.
    pub fn divide_components(vector: Vector2, scale: Vector2) -> Vector2 {
        Vector2::new(vector.x / scale.x, vector.y / scale.y)
    }

    /// Component-wise minimum.
    pub fn min(a: Vector2, b: Vector2) -> Vector2 {
        Self::component_min(a, b)
    }

    /// Component-wise maximum.
    pub fn max(a: Vector2, b: Vector2) -> Vector2 {
        Self::component_max(a, b)
    }

    /// Parses "x, y" (culture-invariant text). Unparsable parts are 0; missing parts stay 0.
    pub fn parse(s: &str) -> Vector2 {
        let mut result = Vector2::ZERO;
        let values: Vec<f64> = s.split(',').map(parse_f64).collect();
        for (i, value) in values.iter().take(2).enumerate() {
            result[i] = *value;
        }
        result
    }

    /// Each component limited to `[min, max]`.
    pub fn clamp(vec: Vector2, min: Vector2, max: Vector2) -> Vector2 {
        Vector2::new(
            if vec.x < min.x {
                min.x
            } else if vec.x > max.x {
                max.x
            } else {
                vec.x
            },
            if vec.y < min.y {
                min.y
            } else if vec.y > max.y {
                max.y
            } else {
                vec.y
            },
        )
    }

    /// A unit-length copy of `vec`.
    pub fn normalized(vec: Vector2) -> Vector2 {
        let scale = 1.0 / vec.length();
        Vector2::new(vec.x * scale, vec.y * scale)
    }

    /// Distance from `point` to the segment `line_start`-`line_end`.
    pub fn distance_point_to_line(point: Vector2, line_start: Vector2, line_end: Vector2) -> f64 {
        if point == line_start || point == line_end {
            return 0.0;
        }

        if line_start == line_end {
            return (point - line_start).length();
        }

        let line_delta = line_end - line_start;
        let line_length = line_delta.length();
        let line_normal = line_delta.get_normal();
        let delta_from_start = point - line_start;
        let distance_from_start = line_normal.dot(delta_from_start);
        // if we are within the cone of the line
        if distance_from_start >= 0.0 && distance_from_start <= line_length {
            let perpendicular_normal = line_normal.get_perpendicular_left();
            let distance_to_line = perpendicular_normal.dot(delta_from_start);
            return distance_to_line.abs();
        }

        if distance_from_start < 0.0 {
            return delta_from_start.length();
        }

        let delta_from_end = point - line_end;
        delta_from_end.length()
    }

    /// Dot product.
    pub fn dot(&self, b: Vector2) -> f64 {
        self.x * b.x + self.y * b.y
    }

    /// The z of the 3D cross product.
    pub fn cross(&self, right: Vector2) -> f64 {
        self.x * right.y - self.y * right.x
    }

    /// Linear blend from `a` (0) to `b` (1).
    pub fn lerp(a: Vector2, b: Vector2, blend: f64) -> Vector2 {
        Vector2::new(blend * (b.x - a.x) + a.x, blend * (b.y - a.y) + a.y)
    }

    /// `a + u * (b - a) + v * (c - a)`.
    pub fn bary_centric(a: Vector2, b: Vector2, c: Vector2, u: f64, v: f64) -> Vector2 {
        a + u * (b - a) + v * (c - a)
    }

    /// Component-wise minimum.
    pub fn component_min(a: Vector2, b: Vector2) -> Vector2 {
        Vector2::new(
            if a.x < b.x { a.x } else { b.x },
            if a.y < b.y { a.y } else { b.y },
        )
    }

    /// Component-wise maximum.
    pub fn component_max(a: Vector2, b: Vector2) -> Vector2 {
        Vector2::new(
            if a.x > b.x { a.x } else { b.x },
            if a.y > b.y { a.y } else { b.y },
        )
    }

    /// True when every component is strictly within `error_range` of `other`'s.
    pub fn equals_within(&self, other: Vector2, error_range: f64) -> bool {
        (self.x < other.x + error_range && self.x > other.x - error_range)
            && (self.y < other.y + error_range && self.y > other.y - error_range)
    }

    /// Reads a vector from text: drops the first and last character (the parentheses of
    /// [`Display`](fmt::Display)'s "(x, y)") and parses the rest; one value fills both
    /// components. `None` where the text is not a vector (empty or too-short text, or more
    /// than two values).
    pub fn convert_from_string(text: &str) -> Option<Vector2> {
        let chars: Vec<char> = text.chars().collect();
        if chars.len() <= 3 {
            return None;
        }
        let inner: String = chars[1..chars.len() - 1].iter().collect();
        let values: Vec<f64> = inner.split(',').map(parse_f64).collect();
        match values.len() {
            1 => Some(Vector2::new(values[0], values[0])),
            2 => Some(Vector2::new(values[0], values[1])),
            _ => None,
        }
    }
}

/// Indexing: 0 is x, 1 is y; reading any other index gives 0, writing one is a caller
/// bug and panics.
impl Index<usize> for Vector2 {
    type Output = f64;
    fn index(&self, index: usize) -> &f64 {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => &0.0,
        }
    }
}

impl IndexMut<usize> for Vector2 {
    fn index_mut(&mut self, index: usize) -> &mut f64 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Vector2 index {index} out of range"),
        }
    }
}

impl Add for Vector2 {
    type Output = Vector2;
    fn add(self, right: Vector2) -> Vector2 {
        Vector2::new(self.x + right.x, self.y + right.y)
    }
}

impl Sub for Vector2 {
    type Output = Vector2;
    fn sub(self, right: Vector2) -> Vector2 {
        Vector2::new(self.x - right.x, self.y - right.y)
    }
}

impl Neg for Vector2 {
    type Output = Vector2;
    fn neg(self) -> Vector2 {
        Vector2::new(-self.x, -self.y)
    }
}

impl Mul<f64> for Vector2 {
    type Output = Vector2;
    fn mul(self, f: f64) -> Vector2 {
        Vector2::new(self.x * f, self.y * f)
    }
}

impl Mul<Vector2> for f64 {
    type Output = Vector2;
    fn mul(self, vec: Vector2) -> Vector2 {
        Vector2::new(vec.x * self, vec.y * self)
    }
}

/// Multiplies by the reciprocal (not a true division), so `v / s` equals `v * (1 / s)`.
impl Div<f64> for Vector2 {
    type Output = Vector2;
    fn div(self, f: f64) -> Vector2 {
        let mult = 1.0 / f;
        Vector2::new(self.x * mult, self.y * mult)
    }
}

impl Div<Vector2> for f64 {
    type Output = Vector2;
    fn div(self, vec: Vector2) -> Vector2 {
        Vector2::new(self / vec.x, self / vec.y)
    }
}

impl AddAssign for Vector2 {
    fn add_assign(&mut self, right: Vector2) {
        *self = *self + right;
    }
}

impl SubAssign for Vector2 {
    fn sub_assign(&mut self, right: Vector2) {
        *self = *self - right;
    }
}

impl MulAssign<f64> for Vector2 {
    fn mul_assign(&mut self, f: f64) {
        *self = *self * f;
    }
}

impl DivAssign<f64> for Vector2 {
    fn div_assign(&mut self, f: f64) {
        *self = *self / f;
    }
}

/// "(x, y)" with each component in the default double text (`format_f64`).
impl fmt::Display for Vector2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", format_f64(self.x), format_f64(self.y))
    }
}

/// Hashes the components' bits, with -0.0 and 0.0 alike (see `float_hash`).
impl std::hash::Hash for Vector2 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        crate::float_hash::hash_f64(self.x, state);
        crate::float_hash::hash_f64(self.y, state);
    }
}
