//! Axis-aligned rectangle in doubles (y up: bottom < top). Inflating by a border and containing
//! a raster point take GUI/raster types and are not here (agg-gui owns borders).

use crate::ieee_math::{max_f64, min_f64};
use crate::number_format::format_f64;
use crate::rectangle_int::RectangleInt;
use crate::Vector2;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::ops::{BitAnd, BitOr, Mul};

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct RectangleDouble {
    pub left: f64,
    pub bottom: f64,
    pub right: f64,
    pub top: f64,
}

/// Equal rectangles hash equal. -0 is folded into 0, which `==` treats as the same value.
impl Hash for RectangleDouble {
    fn hash<H: Hasher>(&self, state: &mut H) {
        for side in [self.left, self.right, self.bottom, self.top] {
            (side + 0.0).to_bits().hash(state);
        }
    }
}

/// Which sides of a rectangle a point lies beyond (Cohen-Sutherland), as combinable bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct OutCode(pub u32);

impl OutCode {
    pub const INSIDE: OutCode = OutCode(0);
    pub const LEFT: OutCode = OutCode(1);
    pub const RIGHT: OutCode = OutCode(2);
    pub const BOTTOM: OutCode = OutCode(4);
    pub const TOP: OutCode = OutCode(8);
    pub const SURROUNDED: OutCode = OutCode(15);

    pub fn has_flag(self, flag: OutCode) -> bool {
        self.0 & flag.0 == flag.0
    }
}

impl BitOr for OutCode {
    type Output = OutCode;
    fn bitor(self, rhs: OutCode) -> OutCode {
        OutCode(self.0 | rhs.0)
    }
}

impl BitAnd for OutCode {
    type Output = OutCode;
    fn bitand(self, rhs: OutCode) -> OutCode {
        OutCode(self.0 & rhs.0)
    }
}

impl RectangleDouble {
    /// An inverted rectangle, so the first `expand_to_include` sets every side.
    pub const ZERO_INTERSECTION: RectangleDouble =
        RectangleDouble::new(f64::MAX, f64::MAX, f64::MIN, f64::MIN);

    pub const fn new(left: f64, bottom: f64, right: f64, top: f64) -> Self {
        RectangleDouble {
            left,
            bottom,
            right,
            top,
        }
    }

    pub fn from_rectangle_int(int_rect: RectangleInt) -> Self {
        Self::new(
            f64::from(int_rect.left),
            f64::from(int_rect.bottom),
            f64::from(int_rect.right),
            f64::from(int_rect.top),
        )
    }

    /// The rectangle spanning two corners, in either order.
    pub fn from_points(position1: Vector2, position2: Vector2) -> Self {
        Self::new(
            min_f64(position1.x, position2.x),
            min_f64(position1.y, position2.y),
            max_f64(position1.x, position2.x),
            max_f64(position1.y, position2.y),
        )
    }

    pub fn set_rect(&mut self, left: f64, bottom: f64, right: f64, top: f64) {
        self.init(left, bottom, right, top);
    }

    pub fn init(&mut self, left: f64, bottom: f64, right: f64, top: f64) {
        *self = Self::new(left, bottom, right, top);
    }

    /// Whether every side is within `epsilon` of `other`'s.
    pub fn equals_eps(&self, other: &RectangleDouble, epsilon: f64) -> bool {
        (self.left - other.left).abs() <= epsilon
            && (self.bottom - other.bottom).abs() <= epsilon
            && (self.right - other.right).abs() <= epsilon
            && (self.top - other.top).abs() <= epsilon
    }

    pub fn width(&self) -> f64 {
        self.right - self.left
    }

    pub fn height(&self) -> f64 {
        self.top - self.bottom
    }

    /// Swaps sides so left <= right and bottom <= top; returns the result.
    pub fn normalize(&mut self) -> RectangleDouble {
        if self.left > self.right {
            std::mem::swap(&mut self.left, &mut self.right);
        }
        if self.bottom > self.top {
            std::mem::swap(&mut self.bottom, &mut self.top);
        }
        *self
    }

    pub fn clip(&mut self, r: RectangleDouble) -> bool {
        if self.right > r.right {
            self.right = r.right;
        }
        if self.top > r.top {
            self.top = r.top;
        }
        if self.left < r.left {
            self.left = r.left;
        }
        if self.bottom < r.bottom {
            self.bottom = r.bottom;
        }
        self.left <= self.right && self.bottom <= self.top
    }

    pub fn is_valid(&self) -> bool {
        self.left <= self.right && self.bottom <= self.top
    }

    pub fn contains_xy(&self, x: f64, y: f64) -> bool {
        x >= self.left && x <= self.right && y >= self.bottom && y <= self.top
    }

    pub fn contains_rect(&self, inner_rect: RectangleDouble) -> bool {
        self.contains_xy(inner_rect.left, inner_rect.bottom)
            && self.contains_xy(inner_rect.right, inner_rect.top)
    }

    pub fn contains(&self, position: Vector2) -> bool {
        self.contains_xy(position.x, position.y)
    }

    /// Sets this to `rect_to_copy` clipped by `rect_to_intersect_with`; true when non-empty.
    pub fn intersect_rectangles(
        &mut self,
        rect_to_copy: RectangleDouble,
        rect_to_intersect_with: RectangleDouble,
    ) -> bool {
        *self = rect_to_copy;
        self.intersect_with_rectangle(rect_to_intersect_with)
    }

    pub fn is_touching(&self, rect_to_intersect_with: RectangleDouble) -> bool {
        let mut temp = *self;
        temp.intersect_with_rectangle(rect_to_intersect_with)
    }

    pub fn intersect_with_rectangle(&mut self, r: RectangleDouble) -> bool {
        if self.left < r.left {
            self.left = r.left;
        }
        if self.bottom < r.bottom {
            self.bottom = r.bottom;
        }
        if self.right > r.right {
            self.right = r.right;
        }
        if self.top > r.top {
            self.top = r.top;
        }
        self.left < self.right && self.bottom < self.top
    }

    /// Sets this rectangle to the smallest one holding both `r1` and `r2`.
    pub fn unite_rectangles(&mut self, r1: RectangleDouble, r2: RectangleDouble) {
        self.left = r1.left;
        self.bottom = r1.bottom;
        self.right = r1.right;
        self.top = r1.top;
        if self.right < r2.right {
            self.right = r2.right;
        }
        if self.top < r2.top {
            self.top = r2.top;
        }
        if self.left > r2.left {
            self.left = r2.left;
        }
        if self.bottom > r2.bottom {
            self.bottom = r2.bottom;
        }
    }

    pub fn expand_to_include_rect(&mut self, r: RectangleDouble) {
        if self.right < r.right {
            self.right = r.right;
        }
        if self.top < r.top {
            self.top = r.top;
        }
        if self.left > r.left {
            self.left = r.left;
        }
        if self.bottom > r.bottom {
            self.bottom = r.bottom;
        }
    }

    pub fn expand_to_include(&mut self, position: Vector2) {
        self.expand_to_include_xy(position.x, position.y);
    }

    pub fn expand_to_include_xy(&mut self, x: f64, y: f64) {
        if self.right < x {
            self.right = x;
        }
        if self.top < y {
            self.top = y;
        }
        if self.left > x {
            self.left = x;
        }
        if self.bottom > y {
            self.bottom = y;
        }
    }

    pub fn inflate(&mut self, inflate_size: f64) {
        self.left -= inflate_size;
        self.bottom -= inflate_size;
        self.right += inflate_size;
        self.top += inflate_size;
    }

    pub fn offset(&mut self, offset: Vector2) {
        self.offset_xy(offset.x, offset.y);
    }

    pub fn offset_xy(&mut self, x: f64, y: f64) {
        self.left += x;
        self.bottom += y;
        self.right += x;
        self.top += y;
    }

    pub fn center(&self) -> Vector2 {
        Vector2::new(self.x_center(), self.y_center())
    }

    pub fn x_center(&self) -> f64 {
        (self.right + self.left) / 2.0
    }

    pub fn y_center(&self) -> f64 {
        (self.top + self.bottom) / 2.0
    }

    pub fn size(&self) -> Vector2 {
        Vector2::new(self.width(), self.height())
    }

    pub fn compute_out_code_xy(&self, x: f64, y: f64) -> OutCode {
        let mut code = OutCode::INSIDE;
        if x < self.left {
            code = code | OutCode::LEFT;
        }
        if x > self.right {
            code = code | OutCode::RIGHT;
        }
        if y < self.bottom {
            code = code | OutCode::BOTTOM;
        }
        if y > self.top {
            code = code | OutCode::TOP;
        }
        code
    }

    pub fn compute_out_code(&self, p: Vector2) -> OutCode {
        self.compute_out_code_xy(p.x, p.y)
    }

    /// `clip_to` always has a side bit here: `clip_segment` only asks for an outside point.
    fn calculate_intersection(&self, p1: Vector2, p2: Vector2, clip_to: OutCode) -> Vector2 {
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        let slope_y = dx / dy;
        let slope_x = dy / dx;
        if clip_to.has_flag(OutCode::TOP) {
            Vector2::new(p1.x + slope_y * (self.top - p1.y), self.top)
        } else if clip_to.has_flag(OutCode::BOTTOM) {
            Vector2::new(p1.x + slope_y * (self.bottom - p1.y), self.bottom)
        } else if clip_to.has_flag(OutCode::RIGHT) {
            Vector2::new(self.right, p1.y + slope_x * (self.right - p1.x))
        } else {
            Vector2::new(self.left, p1.y + slope_x * (self.left - p1.x))
        }
    }

    fn clip_segment(&self, mut p1: Vector2, mut p2: Vector2) -> Option<(Vector2, Vector2)> {
        let mut out_code_p1 = self.compute_out_code(p1);
        let mut out_code_p2 = self.compute_out_code(p2);
        loop {
            if (out_code_p1 | out_code_p2) == OutCode::INSIDE {
                return Some((p1, p2));
            }
            if (out_code_p1 & out_code_p2).0 != 0 {
                return None;
            }
            let out_code = if out_code_p1 != OutCode::INSIDE {
                out_code_p1
            } else {
                out_code_p2
            };
            let p = self.calculate_intersection(p1, p2, out_code);
            if out_code == out_code_p1 {
                p1 = p;
                out_code_p1 = self.compute_out_code(p1);
            } else {
                p2 = p;
                out_code_p2 = self.compute_out_code(p2);
            }
        }
    }

    pub fn clamp(&self, position: Vector2) -> Vector2 {
        let new_x = min_f64(self.right, max_f64(self.left, position.x));
        let new_y = min_f64(self.top, max_f64(self.bottom, position.y));
        Vector2::new(new_x, new_y)
    }

    /// True when the segment p1-p2 crosses this rectangle.
    pub fn clip_line(&self, p1: Vector2, p2: Vector2) -> bool {
        self.clip_segment(p1, p2).is_some()
    }
}

impl Mul<f64> for RectangleDouble {
    type Output = RectangleDouble;
    fn mul(self, b: f64) -> RectangleDouble {
        RectangleDouble::new(self.left * b, self.bottom * b, self.right * b, self.top * b)
    }
}

impl Mul<RectangleDouble> for f64 {
    type Output = RectangleDouble;
    fn mul(self, a: RectangleDouble) -> RectangleDouble {
        a * self
    }
}

/// `L:{Left}, B:{Bottom}, R:{Right}, T:{Top}`.
impl fmt::Display for RectangleDouble {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "L:{}, B:{}, R:{}, T:{}",
            format_f64(self.left),
            format_f64(self.bottom),
            format_f64(self.right),
            format_f64(self.top)
        )
    }
}
