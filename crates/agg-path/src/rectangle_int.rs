//! Axis-aligned rectangle in ints (y up). Sums wrap rather than panic: a rectangle far out of
//! range gives a wrong box, never a crash.

use crate::rectangle_double::RectangleDouble;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Hash)]
pub struct RectangleInt {
    pub left: i32,
    pub bottom: i32,
    pub right: i32,
    pub top: i32,
}

/// [`RectangleInt::clip_rects`] fails when the source and destination differ in size along a
/// clipped axis.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClipRectsError {
    /// "source and dest rects must have the same height"
    HeightMismatch,
    /// "source and dest rects must have the same width"
    WidthMismatch,
}

impl RectangleInt {
    pub const fn new(left: i32, bottom: i32, right: i32, top: i32) -> Self {
        RectangleInt {
            left,
            bottom,
            right,
            top,
        }
    }

    /// Floors left/bottom and ceils right/top (an out-of-range double saturates).
    pub fn from_rectangle_double(rect: RectangleDouble) -> Self {
        Self::new(
            rect.left.floor() as i32,
            rect.bottom.floor() as i32,
            rect.right.ceil() as i32,
            rect.top.ceil() as i32,
        )
    }

    pub fn height(&self) -> i32 {
        self.top.wrapping_sub(self.bottom)
    }

    pub fn width(&self) -> i32 {
        self.right.wrapping_sub(self.left)
    }

    /// Clips `dest` to `bounding`; false when nothing is left. Note: its top/bottom tests read as
    /// a y-down rectangle.
    pub fn clip_rect(bounding: RectangleInt, dest: &mut RectangleInt) -> bool {
        if dest.top < bounding.top {
            dest.top = bounding.top;
            if dest.top >= dest.bottom {
                return false;
            }
        }
        if dest.bottom > bounding.bottom {
            dest.bottom = bounding.bottom;
            if dest.bottom <= dest.top {
                return false;
            }
        }
        if dest.left < bounding.left {
            dest.left = bounding.left;
            if dest.left >= dest.right {
                return false;
            }
        }
        if dest.right > bounding.right {
            dest.right = bounding.right;
            if dest.right <= dest.left {
                return false;
            }
        }
        true
    }

    /// Clips `dest` to `bounding`, moving `source`'s sides by the same amounts (as `clip_rect`,
    /// y-down).
    pub fn clip_rects(
        bounding: RectangleInt,
        source: &mut RectangleInt,
        dest: &mut RectangleInt,
    ) -> Result<bool, ClipRectsError> {
        if dest.top < bounding.top {
            if source.height() != dest.height() {
                return Err(ClipRectsError::HeightMismatch);
            }
            source.top = source.top.wrapping_add(bounding.top.wrapping_sub(dest.top));
            dest.top = bounding.top;
            if dest.top >= dest.bottom {
                return Ok(false);
            }
        }
        if dest.bottom > bounding.bottom {
            if source.height() != dest.height() {
                return Err(ClipRectsError::HeightMismatch);
            }
            source.bottom = source
                .bottom
                .wrapping_sub(dest.bottom.wrapping_sub(bounding.bottom));
            dest.bottom = bounding.bottom;
            if dest.bottom <= dest.top {
                return Ok(false);
            }
        }
        if dest.left < bounding.left {
            if source.width() != dest.width() {
                return Err(ClipRectsError::WidthMismatch);
            }
            source.left = source
                .left
                .wrapping_add(bounding.left.wrapping_sub(dest.left));
            dest.left = bounding.left;
            if dest.left >= dest.right {
                return Ok(false);
            }
        }
        if dest.right > bounding.right {
            if source.width() != dest.width() {
                return Err(ClipRectsError::WidthMismatch);
            }
            source.right = source
                .right
                .wrapping_sub(dest.right.wrapping_sub(bounding.right));
            dest.right = bounding.right;
            if dest.right <= dest.left {
                return Ok(false);
            }
        }
        Ok(true)
    }

    pub fn do_intersect(rect1: RectangleInt, rect2: RectangleInt) -> bool {
        let mut temp = rect1;
        temp.intersect_with_rectangle(rect2)
    }

    pub fn clip(&mut self, r: RectangleInt) -> bool {
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

    pub fn expand_to_include(&mut self, r: RectangleInt) {
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

    pub fn hit_test(&self, x: i32, y: i32) -> bool {
        x >= self.left && x <= self.right && y >= self.bottom && y <= self.top
    }

    pub fn inflate(&mut self, inflate_size: i32) {
        self.left = self.left.wrapping_sub(inflate_size);
        self.bottom = self.bottom.wrapping_sub(inflate_size);
        self.right = self.right.wrapping_add(inflate_size);
        self.top = self.top.wrapping_add(inflate_size);
    }

    pub fn init(&mut self, x1: i32, y1: i32, x2: i32, y2: i32) {
        *self = Self::new(x1, y1, x2, y2);
    }

    pub fn intersect_rectangles(
        &mut self,
        rect_to_copy: RectangleInt,
        rect_to_intersect_with: RectangleInt,
    ) -> bool {
        *self = rect_to_copy;
        self.intersect_with_rectangle(rect_to_intersect_with)
    }

    pub fn intersect_with_rectangle(&mut self, r: RectangleInt) -> bool {
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

    pub fn is_valid(&self) -> bool {
        self.left <= self.right && self.bottom <= self.top
    }

    pub fn normalize(&mut self) -> RectangleInt {
        if self.left > self.right {
            std::mem::swap(&mut self.left, &mut self.right);
        }
        if self.bottom > self.top {
            std::mem::swap(&mut self.bottom, &mut self.top);
        }
        *self
    }

    pub fn offset(&mut self, x: i32, y: i32) {
        self.left = self.left.wrapping_add(x);
        self.bottom = self.bottom.wrapping_add(y);
        self.right = self.right.wrapping_add(x);
        self.top = self.top.wrapping_add(y);
    }

    pub fn set_rect(&mut self, left: i32, bottom: i32, right: i32, top: i32) {
        self.init(left, bottom, right, top);
    }

    /// The union of `r1` and `r2` (it assigns `right = r1.top`; see RectangleDouble).
    pub fn unite_rectangles(&mut self, r1: RectangleInt, r2: RectangleInt) {
        self.left = r1.left;
        self.bottom = r1.bottom;
        self.right = r1.top;
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
}
