//! Polyline helpers over `Vector2` points: functions over `&[Vector2]`, plus
//! `Vector2::get_turn_amount` (below).

use crate::Vector2;

/// The length of the polyline; `is_closed` adds the closing segment from the last point back
/// to the first.
pub fn polygon_length(polygon: &[Vector2], is_closed: bool) -> f64 {
    let mut length = 0.0;
    if polygon.len() > 1 {
        let mut previous_point = polygon[0];
        if is_closed {
            previous_point = polygon[polygon.len() - 1];
        }

        let start = if is_closed { 0 } else { 1 };
        for current_point in &polygon[start..] {
            length += (previous_point - *current_point).length();
            previous_point = *current_point;
        }
    }

    length
}

/// The length along the polyline from the first point to point `index` (clamped to the
/// points); `_is_closed` is unused, since the length up to a point never needs the closing
/// segment.
pub fn length_to(polygon: &[Vector2], index: i32, _is_closed: bool) -> f64 {
    let mut length = 0.0;
    let last = polygon.len() as i64 - 1;
    let index = 0.max(last.min(i64::from(index)));
    for i in 1..=index as usize {
        length += (polygon[i] - polygon[i - 1]).length();
    }

    length
}

/// The point `length_from_start` along the polyline. A closed polyline
/// wraps the length around; an open one clamps to its ends. The total it wraps by is always
/// the closed length, even for an open polyline.
///
/// Panics on an empty `polygon` (an open polyline asked for a point past either end), a
/// caller bug.
pub fn get_position_at(polygon: &[Vector2], length_from_start: f64, closed: bool) -> Vector2 {
    let mut length_from_start = length_from_start;
    let total_length = polygon_length(polygon, true);
    if length_from_start > total_length {
        if closed {
            let ratio = length_from_start / total_length;
            // The cast truncates, and saturates rather than wrapping on overflow.
            let times = ratio as i32;
            let remainder = ratio - f64::from(times);
            length_from_start = remainder * total_length;
        } else {
            return polygon[polygon.len() - 1];
        }
    } else if length_from_start <= 0.0 {
        if closed {
            let ratio = length_from_start / total_length;
            let times = ratio as i32;
            let remainder = ratio - f64::from(times);
            length_from_start = (1.0 + remainder) * total_length;
        } else {
            return polygon[0];
        }
    }

    let mut position = Vector2::default();
    let mut length = 0.0;
    if polygon.len() > 1 {
        position = polygon[0];
        let mut current_point = polygon[0];
        let polygon_count = polygon.len();
        let end = if closed {
            polygon_count + 1
        } else {
            polygon_count
        };
        for i in 1..end {
            let next_point = polygon[(polygon_count + i) % polygon_count];
            let segment_length = (next_point - current_point).length();
            if length + segment_length > length_from_start {
                // return the distance along this segment
                let distance_along_this_segment = length_from_start - length;
                let delte_from_current =
                    (next_point - current_point) * distance_along_this_segment / segment_length;
                return current_point + delte_from_current;
            }

            position = next_point;
            length += segment_length;
            current_point = next_point;
        }
    }

    position
}

impl Vector2 {
    /// The signed angle the path turns at this point, coming from `prev_point` and going to
    /// `next_point`; 0 if any two of the points coincide.
    pub fn get_turn_amount(&self, prev_point: Vector2, next_point: Vector2) -> f64 {
        let current_point = *self;
        if prev_point != current_point && current_point != next_point && next_point != prev_point {
            let prev_point = current_point - prev_point;
            let next_point = next_point - current_point;

            let prev_angle = prev_point.y.atan2(prev_point.x);

            // undo the rotation
            let next_point = next_point.get_rotated(-prev_angle);
            return next_point.y.atan2(next_point.x);
        }

        0.0
    }
}
