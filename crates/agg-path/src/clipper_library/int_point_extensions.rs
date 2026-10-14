//! ClipperLib's `IntPoint` helpers (segment orientation, on-segment, intersection kind and
//! point) and ClipperLib's `new IntPoint(double, double)`, on clipper2-rust's `Point64`.
//! Polygon-mesh slices and stitches with them; the polygon helpers are in
//! [`super::polygon_extensions`].

use clipper2_rust::Point64;

/// How two segments meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Intersection {
    None,
    Colinear,
    Intersect,
}

/// ClipperLib's `new IntPoint(double, double)`: each coordinate rounded half to even
/// (`Math.Round`).
pub fn int_point(x: f64, y: f64) -> Point64 {
    Point64::new(x.round_ties_even() as i64, y.round_ties_even() as i64)
}

fn sub(a: Point64, b: Point64) -> Point64 {
    Point64::new(a.x - b.x, a.y - b.y)
}

fn dot(a: Point64, b: Point64) -> i64 {
    a.x * b.x + a.y * b.y
}

pub fn length_squared(point: Point64) -> i64 {
    point.x * point.x + point.y * point.y
}

/// The length truncated to a whole unit.
pub fn length(point: Point64) -> i64 {
    (length_squared(point) as f64).sqrt() as i64
}

/// The orientation of the ordered triplet: 0 collinear, 1 clockwise, 2 counterclockwise.
pub fn orientation(start: Point64, end: Point64, test: Point64) -> i32 {
    let val = (end.y - start.y) * (test.x - end.x) - (end.x - start.x) * (test.y - end.y);
    if val == 0 {
        return 0;
    }
    if val > 0 {
        1
    } else {
        2
    }
}

/// Whether `test_position` lies on the segment `start`-`end`, measured in whole units.
pub fn on_segment(test_position: Point64, start: Point64, end: Point64) -> bool {
    if start == end {
        return test_position == start;
    }

    let segment_delta = sub(end, start);
    let segment_length = length(segment_delta);
    let point_rel_start = sub(test_position, start);
    let distance_from_start = dot(segment_delta, point_rel_start) / segment_length;

    if distance_from_start >= 0 && distance_from_start <= segment_length {
        let segment_delta_left = Point64::new(-segment_delta.y, segment_delta.x);
        let distance_from_start_left = dot(segment_delta_left, point_rel_start) / segment_length;
        if distance_from_start_left == 0 {
            return true;
        }
    }

    false
}

/// How segment A meets segment B.
pub fn get_intersection(
    start_a: Point64,
    end_a: Point64,
    start_b: Point64,
    end_b: Point64,
) -> Intersection {
    let o1 = orientation(start_a, end_a, start_b);
    let o2 = orientation(start_a, end_a, end_b);
    let o3 = orientation(start_b, end_b, start_a);
    let o4 = orientation(start_b, end_b, end_a);

    // The collinear special cases: an end of one segment lies on the other.
    if o1 == 0 && on_segment(start_b, start_a, end_a) {
        return Intersection::Colinear;
    }
    if o2 == 0 && on_segment(end_b, start_a, end_a) {
        return Intersection::Colinear;
    }
    if o3 == 0 && on_segment(start_a, start_b, end_b) {
        return Intersection::Colinear;
    }
    if o4 == 0 && on_segment(end_a, start_b, end_b) {
        return Intersection::Colinear;
    }

    // General case
    if o1 != o2 && o3 != o4 {
        return Intersection::Intersect;
    }

    Intersection::None
}

/// Where the lines through A and B cross, or `None` when they are parallel to within a unit
/// (integer division truncates toward zero).
pub fn calc_intersection(a1: Point64, a2: Point64, b1: Point64, b2: Point64) -> Option<Point64> {
    let intersection_epsilon = 1;
    let num = (a1.y - b1.y) * (b2.x - b1.x) - (a1.x - b1.x) * (b2.y - b1.y);
    let den = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);
    if den.abs() < intersection_epsilon {
        return None;
    }

    Some(Point64::new(
        a1.x + (a2.x - a1.x) * num / den,
        a1.y + (a2.y - a1.y) * num / den,
    ))
}
