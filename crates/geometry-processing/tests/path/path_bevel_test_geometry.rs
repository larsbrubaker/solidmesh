//! Ports `PathBevelTestGeometry`: the measurements both PathBevel test classes take of a beveled
//! contour.

#![allow(dead_code)]

use solidmesh_math::Vector2;

/// How far back along its edge a beveled corner starts.
pub fn tangent_distance(corner: Vector2, arc_point: Vector2) -> f64 {
    (arc_point - corner).length()
}

/// The center of the circle through three points.
pub fn arc_center(a: Vector2, b: Vector2, c: Vector2) -> Vector2 {
    let determinant = 2.0 * ((a.x * (b.y - c.y)) + (b.x * (c.y - a.y)) + (c.x * (a.y - b.y)));
    let a_squared = (a.x * a.x) + (a.y * a.y);
    let b_squared = (b.x * b.x) + (b.y * b.y);
    let c_squared = (c.x * c.x) + (c.y * c.y);
    let x = ((a_squared * (b.y - c.y)) + (b_squared * (c.y - a.y)) + (c_squared * (a.y - b.y)))
        / determinant;
    let y = ((a_squared * (c.x - b.x)) + (b_squared * (a.x - c.x)) + (c_squared * (b.x - a.x)))
        / determinant;
    Vector2::new(x, y)
}

/// The angle in radians between two directions read as lines rather than arrows.
pub fn angle_between_directions(a: Vector2, b: Vector2) -> f64 {
    let cross = (a.x * b.y) - (a.y * b.x);
    let dot = (a.x * b.x) + (a.y * b.y);
    cross.abs().atan2(dot.abs())
}

/// The nearest point of a segment to `point`: its parameter and distance, or `None` for a
/// degenerate segment.
fn nearest_on_segment(from: Vector2, to: Vector2, point: Vector2) -> Option<(f64, f64, f64)> {
    let segment = to - from;
    let length = segment.length();
    if length < 1e-12 {
        return None;
    }
    let t = ((point - from).dot(segment) / (length * length)).clamp(0.0, 1.0);
    let distance = ((from + (segment * t)) - point).length();
    Some((t, distance, length))
}

/// How far along `polyline` the point nearest `point` sits, in arc length from the start.
pub fn position_along(polyline: &[Vector2], point: Vector2) -> f64 {
    let mut position = 0.0;
    let mut travelled = 0.0;
    let mut closest = f64::MAX;
    for i in 1..polyline.len() {
        let Some((t, distance, length)) = nearest_on_segment(polyline[i - 1], polyline[i], point)
        else {
            continue;
        };
        if distance < closest {
            closest = distance;
            position = travelled + (t * length);
        }
        travelled += length;
    }
    position
}

/// The direction of whichever segment of `polyline` the point sits on.
pub fn local_direction_at(polyline: &[Vector2], point: Vector2) -> Vector2 {
    let mut direction = Vector2::ZERO;
    let mut closest = f64::MAX;
    for i in 1..polyline.len() {
        let Some((_, distance, length)) = nearest_on_segment(polyline[i - 1], polyline[i], point)
        else {
            continue;
        };
        if distance < closest {
            closest = distance;
            direction = (polyline[i] - polyline[i - 1]) / length;
        }
    }
    direction
}

/// The points the bevel inserted at one corner: the stretch of `output` off the un-beveled
/// contour, plus the point either side of it (the tangencies).
pub fn inserted_arc(output: &[Vector2], flattened: &[Vector2], tolerance: f64) -> Vec<Vector2> {
    let mut first: Option<usize> = None;
    let mut last = 0;
    for (i, point) in output.iter().enumerate() {
        if distance_to_polyline(flattened, *point) > tolerance {
            if first.is_none() {
                first = Some(i);
            }
            last = i;
        }
    }
    let Some(first) = first else {
        return Vec::new();
    };
    let start = first.saturating_sub(1);
    let end = (last + 1).min(output.len() - 1);
    output[start..=end].to_vec()
}

/// How far `point` is from `polyline`, measured to its segments.
pub fn distance_to_polyline(polyline: &[Vector2], point: Vector2) -> f64 {
    let mut closest = f64::MAX;
    for i in 1..polyline.len() {
        if let Some((_, distance, _)) = nearest_on_segment(polyline[i - 1], polyline[i], point) {
            closest = closest.min(distance);
        }
    }
    closest
}

/// How far round `center` the sampled arc travels, in radians, positive counter-clockwise.
pub fn sweep_about(arc: &[Vector2], center: Vector2) -> f64 {
    let mut sweep = 0.0;
    for i in 1..arc.len() {
        let from = arc[i - 1] - center;
        let to = arc[i] - center;
        sweep += ((from.x * to.y) - (from.y * to.x)).atan2(from.dot(to));
    }
    sweep
}

/// Whether any two non-adjacent segments of `contour` properly cross.
pub fn self_intersects(contour: &[Vector2]) -> bool {
    for i in 0..contour.len().saturating_sub(1) {
        for j in (i + 2)..contour.len().saturating_sub(1) {
            if segments_cross(contour[i], contour[i + 1], contour[j], contour[j + 1]) {
                return true;
            }
        }
    }
    false
}

fn segments_cross(a: Vector2, b: Vector2, c: Vector2, d: Vector2) -> bool {
    let side = |from: Vector2, to: Vector2, point: Vector2| {
        let along = to - from;
        let across = point - from;
        (along.x * across.y) - (along.y * across.x)
    };
    let first = side(a, b, c);
    let second = side(a, b, d);
    let third = side(c, d, a);
    let fourth = side(c, d, b);
    first * second < -1e-18 && third * fourth < -1e-18
}
