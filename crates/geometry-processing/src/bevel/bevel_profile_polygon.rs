//! The plane-polygon arithmetic every swept section's CROSS SECTION needs: which way it is wound,
//! and how to cap it with triangles.
//!
//! Separate from the sweep because three different builders cap the same shape - the chain
//! sweep's own square ends, the tapered runout's, and `BevelPartneredBeadSection`'s stations - and
//! it is a simple polygon rather than a convex one (the cleared apex turns back over the corner),
//! so a fan from any one point would fold and each of them would have to get the same ear clip
//! right.

use vector_math::Vector2;

/// Ear clipping over the cross section, which is a simple polygon but not a convex one - the
/// cleared apex turns back over the corner - so a fan from any one point would fold. Returns
/// vertex indices in threes, counter clockwise, or `None` if the polygon was not simple after all.
///
/// Shared with `BevelPartneredBead`, which caps its own stations and has exactly the same polygon
/// to cap: simple, wound the same way, and reflex where the arc turns back.
///
/// A refusal is `None`: no caller has a use for a partial triangle list. Fewer than three points is
/// `None` too, since no polygon that small can be capped.
pub fn try_triangulate(polygon: &[Vector2]) -> Option<Vec<i32>> {
    if polygon.len() < 3 {
        return None;
    }

    let mut triangles = Vec::with_capacity((polygon.len() - 2) * 3);
    let mut remaining: Vec<i32> = (0..polygon.len() as i32).collect();

    // Each pass either clips an ear or the polygon is not simple; the guard is what stops a bad
    // profile spinning here rather than failing. The count is tested before it is decremented.
    let mut attempts_left = (polygon.len() * polygon.len()) as i64;

    while remaining.len() > 3 && {
        let go = attempts_left > 0;
        attempts_left -= 1;
        go
    } {
        let mut clipped = false;

        for i in 0..remaining.len() {
            let previous = remaining[(i + remaining.len() - 1) % remaining.len()];
            let current = remaining[i];
            let next = remaining[(i + 1) % remaining.len()];

            if !is_ear(polygon, &remaining, previous, current, next) {
                continue;
            }

            triangles.push(previous);
            triangles.push(current);
            triangles.push(next);

            remaining.remove(i);
            clipped = true;
            break;
        }

        if !clipped {
            return None;
        }
    }

    if remaining.len() != 3 {
        return None;
    }

    triangles.push(remaining[0]);
    triangles.push(remaining[1]);
    triangles.push(remaining[2]);

    Some(triangles)
}

fn is_ear(polygon: &[Vector2], remaining: &[i32], previous: i32, current: i32, next: i32) -> bool {
    let a = polygon[previous as usize];
    let b = polygon[current as usize];
    let c = polygon[next as usize];

    let area = cross(b - a, c - a);

    // Reflex (or collinear) in a counter clockwise polygon, so not an ear. Written as an `area <=
    // 0` refusal, so a NaN area is not refused here.
    if area <= 0.0 {
        return false;
    }

    for &index in remaining {
        if index == previous || index == current || index == next {
            continue;
        }

        let point = polygon[index as usize];

        if cross(b - a, point - a) >= 0.0
            && cross(c - b, point - b) >= 0.0
            && cross(a - c, point - c) >= 0.0
        {
            return false;
        }
    }

    true
}

/// The polygon's signed area: positive when it winds counter clockwise.
pub fn signed_area(polygon: &[Vector2]) -> f64 {
    let mut twice_area = 0.0;

    for i in 0..polygon.len() {
        twice_area += cross(polygon[i], polygon[(i + 1) % polygon.len()]);
    }

    twice_area / 2.0
}

fn cross(a: Vector2, b: Vector2) -> f64 {
    (a.x * b.y) - (a.y * b.x)
}
