//! The walk along the level line: whether a point stands in a triangle ([`in_triangle`]) and the
//! walk to the line near a point
//! ([`try_find_line_point_near`]).

use solidmesh_math::Vector3;

/// True when `p` (in the triangle's plane) stands inside triangle abc or on its rim, to a
/// barycentric slack of 1e-12; false for a triangle with no area.
pub fn in_triangle(a: Vector3, b: Vector3, c: Vector3, p: Vector3) -> bool {
    const SLACK: f64 = 1e-12;
    let normal = (b - a).cross(c - a);
    let area_squared = normal.length_squared();
    if area_squared <= 0.0 {
        return false;
    }

    let u = (b - p).cross(c - p).dot(normal) / area_squared;
    let v = (c - p).cross(a - p).dot(normal) / area_squared;
    let w = 1.0 - u - v;
    u >= -SLACK && v >= -SLACK && w >= -SLACK
}

/// The true line's point near a chord midpoint - step from the
/// point along the field's steepest direction in the triangle's own plane (read by central
/// differences) and bisect the field there; the opposite direction second. `None` where the
/// triangle has no area, the point stands on a crease (the one-sided slopes disagree) or a ridge
/// (a slope that is only rounding), or the line is not reached inside the triangle - the caller
/// then halves the triangle, which is exact.
pub fn try_find_line_point_near(
    mesh: &super::LevelLineMesh,
    reading: &super::VertexFieldReading,
    triangle: i32,
    from: Vector3,
    scale: f64,
) -> Result<Option<Vector3>, super::WorkError> {
    let t = mesh.triangles()[triangle as usize];
    let a = mesh.position(t.a);
    let b = mesh.position(t.b);
    let c = mesh.position(t.c);

    let e1 = (b - a).get_normal();
    let normal = (b - a).cross(c - a);
    if normal.length_squared() <= 0.0 {
        return Ok(None);
    }

    let e2 = normal.get_normal().cross(e1).get_normal();
    let here = reading.value_at_point(from)?;
    let h = 1e-7 * scale;
    let ahead_of_e1 = reading.value_at_point(from + (e1 * h))?;
    let behind_e1 = reading.value_at_point(from - (e1 * h))?;
    let ahead_of_e2 = reading.value_at_point(from + (e2 * h))?;
    let behind_e2 = reading.value_at_point(from - (e2 * h))?;
    let slope_along_e1 = (ahead_of_e1 - behind_e1) / (2.0 * h);
    let slope_along_e2 = (ahead_of_e2 - behind_e2) / (2.0 * h);
    let uphill = (e1 * slope_along_e1) + (e2 * slope_along_e2);

    // A CREASE has no gradient: the slope ahead and the slope behind disagree by an order one
    // amount, and their average is a fact about the two sample points, not the field.
    const SLOPES_THAT_DISAGREE: f64 = 1e-3;
    let jump_along_e1 = (((ahead_of_e1 - here) - (here - behind_e1)) / h).abs();
    let jump_along_e2 = (((ahead_of_e2 - here) - (here - behind_e2)) / h).abs();
    if jump_along_e1 > SLOPES_THAT_DISAGREE || jump_along_e2 > SLOPES_THAT_DISAGREE {
        return Ok(None);
    }

    // The field is 1-Lipschitz, so a REAL slope is of order one; one far below that is the
    // central difference's own rounding (all there is on a RIDGE), read as NO slope.
    const SLOPE_THAT_IS_ONLY_ROUNDING: f64 = 1e-6;
    if uphill.length() <= SLOPE_THAT_IS_ONLY_ROUNDING {
        return Ok(None);
    }

    let downhill = uphill.get_normal() * (if here > 0.0 { -1.0 } else { 1.0 });

    // The steep direction is tried first and the opposite second: in the middle of a hole the
    // field rises towards the hole's centre and the line being drawn is the other way.
    if let Some(point) = try_along(reading, (a, b, c), from, here, downhill)? {
        return Ok(Some(point));
    }

    try_along(reading, (a, b, c), from, here, -downhill)
}

/// Walks from a point of a triangle in one direction to the triangle's
/// boundary and bisects for the line if the field changes sign on the way; `None` when the line is
/// not reached in this triangle.
fn try_along(
    reading: &super::VertexFieldReading,
    (a, b, c): (Vector3, Vector3, Vector3),
    from: Vector3,
    here: f64,
    direction: Vector3,
) -> Result<Option<Vector3>, super::WorkError> {
    // How far the direction can be walked before leaving the triangle, by bisection on the
    // triangle (convex, so "inside" is one interval along any ray from an interior point).
    let mut inside = 0.0;
    let mut outside = 3.0 * ((b - a).length() + (c - a).length());
    for _ in 0..60 {
        let middle = 0.5 * (inside + outside);
        if in_triangle(a, b, c, from + (direction * middle)) {
            inside = middle;
        } else {
            outside = middle;
        }
    }

    let far = from + (direction * inside);
    if (reading.value_at_point(far)? < 0.0) == (here < 0.0) {
        return Ok(None);
    }

    Ok(Some(super::edge_root::root_on(from, far, reading)?))
}
