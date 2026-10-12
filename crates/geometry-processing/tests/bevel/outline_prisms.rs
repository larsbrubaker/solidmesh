//! Ports of `LevelLineTestGeometry.PrismOnOutline` (`LevelLine/LevelLineTestGeometry.Solids.cs`)
//! and `LevelLineTestGeometry.DistanceToSurface` (`LevelLine/LevelLineTestGeometry.cs`): a prism
//! on a star-shaped outline, and how far a point stands from a triangle list. Used by
//! `end_region_shapes_tests.rs`.

#![allow(dead_code)]

use vector_math::{Vector2, Vector3};

use crate::end_face_measures::SolidWithRim;

/// C# `PrismOnOutline(outline, starPoint, height = 4)`: a prism on a counter-clockwise outline in
/// xy, from z = 0 to `height`, capped top and bottom by a fan from `star_point`, which must see
/// every corner of the outline. Each wall is one quad of two triangles; the top rim, in outline
/// order, is handed back as the chain a rim pick takes.
pub fn prism_on_outline(outline: &[Vector2], star_point: Vector2, height: f64) -> SolidWithRim {
    let n = outline.len();

    let mut positions = Vec::new();
    let mut top = Vec::new();
    let mut bottom = Vec::new();
    for p in outline {
        top.push(positions.len() as i32);
        positions.push(Vector3::new(p.x, p.y, height));
    }

    for p in outline {
        bottom.push(positions.len() as i32);
        positions.push(Vector3::new(p.x, p.y, 0.0));
    }

    let centre = star_point;
    let top_centre = positions.len() as i32;
    positions.push(Vector3::new(centre.x, centre.y, height));
    let bottom_centre = positions.len() as i32;
    positions.push(Vector3::new(centre.x, centre.y, 0.0));

    let mut indices = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((top_centre, top[i], top[j]));
        indices.push((bottom_centre, bottom[j], bottom[i]));
        indices.push((top[i], bottom[i], bottom[j]));
        indices.push((top[i], bottom[j], top[j]));
    }

    (positions, indices, top)
}

/// C# `DistanceToSurface`: how far a point stands from the nearest of the given triangles.
pub fn distance_to_surface(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    point: Vector3,
) -> f64 {
    let mut least = f64::INFINITY;
    for &(ia, ib, ic) in indices {
        least = least.min(distance_to_triangle(
            positions[ia as usize],
            positions[ib as usize],
            positions[ic as usize],
            point,
        ));
    }

    least
}

fn distance_to_triangle(a: Vector3, b: Vector3, c: Vector3, point: Vector3) -> f64 {
    // The foot of the perpendicular in barycentric coordinates, then clamped onto the triangle by
    // taking the least distance to its three edges when that foot falls outside.
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    let area = normal.length();
    if area > 0.0 {
        let foot = point - (normal * (normal.dot(point - a) / (area * area)));
        let u = normal.dot((b - foot).cross(c - foot)) / (area * area);
        let v = normal.dot((c - foot).cross(a - foot)) / (area * area);
        let w = 1.0 - u - v;
        if u >= 0.0 && v >= 0.0 && w >= 0.0 {
            return (point - foot).length();
        }
    }

    f64::min(
        distance_to_segment(a, b, point),
        f64::min(
            distance_to_segment(b, c, point),
            distance_to_segment(c, a, point),
        ),
    )
}

fn distance_to_segment(a: Vector3, b: Vector3, point: Vector3) -> f64 {
    let along = b - a;
    let squared = along.length_squared();
    let t = if squared > 0.0 {
        (along.dot(point - a) / squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (point - (a + (along * t))).length()
}
