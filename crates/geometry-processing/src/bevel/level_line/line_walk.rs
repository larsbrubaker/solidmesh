//! The walk along the level line. Ports `Bevel/LevelLine/LineWalk.cs` (a C# static class): so far
//! whether a point stands in a triangle ([`in_triangle`]); the walk lands with later packages.

use vector_math::Vector3;

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
