//! A side consumed to its rim. Ports `Bevel/LevelLine/ConsumedToTheRim.cs` (a C# static class): so
//! far whether an edge lies on one of the rims ([`lies_on_a_rim`]); the reading lands with later
//! packages.

use vector_math::Vector3;

/// True when both `p` and `q` stand within `snap` of one rim segment.
pub fn lies_on_a_rim(rims: &[(Vector3, Vector3)], p: Vector3, q: Vector3, snap: f64) -> bool {
    rims.iter()
        .any(|&rim| distance_to_segment(p, rim) <= snap && distance_to_segment(q, rim) <= snap)
}

/// How far `p` stands from the segment.
pub fn distance_to_segment(p: Vector3, segment: (Vector3, Vector3)) -> f64 {
    let along = segment.1 - segment.0;
    let t = if along.length_squared() > 0.0 {
        ((p - segment.0).dot(along) / along.length_squared()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (segment.0 + (along * t))).length()
}
