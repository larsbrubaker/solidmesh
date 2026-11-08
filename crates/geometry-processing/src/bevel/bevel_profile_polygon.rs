//! The plane-polygon arithmetic a swept section's CROSS SECTION needs: which way it is wound
//! ([`signed_area`]), so every profile of a run can be wound the same way
//! ([`super::bevel_chain_sweep_sections`]).

use solidmesh_math::Vector2;

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
