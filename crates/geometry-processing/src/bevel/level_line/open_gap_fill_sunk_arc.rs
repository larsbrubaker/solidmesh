//! The open fill's sunk arc. Ports `Bevel/LevelLine/OpenGapFill.SunkArc.cs` (a partial of the C#
//! static class `OpenGapFill`): so far a triangle's doubled area ([`area2`]), the planes of a side's
//! removed triangles ([`planes`]), whether a spoke stands past an end ([`past_an_end`]) and whether
//! a step turns into the left side's nearest plane ([`concave`]); the arc lands with later
//! packages.

use vector_math::Vector3;

use super::{ChainField, LevelLineMesh, LevelLineTriangle, OpenGapFillOpenRail};

/// Twice a triangle's area.
pub fn area2(mesh: &LevelLineMesh, t: LevelLineTriangle) -> f64 {
    (mesh.position(t.b) - mesh.position(t.a))
        .cross(mesh.position(t.c) - mesh.position(t.a))
        .length()
}

/// The planes - centroid and unit normal - of the side's REMOVED triangles with area, in triangle
/// order: the faces the band took out of the side, which the arc is read against.
pub fn planes(mesh: &LevelLineMesh, side: i32) -> Vec<(Vector3, Vector3)> {
    let mut planes = Vec::new();
    for t in 0..mesh.triangle_count() {
        let triangle = mesh.triangles()[t as usize];
        if !mesh.is_removed(t) || triangle.tag != side {
            continue;
        }

        let a = mesh.position(triangle.a);
        let normal = (mesh.position(triangle.b) - a).cross(mesh.position(triangle.c) - a);
        if normal.length() > 0.0 {
            planes.push((
                (a + mesh.position(triangle.b) + mesh.position(triangle.c)) / 3.0,
                normal.get_normal(),
            ));
        }
    }

    planes
}

/// True when a spoke's mean extended parameter stands past either end of the chain.
pub fn past_an_end(
    chain: &ChainField,
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    spoke: (i32, i32),
) -> bool {
    let mean = (left.parameters[spoke.0 as usize] + right.parameters[spoke.1 as usize]) / 2.0;
    mean < 0.0 || mean > chain.length
}

/// True when the step from `p` to `q` runs into the left side's plane nearest `p` (the first of a
/// tie, as C#'s `MinBy`); false with no planes.
pub fn concave(left_planes: &[(Vector3, Vector3)], p: Vector3, q: Vector3) -> bool {
    let mut nearest: Option<((Vector3, Vector3), f64)> = None;
    for &plane in left_planes {
        let reach = (plane.0 - p).length_squared();
        if nearest.is_none_or(|(_, best)| reach < best) {
            nearest = Some((plane, reach));
        }
    }

    nearest.is_some_and(|((_, normal), _)| normal.dot(q - p) > 0.0)
}
