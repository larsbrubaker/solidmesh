//! The open fill's sunk arc. Ports `Bevel/LevelLine/OpenGapFill.SunkArc.cs` (a partial of the C#
//! static class `OpenGapFill`): so far a triangle's doubled area ([`area2`]) and the planes of a
//! side's removed triangles ([`planes`]); the arc lands with later packages.

use vector_math::Vector3;

use super::{LevelLineMesh, LevelLineTriangle};

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
