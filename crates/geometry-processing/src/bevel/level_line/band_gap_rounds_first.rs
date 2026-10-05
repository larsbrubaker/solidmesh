//! The band built rounds first. Ports `Bevel/LevelLine/BandGap.RoundsFirst.cs` (a partial of the C#
//! static class `BandGap`): so far the kept triangles as index triples ([`kept`]); the build lands
//! with later packages.

use super::LevelLineMesh;

/// The mesh's live triangles as index triples, in triangle order.
pub fn kept(mesh: &LevelLineMesh) -> Vec<(i32, i32, i32)> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| {
            let face = mesh.triangles()[t as usize];
            (face.a, face.b, face.c)
        })
        .collect()
}
