//! The corner fill's end-ray trim (`Bevel/LevelLine/CornerFill.EndRayTrim.cs`, a partial of the C#
//! static class `CornerFill`, whose piece type is in `corner_fill.rs`). Only the convexity reading
//! at a run's end ([`is_convex_at`]) lands here so far; the trim itself lands with a later package.

use super::{GapLoop, LevelLineMesh};

/// C# `IsConvexAt`: whether the chain a side run borders is convex at the run's end - the other
/// rail's end (`across`) stands behind the face kept on the run's last edge (the reading
/// `EndFaceFill.IsConcave` takes at a run's middle). A run of fewer than two vertices has no last
/// edge (C# would throw reading one) and reads as not convex.
pub fn is_convex_at(mesh: &LevelLineMesh, run: &GapLoop, across: i32) -> bool {
    let count = run.vertices.len();

    if count < 2 {
        return false;
    }

    let (a, b) = (run.vertices[count - 2], run.vertices[count - 1]);

    for &t in mesh.triangles_on(a, b) {
        if mesh.is_removed(t) {
            continue;
        }

        let face = mesh.triangles[t as usize];
        let a0 = mesh.position(face.a);
        let n = (mesh.position(face.b) - a0).cross(mesh.position(face.c) - a0);

        if n.length() > 0.0 {
            return n.dot(mesh.position(across) - mesh.position(b)) <= 0.0;
        }
    }

    false
}
