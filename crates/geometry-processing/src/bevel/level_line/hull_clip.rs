//! The HULL mode of the strip clip: where a concave bead's rows are run on past the part itself, no
//! neighbour face stands there to grow, so the part's own SUPPORTING planes bound it instead - every
//! plane of a kept face with the whole kept surface on its inner side - and every piece of the
//! strip OUTSIDE any of them is dropped. Ports the nested type of `Bevel/LevelLine/HullClip.cs` (a
//! C# static class): one supporting plane ([`HullClipPlane`]) and the face-directed edge
//! ([`directed`]). The clip itself and its thread-static outcome land with later packages.

use std::collections::HashSet;

use vector_math::Vector3;

use super::LevelLineTriangle;

/// C# `HullClip.Plane`: a supporting plane - a point on it, its outward normal, and the vertices
/// on it by bookkeeping. A vertex of the kept surface in the plane, or one minted on an edge
/// crossing it, is ON the plane by bookkeeping; every other vertex reads its computed distance.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HullClipPlane {
    /// A point on the plane.
    pub origin: Vector3,
    /// The plane's outward unit normal.
    pub normal: Vector3,
    /// The vertices ON the plane by bookkeeping, read as distance zero.
    pub on: HashSet<i32>,
}

/// The edge `low - high` in the direction `face` traverses it.
pub fn directed(face: LevelLineTriangle, low: i32, high: i32) -> (i32, i32) {
    for corner in 0..3 {
        if face.corner(corner) == low && face.corner((corner + 1) % 3) == high {
            return (low, high);
        }
    }

    (high, low)
}
