//! Flow rung 2: the strip clipped on a GROWING neighbour's planes, and the neighbour's caps. Rows
//! laid whole across a virtual rail overlap the neighbour that stands past the reached crease;
//! each of its faces on that crease clips the strip, the strip where EVERY face's distance is below
//! zero is dropped, and the neighbour grows a cap in each face's plane. Ports
//! `Bevel/LevelLine/StripClip.cs` (a C# static class, so a module of functions); the clip itself
//! lands with its method packages.

use super::LevelLineTriangle;

/// The edge `low - high` in the direction `face` traverses it.
pub fn directed(face: LevelLineTriangle, low: i32, high: i32) -> (i32, i32) {
    for corner in 0..3 {
        if face.corner(corner) == low && face.corner((corner + 1) % 3) == high {
            return (low, high);
        }
    }

    (high, low)
}
