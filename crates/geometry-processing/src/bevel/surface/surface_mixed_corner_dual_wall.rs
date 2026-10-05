//! What the WALL gives up and gains under a leaning one-crease corner's patch - the face rail zero
//! runs across, made to end exactly where that rail runs. Ports
//! `Bevel/Surface/SurfaceMixedCornerDualWall.cs`, a static class, so only its record lands here:
//! where rail zero leaves one crease ([`SurfaceMixedCornerDualWallCrossing`]). The wall's own
//! reading lands with the mixed-corner package's functions.

use vector_math::Vector3;

/// C# `SurfaceMixedCornerDualWall.Crossing` (a private readonly struct): where rail zero leaves
/// one crease - the two stations it happens between, and the point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceMixedCornerDualWallCrossing {
    /// The last station standing on the side the walk started from.
    pub before: i32,
    /// The first standing past it - the neighbour of
    /// [`SurfaceMixedCornerDualWallCrossing::before`].
    pub after: i32,
    /// Where between the two rail zero meets the crease's own line.
    pub at: Vector3,
}
