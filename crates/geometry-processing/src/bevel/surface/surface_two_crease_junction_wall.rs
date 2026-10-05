//! What the WALL gives up under a two-crease junction's patch, and what it is owed back
//! (`Bevel/Surface/SurfaceTwoCreaseJunctionWall.cs`, a static class): the one face the patch's row
//! zero runs across, made to end exactly where that row runs - two bites and one fill, and no
//! repair. Only its nested record lands here, where row zero leaves one crease
//! ([`SurfaceTwoCreaseJunctionWallCrossing`]); the bites and the fill land with the two-crease
//! junction package.

use vector_math::Vector3;

/// C# `SurfaceTwoCreaseJunctionWall.Crossing` (a private readonly struct): where row zero leaves
/// one crease - the two stations it happens between, and the point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceTwoCreaseJunctionWallCrossing {
    /// The last station standing on the BEAD's side of the crease.
    pub before: i32,
    /// The first standing on the notch side - the neighbour of
    /// [`SurfaceTwoCreaseJunctionWallCrossing::before`].
    pub after: i32,
    /// Where between the two row zero meets the crease's own line.
    pub at: Vector3,
}
