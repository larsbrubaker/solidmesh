//! What the WALL gives up and gains under a leaning one-crease corner's patch - the face rail zero
//! runs across, made to end exactly where that rail runs. Ports
//! `Bevel/Surface/SurfaceMixedCornerDualWall.cs`, a static class, so only its record lands here:
//! where rail zero leaves one crease ([`SurfaceMixedCornerDualWallCrossing`]), with the bead's foot
//! on its crease ([`try_the_foot`]). The wall's own reading lands with a later package.

use vector_math::Vector3;

use super::SurfaceMixedCorner;
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

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

/// The bead's own FOOT on its crease: where the plane that bead's band stops on crosses the crease
/// line. A point the walkback already left there, so it is added to the pool, which welds it onto
/// that vertex rather than beside it.
pub fn try_the_foot(corner: &SurfaceMixedCorner) -> Result<Vector3, BevelAdvisory> {
    let away = SurfaceMixedCorner::away_from_the_corner(&corner.bead, corner.corner);
    let stops_at = corner.bead_stops_at;
    let leaning = away.dot(stops_at.inward);

    if leaning.abs() < 1e-9 {
        // DEFENSIVE: a band that stops on a plane its own edge runs along stops nowhere.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![BevelAdvisoryArg::Vector3(corner.corner)],
        ));
    }

    Ok(corner.corner + (away * ((stops_at.point - corner.corner).dot(stops_at.inward) / leaning)))
}
