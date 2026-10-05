//! The grid a LEANING one-crease mixed corner's patch is sewn over ([`SurfaceMixedCornerDualGrid`]).
//! Ports the record of `Bevel/Surface/SurfaceMixedCornerDualGrid.cs`; its reading and its grid
//! accessors land with the mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCornerDualCanal, SurfaceMixedCornerRows};

/// C# `SurfaceMixedCornerDualGrid` (a sealed class): the grid a LEANING one-crease mixed corner's
/// patch is sewn over - read, not emitted. Two end rings and a canal that runs past both of them:
/// the BEAD's own square section at station zero, the CUT's end ring along the far rail while that
/// strip lasts, and then the run-out, which owns every rail of its own and closes on a single
/// point.
///
/// **The same transpose a two-crease junction's grid is.** The canal turns about the CUT's centre
/// line, so the stations run along the cut's ring and each station's row runs from the ball's
/// contact on the wall out to its contact on the cut's tube (`SurfaceTwoCreaseJunctionRings` lays
/// the two-crease case the same way). What is different is only the far END: here it carries on
/// past the cut altogether until the ball dies on the crease the cap and the wall meet in.
///
/// **So the grid is in two halves and the join is exact.** Up to the cut strip's own end the
/// stations are the cut ring's own rails, read rather than solved, and the last rail of each row
/// is that rail itself. Past it the rows are the canal's own, sampled at the step the cut's ring
/// was already chorded at, because the blend a strip feeds into is not entitled to be finer than
/// the strip. The closing column is ONE point, kept as a station so the patch has a row to fan its
/// last band to.
pub struct SurfaceMixedCornerDualGrid {
    /// The rows the grid's stations are laid with, which it built and owns.
    pub rows: SurfaceMixedCornerRows,
    /// The canal the whole patch stands on.
    pub canal: Rc<SurfaceMixedCornerDualCanal>,
    /// The CUT's end ring, cut on the canal's own leaning plane, running from its rail on the face
    /// the two edges SHARE round to its rail on the cap.
    pub cut_ring: Vec<Vector3>,
    /// The turn each station of the grid stands at, from zero to the turn the blend dies at.
    pub stations: Vec<f64>,
    /// The BEAD's square end ring, from its contact on the wall out to its contact on the cut's
    /// tube.
    pub bead_ring: Vec<Vector3>,
}
