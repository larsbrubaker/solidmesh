//! Where a mixed corner's RUN-OUT meets a band standing on the WALL it turns against
//! ([`SurfaceRunOutMeetsAWallBand`]). Ports the record of
//! `Bevel/Surface/SurfaceRunOutMeetsAWallBand.cs`; its reading and its point accessor land with
//! the mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::SurfaceMixedCorner;

/// C# `SurfaceRunOutMeetsAWallBand` (a sealed class): where a mixed corner's run-out meets a band
/// standing on the wall it turns against, read as the radius out of the bead's axis at which that
/// band ENDS each column of the run-out.
///
/// **The band ends the column, exactly as the wall does.** A band laid on the wall takes the wall
/// away for its own setback and puts its strip in its place, so from the turn the two first meet
/// it is the BAND that stands nearest the axis and the column ends on it. One `max` of two radii is
/// the whole difference: the rows fall away in the same order and the column stays a prefix of the
/// one before it.
///
/// **The dual of a band on the cap.** A band the run-out crosses on the CAP ends the rows, each at
/// its own turn (`SurfaceRunOutMeetsTheBand`); a band on the WALL ends the columns, each at its own
/// radius. Measured on the lobed prism at `r = 3.756`: `o..o x..x w..w` - standing, then taken by
/// the band, then taken by the wall.
///
/// **One point per rail, in closed form.** A band whose edge runs along the bead's own axis has
/// rails parallel to it, each at a fixed radius `rho_i`, meeting the torus at one height
/// `z = r - sqrt(r^2 - (2r - rho_i)^2)`. The run-out is stationed AT those turns, so each crossing
/// is one vertex both surfaces are drawn to rather than two a rounding apart. And read against the
/// chords, not the arc: where a column ends is asked of the POLYLINE the rails make in the cap's
/// own plane, which is exactly on the strip's own quads.
#[derive(Debug)]
pub struct SurfaceRunOutMeetsAWallBand {
    /// The corner whose run-out this is.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The band's rails, flattened into the cap's own plane.
    pub flat_rails: Vec<Vector3>,
    /// The turn about the bead's axis each of those rails stands at.
    pub rail_turns: Vec<f64>,
    /// The turns past the bead at which the run-out's own columns end ON a rail of the band, in
    /// order. These are the stations the run-out has to be drawn at: a column ending anywhere else
    /// ends part way along a quad of the strip, which is a vertex only one of the two surfaces has.
    pub rail_turns_past_the_bead: Vec<f64>,
}
