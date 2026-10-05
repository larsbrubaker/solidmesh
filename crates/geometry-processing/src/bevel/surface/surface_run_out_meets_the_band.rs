//! Where a mixed corner's RUN-OUT and another chain's BAND land on one another
//! ([`SurfaceRunOutMeetsTheBand`]), read as the turns the run-out's own rows and its foot leave the
//! band's material at. Ports the fields of `Bevel/Surface/SurfaceRunOutMeetsTheBand.cs`; the
//! reading and its questions land with the run-out crossing package as further `impl` blocks.

use std::rc::Rc;

use vector_math::Vector3;

use super::SurfaceStrip;

/// C# `SurfaceRunOutMeetsTheBand`: a run-out and a band crossing, read per row of the run-out.
///
/// **One arc, swept two ways.** The band's strip is the r-arc swept along a straight line and the
/// run-out's rings are the same r-arc turned about the bead's axis, so at every height the two
/// carry the same offset from their own tangent line: row j stands `d_j` in from the touch point,
/// which puts it `rho_j = 2r - d_j` out of the axis, and rail i stands `d_i` in from the band's
/// foot. The two surfaces cross where those offsets meet, which per ROW is simply the turn at which
/// that row's circle reaches the vertical plane its own rail stands in.
///
/// **Rows leave from the cap upward**, a PREFIX of the column, which is what lets the columns stay
/// prefixes of each other. **And the foot last**: past the turn the run-out's trim point enters the
/// section the band's end cut takes, it is standing on face the band has already taken, so that
/// turn is where it dies. **Paired by where they stand, never by index**: rows and rails are
/// matched by height and offset, and a pair that does not match to the part's own ruler is refused
/// by name rather than sewn to the nearest rail.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceRunOutMeetsTheBand {
    /// Per row of the run-out's seed, the turn past the bead's own last at which that row leaves
    /// the band's material; infinity where that row's circle never reaches the band at all.
    pub crosses_at: Vec<f64>,
    /// Per row of the seed, whether the band already has it at turn zero.
    pub the_band_has_it_at_the_seed: Vec<bool>,
    /// The band's strip, whose rails the crossings stand on.
    pub band: Rc<SurfaceStrip>,
    /// The band's profile, rail by rail, read at the strip's own origin.
    pub profile: Vec<Vector3>,
    /// The band's own sharp corner - the edge it was taken off - in that same section.
    pub sharp_corner: Vector3,
    /// One of the two axes the band's section is read in, square to the way its rails run.
    pub across: Vector3,
    /// The other axis of the band's section; see [`SurfaceRunOutMeetsTheBand::across`].
    pub up: Vector3,
}
