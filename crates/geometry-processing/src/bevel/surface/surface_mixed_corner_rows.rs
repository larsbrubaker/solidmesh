//! The mixed-corner patch's rows laid ALONG THE SPINE ([`SurfaceMixedCornerRows`]). Ports the
//! record of `Bevel/Surface/SurfaceMixedCornerRows.cs`; its two readings and its rail accessor
//! land with the mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::SurfaceMixedCornerSpine;

/// How far the row at a turn RUNS, as an angle at the ball's centre (C# `Func<double, double>`).
pub type SurfaceMixedCornerRowRunsTo = Rc<dyn Fn(f64) -> f64>;

/// C# `SurfaceMixedCornerRows` (a sealed class): the patch's rows, laid ALONG THE SPINE - the ring
/// at station zero carried from station to station on the rolling ball rather than turned rigidly
/// about an axis.
///
/// **Which canal, though - the CALLER's to say.** Told a corner, these rows are laid on the
/// CAP-RIDING canal: exact for a square corner, and a sub-band of an oblique one, which the build's
/// own gate (`BevelAdvisoryCode::SurfaceObliqueMixedCornerRowsNotBuilt`) still holds back. Told a
/// SPINE instead, the same arithmetic lays the dual's rows - which is what a two-crease junction's
/// patch is made of (`SurfaceTwoCreaseJunctionRings`).
///
/// **What a station's ring is.** A canal of CONSTANT radius meets each of its balls in the circle
/// of radius `r` lying in the plane NORMAL TO THE SPINE'S TANGENT. The two contacts the ball is
/// defined by - `TContactAt` at `+u` and `BeadContactAt` at `-e(phi)` - are the two ENDS of the
/// station's own ring, and the ring is the arc between them.
///
/// **And the arc SHRINKS along the spine, which is the whole of what leaning costs.** On the
/// oblique fixture 112.62 degrees at turn zero closing to 66.91 at the end of the turn. A rigid
/// turn keeps the first all the way round, which lifts its own outer rail off the cap and off the
/// bead's cylinder alike. So each rail is carried at its FRACTION of the sweep: rail `p` sits at
/// `psi_p` of the ring at turn zero and at `psi_p * sweep(phi) / sweep(0)` at every station after.
///
/// **A square cap is the rigid turn, rail for rail** - Rodrigues written out - for a ring of ANY
/// shape, so nothing that builds today moves off the arithmetic's last bits.
pub struct SurfaceMixedCornerRows {
    /// The spine the rows are carried along.
    pub spine: SurfaceMixedCornerSpine,
    /// The way from the ball's centre to the face it rides.
    pub towards_the_plate: Vector3,
    /// The ring's sweep at station zero, which every later station's fraction is taken of.
    pub sweep_at_turn_zero: f64,
    /// How far the row at a turn RUNS, or `None` where that is simply the ball's own sweep.
    pub runs_to: Option<SurfaceMixedCornerRowRunsTo>,
    /// Each rail as its angle round the station's ring, its radius there, and how far it stands
    /// off that ring's own plane.
    pub rails: Vec<(f64, f64, f64)>,
}
