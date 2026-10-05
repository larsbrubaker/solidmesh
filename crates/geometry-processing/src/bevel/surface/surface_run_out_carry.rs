//! How the run-out carries the patch's own last column PAST the end of the bead's rim
//! ([`SurfaceRunOutCarry`]). Ports the record of `Bevel/Surface/SurfaceRunOutCarry.cs`; its
//! reading and its carry land with the mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceMixedCornerRows};

/// C# `SurfaceRunOutCarry` (a sealed class): how the run-out carries the patch's own last column
/// past the end of the bead's rim - along the SPINE where the corner leans and rides the cap, and
/// rigidly about the bead's axis where it does not.
///
/// **One question asked in four places.** The run-out is more stations of the same turn, so what
/// it draws at each of them is the patch's last column carried there - where a row STANDS, and how
/// far out of the axis it stands there, which is what decides whether the wall has swallowed it.
/// Carried rigidly those radii are the same at every station; carried on the spine the ring
/// SHRINKS along the turn ([`SurfaceMixedCornerRows`]) and every station has its own.
///
/// **And the rigid carry is kept exactly as it was.** Told no rows - every corner that does not
/// lean, and the MITRE handover, whose grid is scheduled rather than laid on the spine - it is the
/// corner's own rigid turn and the one array of radii, bit for bit as before.
pub struct SurfaceRunOutCarry {
    /// The corner the run-out belongs to.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The rows the patch laid on the spine, or `None` where the column is turned rigidly. Shared
    /// with the patch that laid them.
    pub rows: Option<Rc<SurfaceMixedCornerRows>>,
    /// Where each row of the patch's last column stands - the seat the rigid turn carries.
    pub seats: Vec<Vector3>,
    /// Those rows' radii out of the axis, which a rigid turn does not change.
    pub where_they_stand: Vec<f64>,
}
