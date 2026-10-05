//! Closes a JUNCTION - two cuts and a bead at one vertex - with the single torus patch between the
//! two cuts' end rings, and the sector it takes out of the cap face
//! (`Bevel/Surface/SurfaceMixedJunctionPatch.cs`, a static class). Only its nested record lands
//! here, one strip standing at the junction ([`SurfaceMixedJunctionPatchArrival`]); the patch's
//! construction lands with the mixed-junction package.

use std::rc::Rc;

use super::SurfaceStrip;

/// C# `SurfaceMixedJunctionPatch.Arrival` (a readonly struct): one strip standing at the junction,
/// as the strip ends hand it over.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunctionPatchArrival {
    /// The strip itself.
    pub strip: Rc<SurfaceStrip>,
    /// Which edge of the whole build it belongs to, which is how its seams are found.
    pub edge_index: i32,
    /// Whether the corner is that strip's FINISH rather than its start.
    pub ends_here: bool,
}
