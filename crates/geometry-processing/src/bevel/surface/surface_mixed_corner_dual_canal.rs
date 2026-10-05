//! A LEANING one-crease mixed corner's canal, read WHOLE ([`SurfaceMixedCornerDualCanal`]).
//! Ports the record of `Bevel/Surface/SurfaceMixedCornerDualCanal.cs`; its reading, the turn it
//! dies at and its row accessors land with the mixed-corner package's methods.

use std::rc::Rc;

use super::{SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerDualFrame};

/// C# `SurfaceMixedCornerDualCanal` (a sealed class): the dual's void ball rolling from the bead's
/// own end ring round until it dies on the crease the cap and the wall meet in - which is further
/// than the cut's own strip goes, and is the whole of why this corner needed a reading of its own.
///
/// **The canal does not stop where the cut does.** Measured against the Onshape control of
/// `NoseInNotchTestMeshes`, the blend KEEPS TURNING past the cut strip's end - 112.62 degrees to
/// 158.5036 at `r = 1` - and every vertex out there stands one radius off the SAME spine
/// (`BevelTopCenterFaceRegressionTests`). So the run-out is this canal turned further, and what
/// this adds to the frame is only WHERE it stops and WHAT bounds each row once the cut's strip has
/// ended.
///
/// **Where it stops.** The ball keeps circling until its contact on the WALL reaches an
/// UNSELECTED edge - [`SurfaceMixedCornerDualCanal::crease`] - at
/// [`SurfaceMixedCornerDualCanal::dies_at`], where the ball's centre stands exactly one radius from
/// the crease's LINE, so it touches the sharp edge at a point and the blend fades out instead of
/// cutting a step. The crease needs no finding: it is where the cap's plane and the wall's plane
/// meet, both through the corner.
///
/// **What bounds a row, and it changes half way.** While the cut's strip is still there a row ends
/// on that strip's tube; once it has ended the row ends where the ball's ring crosses the CAP's
/// plane. The two agree exactly at the cut strip's own end turn, so the boundary is continuous,
/// and the row's extent reaches zero exactly at the turn the blend dies at - the last column is a
/// POINT.
#[derive(Debug)]
pub struct SurfaceMixedCornerDualCanal {
    /// The corner this was read off, shared with every other reading of it.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The canal itself - the frame the whole of this corner stands on.
    pub dual: Rc<SurfaceMixedCornerDualFrame>,
    /// The edge the cap and the wall meet in, from the corner outwards - the unselected edge the
    /// blend runs out onto and dies on.
    pub crease: SurfaceLine,
    /// The turn the blend dies at, where the ball's contact on the wall reaches the crease.
    pub dies_at: f64,
}
