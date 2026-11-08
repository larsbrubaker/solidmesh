//! The one entry point for a whole part's fillet or chamfer: [`bevel_part`]. It routes a request
//! with any Point-mode group through the Point groups (`point_groups`, which hand the edge groups
//! to the surface construction first) and any other straight to the prologue and the surface
//! construction (`prologue::build_edges`).

use solidmesh_mesh::CancelToken;

use super::{
    point_groups, prologue, BevelPartError, BevelPartOutcome, BevelPartRequest, BevelProgressReport,
};
use crate::topology::BevelSelectionKind;

/// Bevels one part: every group of `request` built at its size in one pass where it fits, and
/// refused with a code - and, where one builds, the largest size that does - where it does not.
///
/// `cancel` is polled throughout; a cancelled build answers `Err(BevelPartError::Cancelled)`.
/// `progress` is told the build's phases ([`BevelProgress`](super::BevelProgress)) on one bar that
/// only moves forward, for the caller to word; `None` reports nothing.
///
/// A refusal is never an error: the outcome's rows say which groups built and why the others did
/// not, with no mesh where nothing built. The other errors mean the build could not run at all;
/// the rows to show for them are [`BevelPartOutcome::every_group_refused`].
pub fn bevel_part(
    request: BevelPartRequest,
    cancel: &CancelToken,
    progress: Option<BevelProgressReport>,
) -> Result<BevelPartOutcome, BevelPartError> {
    if request
        .groups
        .iter()
        .any(|group| group.selection_kind == BevelSelectionKind::Point)
    {
        point_groups::build_part(&request, cancel, progress)
    } else {
        prologue::build_edges(&request, cancel, progress)
    }
}
