//! A mixed corner that hands over on a MITRE ring the chain already had
//! (`Bevel/Surface/SurfaceMixedCornerHandsOverAtTheMitre.cs`, a static class: free functions
//! here). The reading itself (`TryRead`) lands with the round-2 package; this module holds the
//! plane the band stops on where an edge meets its neighbour.

use super::{surface_plan_band_ends, SurfaceBandEnd, SurfaceEdgePlan};

/// The plane `edge`'s band stops on where it meets `neighbour`: the chain's own mitre, written
/// the way the plan writes it so that the ring and the band stop on one plane rather than on two
/// that nearly agree. `ahead` is whether the neighbour is the edge the chain LEAVES this one for.
pub fn mitre_with(
    planned: &[SurfaceEdgePlan],
    edge: usize,
    neighbour: usize,
    ahead: bool,
) -> SurfaceBandEnd {
    // Through the CORNER, and not through the crossing of the two tangent lines an ordinary
    // mitre's plane stands at (SurfaceMitreSeam): this is a MIXED corner, where the convexity
    // turns over and the two bands do not meet on a chain mitre at all. Measured: reading the seam
    // here moves the hand-over ring 3.8 mm along one of its own chords, from (28.91, -2.54, 20) to
    // (25.55, -0.83, 20).
    let here = planned[edge].edge.direction();
    let there = planned[neighbour].edge.direction();

    if ahead {
        surface_plan_band_ends::band_end_where_the_chain_leaves_along(
            planned[edge].to(),
            here,
            there,
        )
    } else {
        surface_plan_band_ends::band_end_where_the_chain_arrives_along(
            planned[edge].from(),
            there,
            here,
        )
    }
}
