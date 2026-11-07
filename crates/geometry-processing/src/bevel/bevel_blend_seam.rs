//! The plane a BLEND has told one end of a run to stop on - the handover half of `BevelChainEnds`,
//! kept in its own file so the chain's end resolution can grow without it. Its sibling
//! [`super::bevel_corner_seam`] answers the same question for a corner patch.
//!
//! The two tables are only ever looked up here; they are taken as [`IndexMap`]s because the blend
//! set that builds them keeps its insertion order, which reaches the output.

use indexmap::IndexMap;
use vector_math::Vector3;

use super::BevelEndCap;

/// What a blend has told one end to stop on, as one plane whichever side of the corner the run is:
/// the cut hands over on the blend's own ring plane at that end of its rim, and the bead ends on
/// the face the blend read its rim from, which is that face's plane through the end's own vertex.
/// `None` where the blend has said nothing for this end (or the table is absent).
///
/// Read by the run's OWN edge as well as by the vertex, because a three-edge mixed corner has two
/// runs of one convexity ending at one vertex and the blend has a different answer for each of
/// them - see `BevelMixedBlendSet.ConvexHandovers`.
pub fn blend_seam(
    vertex: i32,
    own_edge: i32,
    position: Vector3,
    is_bead: bool,
    blend_handovers: Option<&IndexMap<(i32, i32), BevelEndCap>>,
    blend_capped_corners: Option<&IndexMap<(i32, i32), Vector3>>,
) -> Option<BevelEndCap> {
    if !is_bead {
        return blend_handovers
            .and_then(|handovers| handovers.get(&(vertex, own_edge)))
            .filter(|handover| handover.is_capped())
            .cloned();
    }

    blend_capped_corners
        .and_then(|corners| corners.get(&(vertex, own_edge)))
        .map(|&cap_normal| BevelEndCap::new(cap_normal, cap_normal.dot(position)))
}
