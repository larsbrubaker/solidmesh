//! Where the bands stop at a JUNCTION - three picked edges at one vertex with one crease among them
//! (`SurfaceMixedJunction`) - which is the one corner whose rules are not the plan's own
//! (a free function). Its own file
//! rather than the plan's: the plan's job is the setbacks and the chain's own corners, and this is a
//! reading of a corner that spans three chains.

use std::rc::Rc;

use super::{SurfaceBandEnd, SurfaceEdgePlan, SurfaceMixedCorner};

/// What bounds one side of one edge's band at this corner, added to `ends` (the band ends
/// gathered so far for this side of this edge). `arms` is the corner as each arm that bounds THIS
/// edge read it - one arm on a chain the other two meet at the end of, both arms on the chain shared
/// with each of them; `region_key` is which of the edge's two faces the band lies on.
///
/// **One rule, read off the FACE.** A band is bounded at this corner by the edge's own stopping
/// plane ([`SurfaceMixedCorner::band_end_for`]) unless the face it lies on is the face some arm
/// SHARES - on a shared face the other edge's own band is what ends it. Unless that arm hands over
/// on a MITRE ring, where both bands have to stop on it
/// ([`SurfaceMixedCorner::both_bands_stop_on_the_shared_face`]).
///
/// **And that one rule is both junctions.** At a ONE-crease junction the cut sees its own arm only
/// and stops on its cap; the bead sees both arms and is bounded on neither. At a TWO-crease junction
/// the roles turn over.
///
/// **The end's own run-out and trim are not consulted, which is not an oversight:** at a junction
/// what closes the chain is the corner rather than a face beyond it, and the faces the junction
/// leaves unbounded end on the crease the bead is drawn on.
///
/// `arms` is never empty where the plan calls this; an empty list adds nothing.
pub fn add(
    ends: &mut Vec<SurfaceBandEnd>,
    arms: &[Rc<SurfaceMixedCorner>],
    edge: &SurfaceEdgePlan,
    region_key: i32,
) {
    for arm in arms {
        if arm.shared_region_key != region_key {
            continue;
        }

        if arm.both_bands_stop_on_the_shared_face() {
            ends.push(arm.band_end_for(edge));
        }

        return;
    }

    // Nobody's shared face: the edge's own stopping plane, which every arm of the corner reads the
    // same for this edge - a cut's handover, a bead's own square section.
    if let Some(first) = arms.first() {
        ends.push(first.band_end_for(edge));
    }
}
