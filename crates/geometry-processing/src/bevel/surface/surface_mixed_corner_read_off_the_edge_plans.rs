//! Reading a mixed corner's frame off the two edge plans that meet there
//! (`Bevel/Surface/SurfaceMixedCornerReadOffTheEdgePlans.cs`, a static class: free functions
//! here). The questions the reading is made of land here first: which region the two edges
//! share, whether the bead rests on a crease of it, and which side's normal and tangent line to
//! read. The reading itself lands with a later package.

use vector_math::Vector3;

use super::{SurfaceEdgePlan, SurfaceLine};

/// Whether the bead's profile rests on a CREASE of the face the two edges share - the one place
/// the corner's own tangency is a line the bead touches along rather than a facet it is tangent
/// to. Both halves have to hold: a bead that rests on a crease of the wall NOBODY PICKED is
/// tangent to the shared face as it always was, and what a crease there moves is only where the
/// turn ENDS, which the wall's own tangent line already says.
pub fn rests_on_the_crease_of_the_shared_face(bead: &SurfaceEdgePlan, shared_key: i32) -> bool {
    bead.wall_section
        .as_ref()
        .is_some_and(|section| section.rests_on_the_crease)
        && (if bead.left_band.is_some() {
            bead.left_region_key()
        } else {
            bead.right_region_key()
        }) == shared_key
}

/// The same question asked of a convex edge rather than of a region key: whether the bead rests
/// on a crease of the face IT and `other` share. What the cut edge's own walk has to ask
/// (`SurfaceMixedCornerCutEdge`), so that the two readings agree about which corners are crease
/// landings and which are the ordinary ones.
pub fn rests_on_the_crease_of_the_face_shared_with(
    bead: &SurfaceEdgePlan,
    other: &SurfaceEdgePlan,
) -> bool {
    try_name_the_shared_region(other, bead)
        .is_some_and(|shared_key| rests_on_the_crease_of_the_shared_face(bead, shared_key))
}

/// The one region both plans name, or `None` where they name none or both. Two edges of one
/// crease each that meet at a vertex share exactly one face; anything else is a corner this
/// reading does not describe.
pub fn try_name_the_shared_region(cut: &SurfaceEdgePlan, bead: &SurfaceEdgePlan) -> Option<i32> {
    let mut shared_key = 0;
    let mut found = 0;

    for mine in [cut.left_region_key(), cut.right_region_key()] {
        for theirs in [bead.left_region_key(), bead.right_region_key()] {
            if mine == theirs {
                shared_key = mine;
                found += 1;
            }
        }
    }

    (found == 1).then_some(shared_key)
}

/// The outward normal of the side of a planned edge that is NOT the shared region - the cap
/// face, for the cut. The shared side is not read this way: what the corner needs there is where
/// the bead really touches that face, which on a rolled one is the facet it rolled to, so it
/// comes off the bead's own tangent line instead.
pub fn outward_of_the_other_side(plan: &SurfaceEdgePlan, shared_key: i32) -> Vector3 {
    let left_is_shared = plan.left_region_key() == shared_key;

    if left_is_shared {
        plan.edge.right_normal()
    } else {
        plan.edge.left_normal()
    }
    .get_normal()
}

/// The line one side of a planned edge is walked back to: the side that IS the shared region
/// (`shared`), or the side that is not. On a rolled edge the wall's line lies in the facet the
/// profile rolled to rather than in the chain edge's own facet, which is the whole reason the
/// turn is read off it.
pub fn tangent_line_of(plan: &SurfaceEdgePlan, shared_key: i32, shared: bool) -> SurfaceLine {
    let left_is_shared = plan.left_region_key() == shared_key;

    if left_is_shared == shared {
        plan.left_tangent_line
    } else {
        plan.right_tangent_line
    }
}
