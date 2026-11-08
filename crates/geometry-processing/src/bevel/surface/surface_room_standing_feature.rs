//! The one reading of "a feature is STANDING on this region here, rather than the region ending
//! here": a rim edge whose far face leans back OUT over the region, so that the material behind
//! that rim is hanging above the landing plane instead of walling it off.
//!
//! Shared by the two halves of the same fact. The room reading (`SurfaceRoomMeasure`) must not take
//! such a rim for a wall - the band runs UNDER the overhang and on across the region, and what it
//! uncovers there is the feature's own root cross-section, an island to be closed rather than a
//! place the face runs out. The walkback closes that island (`SurfaceStandingFeatureIsland`), which
//! is what makes the room reading honest rather than optimistic.
//!
//! Both halves of the test earn their place. CONCAVE, because a convex rim is the region running
//! off an edge and out of the solid - the thing the room really is bounded by. And the far face
//! RISING out of the region: a wall square to the landing plane (a rib's foot, a boss's foot, the
//! crease where two slopes meet) has nothing hanging over the region at all, so no band can pass
//! beneath it and no cross-section is uncovered. Those stay walls here and stay refused by
//! `surface_walkback_bands::try_check_nothing_is_standing_in_the_band` further on.
//!
//! **The two exclusions are part of the predicate, not of either caller.** What the island will not
//! close must go on bounding the room, or the room grants a setback for a root nothing closes and
//! the build comes back as an open rim instead of as a size. So the edges the pass is BEVELING and
//! the features that MEET the chain at a corner are named out here, once, and both halves get the
//! same answer. That second exclusion is read along the crease's own LINE rather than edge by edge:
//! a boolean chords a crease wherever another solid crossed it, and a rim that reaches the chain
//! only through its collinear neighbours meets it just the same.

use std::collections::HashSet;

use solidmesh_math::{math_helper, Vector3};

use crate::topology::{EdgeConvexity, FaceGroup, FeatureEdge};

/// How much of the region's own normal the far face has to carry before it is leaning out over the
/// region rather than standing square on it: the sine of the same angle at which the pass stops
/// telling two faces apart at all ([`FeatureEdge::default_flat_angle_degrees`], about half a
/// degree). A wall built square is square, and a wall drawn a fraction off square is a wall too -
/// reading one of those as a feature to pass under would have the band shave out from beneath it.
/// Nothing near the threshold is a feature in practice: the shapes this is for lean out by tenths
/// (0.39 for the nose in the chevron notch, 0.58 for a fin), and a wall stands at 0. (A function
/// because `sin` is not `const`.)
fn rises_out_of_the_plane() -> f64 {
    math_helper::degrees_to_radians(FeatureEdge::default_flat_angle_degrees()).sin()
}

/// How straight two rim edges have to run for the corner of the mesh between them to be no corner
/// of the SHAPE: a millionth off parallel, which is inside what a boolean's own arithmetic leaves
/// when it cuts one straight crease at a point along it. Nothing looser is wanted - two creases
/// that really meet at a shallow angle are two creases, and letting one off for the other's corner
/// would withhold a face nothing closes.
const ALONG_THE_SAME_LINE: f64 = 1.0 - 1e-6;

/// Whether `rim` is a feature STANDING on `region` - concave, with its far face leaning out over
/// the region - rather than a wall bounding it. `beveled_mesh_edges` are the mesh edges this pass
/// is beveling, which are never features; `chain_vertices` their end vertices - a feature meeting
/// one is the corner's business.
pub fn stands_on(
    region: &FaceGroup,
    rim: &FeatureEdge,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
) -> bool {
    if rim.convexity != EdgeConvexity::Concave {
        return false;
    }

    if beveled_mesh_edges.contains(&rim.mesh_edge_index) {
        // An edge this pass is BEVELING is not a feature to pass under: the strip built along it
        // is what stands there, and the band is one side of that strip. A bead's own concave chain
        // edge reads as standing on the face it beads by geometry alone - its far face does lean
        // out over this one - so it has to be named out.
        return false;
    }

    if chain_vertices.contains(&rim.start_vertex) || chain_vertices.contains(&rim.end_vertex) {
        // Nor is a feature that MEETS the chain at a corner. The band is deepest at its own ends,
        // so anything leaving a chain end vertex stands inside the footprint by construction, and
        // what happens there is the corner's business - the end cut, the blend, the trim by the
        // face beyond - and not an island's (SurfaceBeadPastAKnifeEdgeTipTests).
        return false;
    }

    let Some((landing, feature)) = try_the_two_sides(region, rim) else {
        // Neither face is the region's, so this rim is not that region's to read.
        return false;
    };

    if feature.dot(landing) <= rises_out_of_the_plane() {
        return false;
    }

    // The corner exclusion again, asked of the CREASE this time and not of the mesh edge, because a
    // boolean chords a crease wherever another solid's outline crossed it: "Center Flows
    // Through.mcx" arrives with the nose's upper right crease in two, and only the far piece
    // touches the corner it dies in. A crease is one crease however it is chorded.
    //
    // LAST of the four because it is the only one that costs a walk of the whole rim: every test
    // here is a refusal, so the answer does not depend on their order.
    !meets_the_chain_along_its_line(region, rim, chain_vertices)
}

/// Whether a wall of `region` ENDS at `corner` only because a standing feature overhangs the
/// region there - in which case the wall's far end is no limit on how far a band may be set back,
/// because the wall itself carries on behind the overhang.
///
/// The nose in the chevron notch is the case: the notch's own centre crease is one wall,
/// interrupted twice where the nose's two faces reach it, so its rim comes in three pieces with two
/// overhangs between them. Reading the first piece's end as the end of the wall says the face is
/// 6.667 across when it is the whole 20 - the wall is there the whole way, and what is missing is
/// only the RIM, which the island puts back. `tolerance` is how near two readings of one corner
/// have to be to be the same corner.
pub fn hides_the_end_of_a_wall(
    region: &FaceGroup,
    wall: &FeatureEdge,
    corner: Vector3,
    tolerance: f64,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
) -> bool {
    region.boundary_edges().iter().any(|rim| {
        rim.mesh_edge_index != wall.mesh_edge_index
            && ((rim.start - corner).length() <= tolerance
                || (rim.end - corner).length() <= tolerance)
            && stands_on(region, rim, beveled_mesh_edges, chain_vertices)
    })
}

/// Whether the straight RUN of rim edges that `rim` is one chord of reaches a chain vertex at
/// either of its ends - which is the same question as whether the crease it is part of meets the
/// chain, asked of a mesh that may have cut that crease anywhere along it.
fn meets_the_chain_along_its_line(
    region: &FaceGroup,
    rim: &FeatureEdge,
    chain_vertices: &HashSet<i32>,
) -> bool {
    if chain_vertices.is_empty() {
        return false;
    }

    let mut along = rim.end - rim.start;
    let length = along.length();

    if length <= 0.0 {
        return false;
    }

    along /= length;

    the_run_reaches_the_chain(region, rim.end_vertex, along, rim.convexity, chain_vertices)
        || the_run_reaches_the_chain(
            region,
            rim.start_vertex,
            -along,
            rim.convexity,
            chain_vertices,
        )
}

/// The walk itself, one way: from `from` onwards over whatever rim of the same region carries on
/// in `along` at the same `convexity`, until nothing does.
///
/// The CONVEXITY has to hold along the run, because a straight rim may change it at a vertex where
/// nothing else happens - a foot concave up to where something crosses it and convex beyond.
/// Letting the concave piece off for a corner the convex stretch reaches would leave a root nothing
/// closes.
///
/// The direction is what keeps the walk from turning back on itself, so no visited set is wanted:
/// every accepted step carries the projection on `along` forward by a positive amount. The step
/// count is capped at the rim's own length all the same.
///
/// Two readings this takes the simple way, both erring towards standing: `along` is never
/// refreshed, so straightness is measured against the FIRST chord rather than the last; and at a
/// fold vertex where two rims both carry on in the same direction, the first in mesh order is
/// taken and the other is not tried.
fn the_run_reaches_the_chain(
    region: &FaceGroup,
    mut from: i32,
    along: Vector3,
    convexity: EdgeConvexity,
    chain_vertices: &HashSet<i32>,
) -> bool {
    let rims = region.boundary_edges();

    for _ in 0..rims.len() {
        let mut carried_on = false;

        for next in rims {
            if next.convexity != convexity {
                continue;
            }

            let (onwards, beyond) = if next.start_vertex == from {
                (next.end - next.start, next.end_vertex)
            } else if next.end_vertex == from {
                (next.start - next.end, next.start_vertex)
            } else {
                continue;
            };

            let length = onwards.length();

            if length <= 0.0 || onwards.dot(along) / length < ALONG_THE_SAME_LINE {
                continue;
            }

            if chain_vertices.contains(&beyond) {
                return true;
            }

            from = beyond;
            carried_on = true;
            break;
        }

        if !carried_on {
            return false;
        }
    }

    false
}

/// Which FACE across `rim` is not the region's own, or `None` where neither side is the region's.
pub fn try_the_face_beyond(region: &FaceGroup, rim: &FeatureEdge) -> Option<i32> {
    if holds(region, rim.face0) {
        return Some(rim.face1);
    }

    if holds(region, rim.face1) {
        return Some(rim.face0);
    }

    None
}

/// The two normals of `rim` told apart by which side of it the region is on, as
/// `(landing, feature)`: the LANDING normal is the region's own face there, and the FEATURE normal
/// is the face across the rim. `None` where neither side is the region's.
pub fn try_the_two_sides(region: &FaceGroup, rim: &FeatureEdge) -> Option<(Vector3, Vector3)> {
    if holds(region, rim.face0) {
        return Some((rim.normal0, rim.normal1));
    }

    if holds(region, rim.face1) {
        return Some((rim.normal1, rim.normal0));
    }

    None
}

/// Whether the region holds one face. [`FaceGroup::faces`] is in ascending index order, so this is
/// a binary search rather than a walk - the question is asked once per rim edge of regions that
/// run to thousands of facets.
fn holds(region: &FaceGroup, face: i32) -> bool {
    region.faces().binary_search(&face).is_ok()
}
