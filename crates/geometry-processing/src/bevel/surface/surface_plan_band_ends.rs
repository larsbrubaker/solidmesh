//! Where a chain edge's band stops at a corner of the chain - the mitre plane between two edges -
//! as the plan writes it (`Bevel/Surface/SurfacePlanBandEnds.cs`, a static class: free functions
//! here). The overloads that read two edge plans (through the mitre seam) land with the round-2
//! package; this module holds the direction-only forms and the side test.

use vector_math::Vector3;

use super::{SurfaceBandEnd, SurfaceEdgePlan};

/// How far apart two wedges have to be before one bead is called wider than the other. Well under
/// any real difference - the case this tells apart is 45 against 90 - and well over the rounding
/// of two dihedrals read off the same mesh, so two edges of one rim are never accidentally ranked.
const WEDGES_ARE_THE_SAME_WITHIN: f64 = 1e-6;

/// Whether the band on region `region_key` of `here` stops at the mitre it shares with
/// `neighbour`. Always on the face the two edges share; otherwise it comes down to which of the
/// two beads is the WIDER - the one whose wedge is the smaller. The wider bead has rails the
/// narrower has none to pair with, and those rails land on the narrower edge's own far face; so
/// THAT face stops at the mitre, while the wider bead's own far face has nothing arriving on it
/// and runs to its region's rim.
pub fn the_mitre_bounds_this_side(
    here: &SurfaceEdgePlan,
    neighbour: &SurfaceEdgePlan,
    region_key: i32,
) -> bool {
    if region_key == neighbour.left_region_key() || region_key == neighbour.right_region_key() {
        return true;
    }

    here.wedge_angle_degrees > neighbour.wedge_angle_degrees + WEDGES_ARE_THE_SAME_WITHIN
}

/// The band end at the corner an edge STARTS at, where the chain arrived along `arriving` and goes
/// on along `here`: the mitre plane between the two, with its normal pointing the way this edge
/// lies, which is into its own band. (C#'s `BandEndWhereTheChainArrives(Vector3, Vector3,
/// Vector3)` overload.)
pub fn band_end_where_the_chain_arrives_along(
    corner: Vector3,
    arriving: Vector3,
    here: Vector3,
) -> SurfaceBandEnd {
    SurfaceBandEnd::new(corner, mitre_normal(arriving, here, here))
}

/// The band end at the corner an edge ENDS at, where the chain runs along `here` and leaves along
/// `leaving`: the same mitre plane, with its normal turned to point BACK along this edge, which is
/// again into its own band. (C#'s `BandEndWhereTheChainLeaves(Vector3, Vector3, Vector3)`.)
///
/// The square-to fallback is `here` and not its negative, for all that this is the end that
/// negates: what the fallback names is the plane's own facing, taken before the turn, so negating
/// it here is what makes the degenerate answer point into the band like every other one.
pub fn band_end_where_the_chain_leaves_along(
    corner: Vector3,
    here: Vector3,
    leaving: Vector3,
) -> SurfaceBandEnd {
    SurfaceBandEnd::new(corner, -mitre_normal(here, leaving, here))
}

/// The mitre plane's normal for a chain arriving along `arriving` and leaving along `leaving`,
/// pointing the way the LEAVING edge lies. Where the two double straight back on each other the
/// sum is nothing and there is no mitre, so the band is bounded square to the edge instead -
/// `square_to` - which is the answer the strips will refuse at anyway.
pub fn mitre_normal(arriving: Vector3, leaving: Vector3, square_to: Vector3) -> Vector3 {
    let summed = arriving + leaving;

    if summed.length_squared() > 0.0 {
        summed.get_normal()
    } else {
        square_to
    }
}

/// How near the two tangent lines have to come to count as crossing, at PLAN time - where there is
/// no pool and so no part ruler to ask: a millionth of the wider setback of the two edges.
pub fn crosses_within(one: &SurfaceEdgePlan, other: &SurfaceEdgePlan) -> f64 {
    vector_math::cs_math::max_f64(one.setback(), other.setback()) * 1e-6
}
