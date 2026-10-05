//! A junction where TWO creases meet one bead at one corner
//! (`Bevel/Surface/SurfaceTwoCreaseJunction.cs`, a static class: free functions here). The tests
//! the junction's reading is made of, and the bead edge that carries its ring, land here first;
//! the reading itself lands with a later package.

use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::{
    SurfaceBevelPlan, SurfaceMixedCorner, SurfaceMixedCornerDualFrame, SurfaceMixedJunctionArm,
};

/// How near the two arms' readings of the SAME turn have to agree, as a dimensionless angle and
/// so a fixed epsilon - the same reasoning as [`SurfaceMixedCorner::SQUARE_ENOUGH`].
pub const SAME_TURN: f64 = 1e-6;

/// How near the two walls' outward normals have to stand to be one plane, as the length between
/// two unit vectors. Dimensionless, so fixed rather than the part's own ruler.
pub const SAME_WALL: f64 = 1e-6;

/// Whether the two creases stand against ONE face - the same region, and one plane. Both,
/// because two coplanar faces of a part are still two faces to walk back, and two facets of one
/// region can stand at an angle where a bead rolled along it.
pub fn the_creases_stand_against_one_wall(
    first: &SurfaceMixedCorner,
    second: &SurfaceMixedCorner,
) -> bool {
    wall_key_of(first) == wall_key_of(second)
        && (first.out_of_the_wall - second.out_of_the_wall).length() <= SAME_WALL
}

/// Whether both arms read ONE canal: the same turn, and the ball each hands over at is the ball
/// the other starts from. Asked both ways about, because the two arms trace the one ellipse in
/// opposite senses and a reading that agreed at one end only would be two arcs of it rather than
/// one.
pub fn the_two_duals_are_one_canal(
    first: &SurfaceMixedCornerDualFrame,
    second: &SurfaceMixedCornerDualFrame,
    near_enough: f64,
) -> bool {
    (first.turn_radians - second.turn_radians).abs() <= SAME_TURN
        && (first.spine.centre_at(first.turn_radians) - second.spine.centre_at(0.0)).length()
            <= near_enough
        && (second.spine.centre_at(second.turn_radians) - first.spine.centre_at(0.0)).length()
            <= near_enough
}

/// The face the bead stands against that nobody picked - its side that is not the shared one.
pub fn wall_key_of(mixed: &SurfaceMixedCorner) -> i32 {
    if mixed.bead.left_region_key() == mixed.shared_region_key {
        mixed.bead.right_region_key()
    } else {
        mixed.bead.left_region_key()
    }
}

/// Which edge of the bead's chain really CARRIES the blend's end ring on `arm`, and which chords
/// between it and the corner the blend SWALLOWS, as `(carries, swallowed)` - or `None` where the
/// dual's first ball does not stand on the bead's axis, stands behind the corner, or the ring
/// stands past the whole chain.
///
/// **Read as the station, not as a count of edges.** The canal's first ball on this arm is
/// centred ON the bead's own axis, so the ring it is tangent to is the bead's SQUARE section
/// there, and the one number that places it is how far along the axis that centre stands from
/// the corner's own foot on it. The edge whose span reaches that far is the one with band left to
/// carry the ring; every edge the walk passed on the way is wholly inside the blend and owes no
/// strip. Measured ALONG the axis - exact for a COLLINEAR chain, which is what a split crease is.
/// A station landing exactly on a chord's far end is given to that chord rather than the next.
pub fn try_the_bead_edge_carrying_the_ring(
    plans: &[Rc<SurfaceBevelPlan>],
    arm: &SurfaceMixedJunctionArm,
    dual: &SurfaceMixedCornerDualFrame,
    corner: Vector3,
    near_enough: f64,
) -> Option<(i32, Vec<i32>)> {
    let mixed = &arm.mixed;
    let centre = dual.spine.centre_at(0.0);

    if (centre - mixed.axis().closest_point_to(centre)).length() > near_enough {
        return None;
    }

    let away = SurfaceMixedCorner::away_from_the_corner(&mixed.bead, corner);
    let foot = mixed.axis().closest_point_to(corner);
    let station = (centre - foot).dot(away);

    if station < 0.0 {
        return None;
    }

    let edges = &plans[arm.bead_end.plan as usize].edges;
    let step = if arm.bead_end.at_the_start { 1 } else { -1 };
    let mut passed = Vec::new();

    let mut at = arm.bead_end.edge;
    while at >= 0 && (at as usize) < edges.len() {
        let edge = &edges[at as usize];

        // How far down the axis this edge reaches. Both ends are asked and the further taken, so
        // which way the edge's own From and To run is nobody's business here.
        let reaches =
            cs_math::max_f64((edge.from() - foot).dot(away), (edge.to() - foot).dot(away));

        if station <= reaches + near_enough {
            return Some((at, passed));
        }

        passed.push(at);
        at += step;
    }

    None
}
