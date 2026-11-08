//! A junction where TWO creases meet one bead at one corner, as free functions: the reading
//! ([`try_read`]), the tests it is made of, and the bead edge that carries its ring.

use std::rc::Rc;

use solidmesh_math::{ieee_math, Vector3};

use super::{
    SurfaceBevelPlan, SurfaceMixedCorner, SurfaceMixedCornerDualFrame, SurfaceMixedJunction,
    SurfaceMixedJunctionArm, SurfaceMixedJunctionChainEnd,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// How near the two arms' readings of the SAME turn have to agree, as a dimensionless angle and
/// so a fixed epsilon - the same reasoning as [`SurfaceMixedCorner::SQUARE_ENOUGH`].
pub const SAME_TURN: f64 = 1e-6;

/// How near the two walls' outward normals have to stand to be one plane, as the length between
/// two unit vectors. Dimensionless, so fixed rather than the part's own ruler.
pub const SAME_WALL: f64 = 1e-6;

/// Reads the junction one cut and two creases make at `corner` (the welded mesh's
/// `vertex`), or says why it cannot be read - always named AT the corner. Pure: it asks the plans
/// and nothing else. `ends` are the three chain ends the gathering found, in the plans' own order;
/// `creases` says which two of them are the creases (exactly two, which is what picks this
/// reading), so the third end is the cut. A shape this does not answer falls back to
/// [`BevelAdvisoryCode::SurfaceJunctionOfTwoCreases`].
pub fn try_read(
    plans: &[Rc<SurfaceBevelPlan>],
    vertex: i32,
    corner: Vector3,
    near_enough: f64,
    ends: &[SurfaceMixedJunctionChainEnd],
    creases: &[usize],
) -> Result<SurfaceMixedJunction, BevelAdvisory> {
    let two_creases = || refusal(BevelAdvisoryCode::SurfaceJunctionOfTwoCreases, corner);

    // The one end of the three that is not a crease is the cut, and BOTH arms are read off it.
    let mut cut = 0;
    for end in 0..ends.len() {
        if creases[0] != end && creases[1] != end {
            cut = end;
        }
    }

    let mut arms = Vec::with_capacity(2);
    for &crease in creases {
        let arm = SurfaceMixedJunction::try_read_the_arm(
            plans,
            ends[cut],
            ends[crease],
            corner,
            plans[ends[crease].plan as usize].size,
            near_enough,
        )
        .map_err(|reason| reason.unwrap_or_else(two_creases))?;

        if arm.mixed.hands_over_at_a_mitre {
            // The ring stands short of the tangency on a chord BEFORE the one that touches, so this
            // arm's dual is drawn about a different cut line from the other's - two canals, and the
            // shape is still not one this reading answers.
            return Err(two_creases());
        }

        arms.push(arm);
    }

    if arms[0].mixed.shared_region_key == arms[1].mixed.shared_region_key {
        // Both creases read as standing on the SAME face of the cut. DEFENSIVE: the mirror of the
        // one-crease branch's "two cuts sharing one face" guard, and no fixture drives it - named
        // rather than built, because a corner whose arms disagree about the faces is not one canal.
        return Err(two_creases());
    }

    if !the_creases_stand_against_one_wall(&arms[0].mixed, &arms[1].mixed) {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceJunctionOfTwoCreasesOnTwoWalls,
            corner,
        ));
    }

    let mut duals = Vec::with_capacity(2);
    for arm in &arms {
        duals.push(Rc::new(arm.mixed.try_read_the_dual_canal()?));
    }

    if !the_two_duals_are_one_canal(&duals[0], &duals[1], near_enough) {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceJunctionBeadsDoNotShareOneCanal,
            corner,
        ));
    }

    let mut carrying = Vec::with_capacity(2);
    for (arm, dual) in arms.iter().zip(&duals) {
        carrying.push(
            try_the_bead_edge_carrying_the_ring(plans, arm, dual, corner, near_enough).ok_or_else(
                || {
                    refusal(
                        BevelAdvisoryCode::SurfaceJunctionCanalRunsOffTheBead,
                        corner,
                    )
                },
            )?,
        );
    }

    let (second_carries, second_swallowed) = carrying.pop().expect("two arms were read");
    let (first_carries, first_swallowed) = carrying.pop().expect("two arms were read");

    Ok(SurfaceMixedJunction::of_two_creases(
        corner,
        vertex,
        with_the_canal(&arms[0], duals[0].clone(), first_carries, first_swallowed),
        with_the_canal(&arms[1], duals[1].clone(), second_carries, second_swallowed),
    ))
}

/// A refusal of this reading's, which every code here words at the corner.
fn refusal(code: BevelAdvisoryCode, at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(code, vec![BevelAdvisoryArg::Vector3(at)])
}

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
            ieee_math::max_f64((edge.from() - foot).dot(away), (edge.to() - foot).dot(away));

        if station <= reaches + near_enough {
            return Some((at, passed));
        }

        passed.push(at);
        at += step;
    }

    None
}

/// `arm` read against the canal (`dual`), pointed at the bead edge that really carries the ring
/// (`carries`), with the chords before it (`swallowed`).
pub fn with_the_canal(
    arm: &SurfaceMixedJunctionArm,
    dual: Rc<SurfaceMixedCornerDualFrame>,
    carries: i32,
    swallowed: Vec<i32>,
) -> SurfaceMixedJunctionArm {
    SurfaceMixedJunctionArm::new(
        arm.plan,
        arm.cut_index,
        arm.corner_at_the_start,
        arm.swallowed.clone(),
        arm.mixed.clone(),
        SurfaceMixedJunctionChainEnd::new(arm.bead_end.plan, carries, arm.bead_end.at_the_start),
        Some(dual),
        Some(swallowed),
    )
}
