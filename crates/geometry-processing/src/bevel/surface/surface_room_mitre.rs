//! The wedge two bands mitre into at a REFLEX corner of one region, as the room reads it. The tip
//! of that wedge is a footprint of its own ([`SurfaceRoomFootprint::MitreTip`]); the overhang a
//! mitre carries a band by is read here too, and every tip of a plan is added ([`add_to`]).

use vector_math::{ieee_math, Vector3};

use super::{SurfaceBevelPlan, SurfaceEdgePlan, SurfaceRoomCut, SurfaceRoomFootprint};

/// How near straight on two edges have to be for their bands to be one band rather than two
/// mitred into a tip, as a cosine.
const STRAIGHT_COSINE: f64 = 0.9999;

/// How near two points have to be to be the same corner, relative to the edge's length with a
/// floor of one unit.
const VERTEX_TOLERANCE: f64 = 1e-5;

/// How nearly along the edge a mitre plane may lie before the crossing it names is off at the far
/// end of the arithmetic.
const OVERHANG_IS_OFF_THE_PART: f64 = 1e-5;

/// The mitre tip at the corner `arriving` ends at and `leaving` starts from, on the region both
/// of the named sides land on - or `None` where they land on two regions, do not meet, run
/// straight on, or the corner is convex (its crossing stands between the two edges rather than
/// BEHIND both of them, out in the middle of the face, which is what nothing else measures).
pub fn try_cut(
    plan: &SurfaceBevelPlan,
    arriving: &SurfaceEdgePlan,
    arriving_left: bool,
    leaving: &SurfaceEdgePlan,
    leaving_left: bool,
) -> Option<SurfaceRoomCut> {
    let region_key = if arriving_left {
        arriving.left_region_key()
    } else {
        arriving.right_region_key()
    };

    if region_key
        != (if leaving_left {
            leaving.left_region_key()
        } else {
            leaving.right_region_key()
        })
    {
        return None;
    }

    let corner = arriving.to();

    if (leaving.from() - corner).length()
        > ieee_math::max_f64((arriving.to() - arriving.from()).length(), 1.0) * VERTEX_TOLERANCE
    {
        // Not one corner at all: the chain's own order says these two edges meet, and where they
        // do not there is no crossing to read.
        return None;
    }

    // The direction at the EDGE and the setback that side really gives up, read exactly as the
    // room measure reads them for the band itself, so the tip stands on the two lines the bands
    // are bounded by.
    let (arriving_inward, arriving_setback) = try_band(plan, arriving, arriving_left)?;
    let (leaving_inward, leaving_setback) = try_band(plan, leaving, leaving_left)?;

    let leaning = arriving_inward.dot(leaving_inward);

    if leaning.abs() >= STRAIGHT_COSINE {
        return None;
    }

    // Where the two offset lines cross, written in the two inward directions themselves: the point
    // at arriving_setback from one line and leaving_setback from the other.
    let across_the_arriving =
        (arriving_setback - (leaning * leaving_setback)) / (1.0 - (leaning * leaning));
    let across_the_leaving =
        (leaving_setback - (leaning * arriving_setback)) / (1.0 - (leaning * leaning));

    let out_of_the_corner =
        (arriving_inward * across_the_arriving) + (leaving_inward * across_the_leaving);
    let reach = out_of_the_corner.length();

    if reach.is_nan() || reach <= 0.0 {
        return None;
    }

    // REFLEX, and told from a convex corner by where the crossing lands rather than by an angle.
    if out_of_the_corner.dot(arriving.from() - corner) >= 0.0
        || out_of_the_corner.dot(leaving.to() - corner) >= 0.0
    {
        return None;
    }

    Some(SurfaceRoomCut {
        shape: SurfaceRoomFootprint::MitreTip,
        ..SurfaceRoomCut::new(
            region_key,
            corner,
            corner,
            Vector3::ZERO,
            out_of_the_corner / reach,
            0.0,
            reach / plan.size,
            f64::INFINITY,
        )
    })
}

/// The way the chain runs on PAST the corner at one end of edge `edge_index` (its end where
/// `at_the_end`, its start otherwise) - out of the corner and down the neighbour - or zero where
/// nothing carries on there: a free end, a mixed corner, or a neighbour that does not meet it.
pub fn chains_direction_past(
    plan: &SurfaceBevelPlan,
    edge_index: usize,
    at_the_end: bool,
) -> Vector3 {
    let last = plan.edges.len() as i64 - 1;
    let mut arrived_on = if at_the_end {
        edge_index as i64
    } else {
        edge_index as i64 - 1
    };

    if arrived_on < 0 {
        arrived_on = if plan.is_closed { last } else { -1 };
    }

    if arrived_on < 0
        || (arrived_on == last && !plan.is_closed)
        || plan.try_mixed_corner_after(arrived_on as i32).is_some()
    {
        return Vector3::ZERO;
    }

    let neighbour = &plan.edges[if at_the_end {
        if (edge_index as i64) < last {
            edge_index + 1
        } else {
            0
        }
    } else {
        arrived_on as usize
    }];
    let edge = &plan.edges[edge_index];
    let corner = if at_the_end { edge.to() } else { edge.from() };

    // Out of the corner and down the neighbour, whichever way round the chain's order holds it.
    let away = if at_the_end {
        neighbour.to() - neighbour.from()
    } else {
        neighbour.from() - neighbour.to()
    };
    let meets = if at_the_end {
        neighbour.from()
    } else {
        neighbour.to()
    };

    let square = away.length_squared();
    if (meets - corner).length()
        > ieee_math::max_f64((edge.to() - edge.from()).length(), 1.0) * VERTEX_TOLERANCE
        || square.is_nan()
        || square <= 0.0
    {
        return Vector3::ZERO;
    }

    away.get_normal()
}

/// How far past the corner a band's material reaches per unit of setback, where the mitre between
/// `into_the_edge` (the way into this edge from the corner) and `past_the_corner` (the way the
/// chain runs on) carries it - zero where nothing carries on, or the mitre cuts the band back.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
pub fn overhang_per_setback(
    into_the_edge: Vector3,
    past_the_corner: Vector3,
    inward: Vector3,
) -> f64 {
    if !(past_the_corner.length_squared() > 0.0) {
        // Nothing carries on past this corner, so nothing mitres the band there.
        return 0.0;
    }

    let normal = past_the_corner - into_the_edge;

    if !(normal.length_squared() > 0.0) {
        return 0.0;
    }

    let normal = normal.get_normal();

    let along_the_edge = into_the_edge.dot(normal);

    if !(along_the_edge.abs() > OVERHANG_IS_OFF_THE_PART) {
        // A chain doubling back on itself, where the mitre plane lies so nearly along the edge
        // that the crossing it names is off at the far end of the arithmetic. The plan bounds the
        // turn long before this, so there is only an infinity to keep out of the window.
        return 0.0;
    }

    ieee_math::max_f64(0.0, inward.dot(normal) / along_the_edge)
}

/// The unit inward direction at the edge and the setback one side of `edge` gives up, as the
/// room measure reads them; `None` where either is missing or not a finite positive length.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_band(
    _plan: &SurfaceBevelPlan,
    edge: &SurfaceEdgePlan,
    left: bool,
) -> Option<(Vector3, f64)> {
    let inward = if left {
        edge.left_inward_at_the_edge
    } else {
        edge.right_inward_at_the_edge
    };
    let setback = if left {
        edge.left_setback
    } else {
        edge.right_setback
    };

    // A plan's regions are always present, so there is no missing-region case to refuse.
    if !(inward.length_squared() > 0.0)
        || !(setback > 0.0)
        || setback.is_infinite()
        || setback.is_nan()
    {
        return None;
    }

    Some((inward.get_normal(), setback))
}

/// Every mitre tip this plan's corners lay on the regions they turn across - one per corner per
/// SHARED region, and only where that corner is reflex in it - added to `cuts`. A MIXED corner is
/// left out: the convexity turns over there, so the two bands do not mitre at all and what stands
/// past the corner is the run-out, read as its own cut.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn add_to(cuts: &mut Vec<SurfaceRoomCut>, plan: Option<&SurfaceBevelPlan>) {
    let Some(plan) = plan else {
        return;
    };

    if !(plan.size > 0.0) || plan.edges.is_empty() {
        return;
    }

    let last = plan.edges.len() - 1;

    for arrived_on in 0..=last {
        let leaves_on = if arrived_on < last { arrived_on + 1 } else { 0 };

        if leaves_on == arrived_on
            || (arrived_on == last && !plan.is_closed)
            || plan.try_mixed_corner_after(arrived_on as i32).is_some()
        {
            continue;
        }

        let arriving = &plan.edges[arrived_on];
        let leaving = &plan.edges[leaves_on];

        for arriving_side in 0..2 {
            for leaving_side in 0..2 {
                if let Some(tip) = try_cut(
                    plan,
                    arriving,
                    arriving_side == 0,
                    leaving,
                    leaving_side == 0,
                ) {
                    cuts.push(tip);
                }
            }
        }
    }
}
