//! The fixed point the room's largest size is stepped down to (`Bevel/Surface/SurfaceRoomFixedPoint.cs`,
//! a static class: free functions here): the named size settled ([`settled`]) by planning every
//! chain again at each candidate and measuring it, then checked ([`accepted`]) and stepped down a
//! ruler while the fit still refuses it. Only ever reached on a refusal.

use std::rc::Rc;

use polygon_mesh::CancelToken;

use super::{surface_room, surface_room_measure, SurfaceBevelPlan, SurfaceVertexPool};
use crate::bevel::level_line::WorkError;

/// The most times the size is stepped down.
pub const MOST_STEPS: i32 = 5;

/// How near a size has to fit to count as fitting.
pub const FIT_TOLERANCE: f64 = 1e-9;

/// C# `Settled`: the size `named` settled - a size that, planned at and measured again, fits.
/// `plans` are the pass as drawn at the requested size; `named` is what the first measurement
/// named, where the walk starts and the most it can return. A cancel of a measurement passes up.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
pub fn settled(
    plans: &[Rc<SurfaceBevelPlan>],
    named: f64,
    cancellation_token: &CancelToken,
) -> Result<f64, WorkError> {
    let ruler = ruler_of(plans);

    if !(named > 0.0) || !(ruler > 0.0) {
        return Ok(named);
    }

    let mut candidate = named;

    for _ in 0..MOST_STEPS {
        let Some(replanned) = try_plan_all(plans, candidate) else {
            // A candidate the chain cannot be planned at names nothing better than the first
            // reading did.
            return Ok(candidate);
        };

        let allowed = largest_at(&replanned, candidate, cancellation_token)?;

        if !(allowed < candidate - (candidate * FIT_TOLERANCE)) {
            // Measured at itself the candidate fits.
            return Ok(candidate);
        }

        // Within one ruler of itself the walk has arrived.
        let arrived = candidate - allowed <= ruler;

        candidate = allowed;

        if arrived {
            break;
        }
    }

    accepted(
        plans,
        stepped_down(candidate, ruler),
        ruler,
        cancellation_token,
    )
}

/// C# `Accepted`: the stepped-down size CHECKED - planned at and measured - and stepped down again
/// by the ruler while the fit still refuses it, at most [`MOST_STEPS`] times more.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x < y)`, which reads a NaN as fitting.
pub fn accepted(
    plans: &[Rc<SurfaceBevelPlan>],
    mut size: f64,
    ruler: f64,
    cancellation_token: &CancelToken,
) -> Result<f64, WorkError> {
    for _ in 0..MOST_STEPS {
        if let Some(replanned) = try_plan_all(plans, size) {
            if !(largest_at(&replanned, size, cancellation_token)? < size - (size * FIT_TOLERANCE))
            {
                return Ok(size);
            }
        }

        let under = stepped_down(size, ruler);

        if !(under < size) {
            // The ruler is as large as the size itself: nowhere under it to step to.
            return Ok(size);
        }

        size = under;
    }

    Ok(size)
}

/// C# `LargestAt`: the largest size the pass, as planned at `size`, leaves room for - read against
/// infinity, so it may come back LARGER than `size`, which every caller here reads as "fits"
/// exactly as a truthful answer would.
pub fn largest_at(
    planned: &[Rc<SurfaceBevelPlan>],
    size: f64,
    cancellation_token: &CancelToken,
) -> Result<f64, WorkError> {
    let cuts = surface_room_measure::measure(planned, None, None);
    let (largest, _, _) =
        surface_room::largest_fitting(&cuts, f64::INFINITY, cancellation_token, size, None, None)?;

    Ok(largest)
}

/// `size` one `ruler` down, where that is still a size; `size` itself otherwise.
pub fn stepped_down(size: f64, ruler: f64) -> f64 {
    if size - ruler > 0.0 {
        size - ruler
    } else {
        size
    }
}

/// C# `TryPlanAll`: the whole pass planned again at `size`, or `None` where any chain of it cannot
/// be, or the pass is empty.
pub fn try_plan_all(
    plans: &[Rc<SurfaceBevelPlan>],
    size: f64,
) -> Option<Vec<Rc<SurfaceBevelPlan>>> {
    let mut replanned = Vec::with_capacity(plans.len());

    for plan in plans {
        replanned.push(plan.recipe.try_plan_at(size).ok()?);
    }

    (!replanned.is_empty()).then_some(replanned)
}

/// The part's weld ruler, read off the first plan with a mesh; zero where none has one.
pub fn ruler_of(plans: &[Rc<SurfaceBevelPlan>]) -> f64 {
    plans
        .iter()
        .find_map(|plan| plan.mesh())
        .map_or(0.0, |mesh| {
            SurfaceVertexPool::near_enough_for(&mesh.vertices)
        })
}
