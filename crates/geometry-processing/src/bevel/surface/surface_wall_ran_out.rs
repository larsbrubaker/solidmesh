//! Where a roll across a curved wall RUNS OUT of wall before the profile fits
//! (`Bevel/Surface/SurfaceWallRanOut.cs`, a static class: free functions here). The bound one
//! linear condition puts on the size lands here first; the reading itself lands with later
//! packages.

use vector_math::cs_math;
use vector_math::Vector3;

use super::surface_roll_across_the_wall::{
    is_finite, out_of_the_facet, try_centre, SurfaceRollAcrossTheWallSides,
};
use super::SurfaceRunAcrossSectionStep;

/// Narrows the window `lowest..highest` of sizes to where the linear condition
/// `per_size * size + at_nothing >= 0` holds: a condition that does not move with the size either
/// always holds or never does (then the window closes).
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
pub fn bound(per_size: f64, at_nothing: f64, lowest: &mut f64, highest: &mut f64) {
    if !(per_size.abs() > 1e-12) {
        // The condition does not move with the size: it either always holds or never does.
        if at_nothing < 0.0 {
            *highest = f64::NEG_INFINITY;
        }

        return;
    }

    let crosses_at = -at_nothing / per_size;

    if per_size > 0.0 {
        *lowest = cs_math::max_f64(*lowest, crosses_at);
    } else {
        *highest = cs_math::min_f64(*highest, crosses_at);
    }
}

/// C# `TheProfileCanRestOnTheCrease`: whether the crease between `near` and `far` turns the way a
/// profile can rest on it - the far facet heading back against the near facet's outward side.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn the_profile_can_rest_on_the_crease(
    side: &SurfaceRollAcrossTheWallSides,
    near: &SurfaceRunAcrossSectionStep,
    far: &SurfaceRunAcrossSectionStep,
    convex: bool,
) -> bool {
    let mut near_trace = near.to - near.from;
    let mut far_trace = far.to - far.from;

    if !(near_trace.length_squared() > 0.0) || !(far_trace.length_squared() > 0.0) {
        return false;
    }

    near_trace = near_trace.get_normal();
    far_trace = far_trace.get_normal();

    let wedge_sign = if convex { -1.0 } else { 1.0 };
    let towards_the_near = out_of_the_facet(side.along, near_trace, near.normal) * wedge_sign;

    far_trace.dot(towards_the_near) < 0.0
}

/// C# `TryFeetAt`: for the circle of radius `offset` tangent to the flat plane and to `step`'s
/// facet, how far along the facet's trace its wall foot falls and how far into the flat face its
/// flat foot does - `(along_the_trace, into_the_flat_face)`, or `None` where there is no centre.
#[allow(clippy::too_many_arguments)]
pub fn try_feet_at(
    side: &SurfaceRollAcrossTheWallSides,
    origin: Vector3,
    step: &SurfaceRunAcrossSectionStep,
    flat_normal: Vector3,
    facet_normal: Vector3,
    trace: Vector3,
    offset: f64,
) -> Option<(f64, f64)> {
    let centre = try_centre(
        side.along,
        origin,
        flat_normal,
        step.from,
        facet_normal,
        offset,
    )?;

    let along_the_trace = (centre - (facet_normal * offset) - step.from).dot(trace);
    let into_the_flat_face = (centre - (flat_normal * offset) - origin).dot(side.flat_inward);

    if is_finite(along_the_trace) && is_finite(into_the_flat_face) {
        Some((along_the_trace, into_the_flat_face))
    } else {
        None
    }
}
