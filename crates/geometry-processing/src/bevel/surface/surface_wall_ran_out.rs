//! Where a roll across a curved wall RUNS OUT of wall before the profile fits
//! (`Bevel/Surface/SurfaceWallRanOut.cs`, a static class: free functions here): the bound one
//! linear condition puts on the size, each facet's window of sizes ([`try_window_on`]) and the
//! advisory that names the largest size that fits ([`advisory`]).

use vector_math::cs_math;
use vector_math::Vector3;

use polygon_mesh::Mesh;

use super::super::{BevelAdvisory, BevelAdvisoryCode};
use super::surface_roll_across_the_wall::{
    is_finite, out_of_the_facet, sections_of, try_centre, SurfaceRollAcrossTheWallSides,
};
use super::surface_run_across::SurfaceRunAcrossRegionMapsCache;
use super::surface_run_across_walk::try_walk_one_section;
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

/// C# `TryWindowOn`: the window of sizes (`lowest`, `highest`) one facet of the walk can hold the
/// profile's tangency for - a chamfer's from where the section entered it to where it leaves, a
/// round's where the wall foot stays on the facet and the flat foot has not walked back behind the
/// edge. `None` where the facet holds no size.
#[allow(clippy::too_many_arguments)]
pub fn try_window_on(
    side: &SurfaceRollAcrossTheWallSides,
    origin: Vector3,
    step: &SurfaceRunAcrossSectionStep,
    size: f64,
    round_profile: bool,
    convex: bool,
    flat_normal: Vector3,
) -> Option<(f64, f64)> {
    if !round_profile {
        // A chamfer's foot is its own size along the section.
        return Some((step.run_to_entry, step.run_to_entry + step.crossed()));
    }

    let trace = step.to - step.from;
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN refuses, as the C#'s `!(x > 0)` does
    let no_trace = !(trace.length_squared() > 0.0);
    if no_trace {
        return None;
    }

    let trace = trace.get_normal();
    let facet_normal = out_of_the_facet(side.along, trace, step.normal);
    let sign = if convex { -1.0 } else { 1.0 };

    let (wall_at_nothing, flat_at_nothing) =
        try_feet_at(side, origin, step, flat_normal, facet_normal, trace, 0.0)?;
    let (wall_at_size, flat_at_size) = try_feet_at(
        side,
        origin,
        step,
        flat_normal,
        facet_normal,
        trace,
        sign * size,
    )?;

    let mut lowest = 0.0;
    let mut highest = f64::INFINITY;

    // The foot is on the facet, and the flat side's foot has not walked back behind the edge.
    bound(
        (wall_at_size - wall_at_nothing) / size,
        wall_at_nothing,
        &mut lowest,
        &mut highest,
    );
    bound(
        -(wall_at_size - wall_at_nothing) / size,
        step.crossed() - wall_at_nothing,
        &mut lowest,
        &mut highest,
    );
    bound(
        (flat_at_size - flat_at_nothing) / size,
        flat_at_nothing,
        &mut lowest,
        &mut highest,
    );

    (highest > lowest && is_finite(highest) && highest > 0.0).then_some((lowest, highest))
}

/// C# `Advisory`: what to tell the reader when a roll across the wall ran out of wall - the
/// largest size every section's walk holds, contiguous from the edge outward
/// (`SizeExceedsFaceWidth` with the size, where and what fits), or that the corner itself has no
/// room (`BevelDoesNotFitCorner`).
#[allow(clippy::too_many_arguments)]
pub fn advisory(
    mesh: &Mesh,
    side: &SurfaceRollAcrossTheWallSides,
    size: f64,
    round_profile: bool,
    convex: bool,
    flat_normal: Vector3,
    on_the_trace: f64,
    mut cache: Option<&mut SurfaceRunAcrossRegionMapsCache>,
) -> BevelAdvisory {
    let mut fits = f64::INFINITY;

    for fraction in sections_of(side, on_the_trace) {
        let Some((steps, _)) = try_walk_one_section(
            mesh,
            &side.wall_region,
            side.chain_edge.from(),
            side.chain_edge.to(),
            side.along,
            side.wall_inward,
            fraction,
            cache.as_deref_mut(),
        ) else {
            continue;
        };

        let mut here = 0.0;

        // CONTIGUOUS from the edge outward, and in ORDER: the walk carries on only while the sizes
        // covered run up to the next facet's, and stops at the first real break.
        for k in 0..steps.len() {
            let Some((lowest, highest)) = try_window_on(
                side,
                steps[0].from,
                &steps[k],
                size,
                round_profile,
                convex,
                flat_normal,
            ) else {
                break;
            };

            if lowest > here + on_the_trace
                && (k == 0
                    || !the_profile_can_rest_on_the_crease(side, &steps[k - 1], &steps[k], convex))
            {
                break;
            }

            here = cs_math::max_f64(here, highest);
        }

        fits = cs_math::min_f64(fits, here);
    }

    #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN refuses, as the C#'s `!(x > y)` does
    let no_room = !is_finite(fits) || !(fits > on_the_trace) || fits >= size;
    if no_room {
        // The corner itself is what has no room in it.
        return BevelAdvisory::new(BevelAdvisoryCode::BevelDoesNotFitCorner, Vec::new());
    }

    BevelAdvisory::new(
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        vec![size.into(), side.midpoint().into(), fits.into()],
    )
}
