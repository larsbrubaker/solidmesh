//! Whether a swept tool FOLDS at the size it was asked for ([`try_check_for_folds`]): a joint or a
//! cap that drags a strip back further than its segment (or a capped straight run) is long turns
//! that strip inside out.

use vector_math::{ieee_math, Vector3};

use super::bevel_chain_sweep_mesh::SweptSegment;
use super::bevel_straight_run::{run_to_ring, straight_run_from, TOUCHING_RING};
use super::{BevelAdvisory, BevelAdvisoryCode, BevelChainEnds};

/// Refuses the sweep where some strip folds at `size`: for each profile point of each segment
/// the strip is dragged back by `start slide - end slide`, which must not outrun the segment's
/// length (or, beside a capped end, the straight run from that end; and, where both ends are
/// capped on one straight run, the whole run). The largest size that would fit is reported - at
/// the cap corner where a cap pulls the tool back, at the worst segment's middle otherwise.
pub fn try_check_for_folds(
    swept: &[SweptSegment],
    ends: &BevelChainEnds,
    closed: bool,
    size: f64,
    point_count: usize,
) -> Result<(), BevelAdvisory> {
    let slides =
        |slides: &Option<Vec<f64>>, point: usize| slides.as_ref().map_or(0.0, |s| s[point]);

    let mut worst_segment: Option<usize> = None;
    let mut largest_that_fits = f64::MAX;

    let mut worst_cap_corner: Option<Vector3> = None;

    let start_is_capped = !closed && ends.start.is_capped();
    let end_is_capped = !closed && ends.end.is_capped();

    let start_run = if start_is_capped {
        straight_run_from(swept, 0, true)
    } else {
        0.0
    };
    let end_run = if end_is_capped {
        straight_run_from(swept, swept.len() - 1, false)
    } else {
        0.0
    };

    let last_index = swept.len() - 1;

    for (i, segment) in swept.iter().enumerate() {
        for point in 0..point_count {
            let start_slide = slides(&segment.start_slide, point);
            let end_slide = slides(&segment.end_slide, point);
            let drag = start_slide - end_slide;

            if drag <= 0.0 {
                continue;
            }

            let mut available = segment.length;

            if i == 0 && start_is_capped && start_slide > 0.0 {
                available = start_run;
            }

            if i == last_index && end_is_capped && end_slide < 0.0 {
                available = end_run;
            }

            let fits = size * available / drag;

            if fits >= largest_that_fits {
                continue;
            }

            largest_that_fits = fits;
            worst_segment = Some(i);

            let from_start_cap = if start_is_capped && i == 0 && start_slide > 0.0 {
                start_slide
            } else {
                0.0
            };
            let from_end_cap = if end_is_capped && i == last_index && end_slide < 0.0 {
                -end_slide
            } else {
                0.0
            };

            worst_cap_corner = if ieee_math::max_f64(from_start_cap, from_end_cap) <= 0.0 {
                None
            } else if from_start_cap >= from_end_cap {
                Some(swept[0].from)
            } else {
                Some(swept[last_index].to)
            };
        }
    }

    let whole_run = run_to_ring(swept, swept.len());

    if start_is_capped && end_is_capped && start_run >= whole_run - TOUCHING_RING {
        let last = &swept[last_index];

        for point in 0..point_count {
            let start_slide = slides(&swept[0].start_slide, point);
            let end_slide = slides(&last.end_slide, point);
            let drag = start_slide - end_slide;

            if drag <= 0.0 {
                continue;
            }

            let fits = size * whole_run / drag;

            if fits >= largest_that_fits {
                continue;
            }

            largest_that_fits = fits;
            worst_segment = Some(0);
            worst_cap_corner = Some(if start_slide >= -end_slide {
                swept[0].from
            } else {
                last.to
            });
        }
    }

    let Some(worst_segment) = worst_segment else {
        return Ok(());
    };
    if largest_that_fits > size {
        return Ok(());
    }

    if let Some(corner) = worst_cap_corner {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::CapPullsToolBackPastSegment,
            vec![corner.into(), size.into(), largest_that_fits.into()],
        ));
    }

    let at = &swept[worst_segment];

    Err(BevelAdvisory::new(
        BevelAdvisoryCode::SizeFoldsToolAtCurve,
        vec![
            size.into(),
            ((at.from + at.to) * 0.5).into(),
            largest_that_fits.into(),
        ],
    ))
}
