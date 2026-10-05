//! Where a run-out's columns END - the stations a cut across the run-out is read at, and the slot
//! the top of a column takes (`Bevel/Surface/SurfaceRunOutColumnEnds.cs`, a static class: free
//! functions here). The wall's own leftover test (`ItLapsOverTheWallsOwnLeftover`) lands with a
//! later package.

use std::cmp::Ordering;

use super::surface_mixed_corner_run_out;

/// How near two turns have to be to be one station - an angle, so fixed.
pub const HAIR: f64 = 1e-9;

/// The slot the top of a column (standing `the_top_stands_off` out of the axis) takes among its
/// rows' stand-offs `out_of_the_axis`, from `first_row`: the last row it swallows. `None` where it
/// swallows none - the top stands ABOVE the column's first row, and taking a slot anyway would
/// drop that row with nothing said about it.
pub fn try_the_slot_the_top_takes(
    out_of_the_axis: &[f64],
    first_row: usize,
    the_top_stands_off: f64,
    near_enough: f64,
) -> Option<usize> {
    let mut slot = first_row;
    let mut swallowed = false;

    while slot < out_of_the_axis.len() && out_of_the_axis[slot] > the_top_stands_off - near_enough {
        slot += 1;
        swallowed = true;
    }

    if !swallowed {
        return None;
    }

    Some(slot - 1)
}

/// The stations a cut across the run-out is read at: the base `stations` short of the FIRST
/// crossing a row the wall has not already swallowed makes (and short of the foot), the
/// `also_keep` turns (a second cut's own stations), every crossing short of the foot, and the
/// foot itself - sorted, with stations within a [`HAIR`] merged.
pub fn with_the_crossings_stationed(
    stations: &[f64],
    crosses_at: &[f64],
    the_wall_has_it: &[bool],
    foot_crosses_at: f64,
    also_keep: Option<&[f64]>,
) -> Vec<f64> {
    let mut asking = Vec::with_capacity(stations.len() + crosses_at.len() + 1);

    // The FIRST crossing is where the plane starts cutting, and from there the only stations are
    // the crossings themselves. Named by a row the WALL has not already swallowed by then.
    let mut starts = f64::INFINITY;

    for (row, &crossing) in crosses_at.iter().enumerate() {
        if crossing != f64::INFINITY
            && !the_wall_has_it[row]
            && crossing > HAIR
            && crossing < starts
        {
            starts = crossing;
        }
    }

    for &turn in stations {
        if turn < foot_crosses_at - HAIR && turn < starts - HAIR {
            asking.push(turn);
        }
    }

    // And the turns the caller says must be stationed whatever the crossings say: where a SECOND
    // cut ends the columns from below, its own stations are as much a part of the answer.
    if let Some(also_keep) = also_keep {
        for &turn in also_keep {
            if turn > HAIR && turn < foot_crosses_at - HAIR {
                asking.push(turn);
            }
        }
    }

    for &crossing in crosses_at {
        if crossing != f64::INFINITY && crossing > HAIR && crossing < foot_crosses_at - HAIR {
            asking.push(crossing);
        }
    }

    asking.push(foot_crosses_at);
    asking.sort_by(cs_compare);

    let mut stationed: Vec<f64> = Vec::with_capacity(asking.len());

    for turn in asking {
        if stationed.last().is_none_or(|&last| turn - last > HAIR) {
            stationed.push(turn);
        }
    }

    stationed
}

/// The first row of a column the cut leaves standing at `past`: one past the rows the plane has
/// taken by then. `None` where the taken rows are not the first few - a row still standing above
/// one already gone leaves a hole in the column.
pub fn try_first_row_standing(crosses_at: &[f64], past: f64) -> Option<usize> {
    let mut first_row = 0;
    let mut taken = 0;

    for (row, &crossing) in crosses_at.iter().enumerate() {
        if crossing < past - HAIR {
            taken += 1;
            first_row = row + 1;
        }
    }

    (first_row == taken).then_some(first_row)
}

/// How far short of where the run-out clears the wall a station stops being worth a column: the
/// turn at which the touch row stands within `near_enough` of the wall, at `size`. Zero for no
/// size.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn too_close_to_the_end(near_enough: f64, size: f64) -> f64 {
    if !(size > 0.0) {
        return 0.0;
    }

    surface_mixed_corner_run_out::clears_the_wall()
        - cs_min(1.0, 1.0 / (2.0 - cs_min(1.0, near_enough / size))).acos()
}

/// C#'s `double.CompareTo`, which orders a NaN before every number (`List<double>.Sort`).
fn cs_compare(a: &f64, b: &f64) -> Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        (false, false) => a.partial_cmp(b).unwrap_or(Ordering::Equal),
    }
}

/// C#'s `Math.Min`, which passes a NaN through.
fn cs_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}
