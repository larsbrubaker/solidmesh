//! Where along a partnered run its sections are taken, for [`super::bevel_partnered_bead`].

use super::bevel_partnered_bead::{
    BevelPartneredBeadEnd, BevelPartneredBeadRun, BevelPartneredBeadStation,
};
use super::bevel_partnered_bead_tolerances::CUT_BY;
use super::BevelRunOutBound;

/// Every station along the run, past its start, along its crease and past its end, sorted; `ends`
/// is the run's `[start, end]` pair, which each station's `past_end` indexes.
pub fn stations(
    run: &BevelPartneredBeadRun,
    ends: &[BevelPartneredBeadEnd; 2],
) -> Vec<BevelPartneredBeadStation> {
    let [start, end] = ends;
    let spacing = spacing(run);

    let start_along = run.along(run.edges[0].from());
    let end_along = run.along(run.edges[run.edges.len() - 1].to());

    let start_reach = reach(start, spacing);
    let end_reach = reach(end, spacing);

    let mut alongs = Vec::new();

    let start_steps = steps(start_reach, spacing);

    for i in (1..=start_steps).rev() {
        alongs.push(start_along - (start_reach * f64::from(i) / f64::from(start_steps)));
    }

    alongs.push(start_along);

    let crease_steps = steps(end_along - start_along, spacing);

    for i in 1..crease_steps {
        alongs.push(
            start_along + ((end_along - start_along) * f64::from(i) / f64::from(crease_steps)),
        );
    }

    alongs.push(end_along);

    let end_steps = steps(end_reach, spacing);

    for i in 1..=end_steps {
        alongs.push(end_along + (end_reach * f64::from(i) / f64::from(end_steps)));
    }

    add_apart(
        &mut alongs,
        run.second_face_from,
        start_along,
        end_along,
        spacing,
    );
    add_apart(
        &mut alongs,
        run.second_face_to,
        start_along,
        end_along,
        spacing,
    );

    let lowest = start_along - start_reach;
    let highest = end_along + end_reach;
    add_the_across_plane_crossings(run, start, &mut alongs, lowest, highest, spacing);
    add_the_across_plane_crossings(run, end, &mut alongs, lowest, highest, spacing);

    alongs.sort_by(f64::total_cmp);

    alongs
        .into_iter()
        .map(|along| {
            BevelPartneredBeadStation::new(
                along,
                past_the_end(run, start_along, end_along, along),
                along <= start_along + CUT_BY || along >= end_along - CUT_BY,
            )
        })
        .collect()
}

/// Adds `along` where it is strictly inside `lowest..highest` and not within a thousandth of the
/// spacing of a station already there.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn add_apart(alongs: &mut Vec<f64>, along: f64, lowest: f64, highest: f64, spacing: f64) {
    if !(along > lowest) || !(along < highest) {
        return;
    }

    let apart = spacing / 1000.0;

    if alongs.iter().any(|already| (already - along).abs() < apart) {
        return;
    }

    alongs.push(along);
}

/// Which end (`0` the start, `1` the end) closes the station at `along`, or `None` for one inside
/// both the crease and the second face's span.
pub fn past_the_end(
    run: &BevelPartneredBeadRun,
    start_along: f64,
    end_along: f64,
    along: f64,
) -> Option<usize> {
    if along <= start_along + CUT_BY || along <= run.second_face_from + CUT_BY {
        return Some(0);
    }

    if along >= end_along - CUT_BY || along >= run.second_face_to - CUT_BY {
        return Some(1);
    }

    None
}

/// A station wherever an arc point crosses the across plane an end stops on, so the section's cut
/// lands on a station.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn add_the_across_plane_crossings(
    run: &BevelPartneredBeadRun,
    end: &BevelPartneredBeadEnd,
    alongs: &mut Vec<f64>,
    lowest: f64,
    highest: f64,
    spacing: f64,
) {
    if !end.ends_on_the_across_plane {
        return;
    }

    let closing = run.direction.dot(end.across_normal);

    if !(closing.abs() > 1e-9) {
        return;
    }

    let at_the_origin = (run.origin(0.0) - end.across_point).dot(end.across_normal);
    let across_x = run.x_axis.dot(end.across_normal);
    let across_y = run.y_axis.dot(end.across_normal);

    for point in &run.arc {
        add_apart(
            alongs,
            -(at_the_origin + (across_x * point.x) + (across_y * point.y)) / closing,
            lowest,
            highest,
            spacing,
        );
    }
}

/// The arc's own chord, which stations along the run are spaced at.
pub fn spacing(run: &BevelPartneredBeadRun) -> f64 {
    2.0 * run.size * (std::f64::consts::PI / 2.0 / f64::from(run.point_count - 1)).sin()
}

/// How far past the crease an end's stations go - one spacing more against a wall.
pub fn reach(end: &BevelPartneredBeadEnd, spacing: f64) -> f64 {
    if end.bound == BevelRunOutBound::ConcaveEdge {
        end.distance + spacing
    } else {
        end.distance
    }
}

/// How many spacings cover `reach` (at least one, where there is any reach).
pub fn steps(reach: f64, spacing: f64) -> i32 {
    if reach > 0.0 {
        1.max((reach / spacing).ceil() as i32)
    } else {
        0
    }
}
