//! Ports `PathSegmentRunTests`: growing a seed segment of a contour into its run of
//! curved-but-close neighbours, under a per-end turning budget and a corner threshold.
//!
//! C#'s `Throws<ArgumentException>` / `Throws<ArgumentOutOfRangeException>` read as the matching
//! `PathSegmentRunError`; the `null` points case is not representable in Rust and is dropped
//! from `a_budget_or_corner_angle_that_is_not_a_number_is_refused`.

use std::collections::HashSet;

use agg_path::VertexStorage;
use geometry_processing::path::path_bevel::{self, Contour};
use geometry_processing::path::path_segment_run::walk;
use geometry_processing::path::{PathSegmentRunError, RunStopReason, SegmentRun};
use vector_math::{math_helper, Vector2};

const RADIUS: f64 = 20.0;

fn square(side: f64) -> Vec<Vector2> {
    vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(side, 0.0),
        Vector2::new(side, side),
        Vector2::new(0.0, side),
    ]
}

fn arc(center: Vector2, start_degrees: f64, sweep_degrees: f64, segments: i32) -> Vec<Vector2> {
    (0..=segments)
        .map(|i| {
            let angle = math_helper::degrees_to_radians(
                start_degrees + (sweep_degrees * i as f64 / segments as f64),
            );
            center + (Vector2::new(angle.cos(), angle.sin()) * RADIUS)
        })
        .collect()
}

fn quarter_arc(segments: i32) -> Vec<Vector2> {
    arc(Vector2::ZERO, 0.0, 90.0, segments)
}

fn circle72() -> Vec<Vector2> {
    arc(Vector2::ZERO, 0.0, 360.0 - (360.0 / 72.0), 71)
}

fn flattened_quarter_circle() -> Contour {
    // the control point offset that makes a cubic bezier hug a circular arc: 4/3 tan(sweep / 4)
    let k = 4.0 / 3.0 * (std::f64::consts::PI / 4.0 / 4.0).tan();
    let mut source = VertexStorage::new();
    source.move_to(RADIUS, 0.0);
    append_arc(&mut source, 0.0, std::f64::consts::PI / 4.0, k);
    append_arc(
        &mut source,
        std::f64::consts::PI / 4.0,
        std::f64::consts::PI / 2.0,
        k,
    );
    path_bevel::cleaned_contours(&source).remove(0)
}

fn append_arc(source: &mut VertexStorage, start_angle: f64, end_angle: f64, k: f64) {
    let start = Vector2::new(start_angle.cos(), start_angle.sin()) * RADIUS;
    let end = Vector2::new(end_angle.cos(), end_angle.sin()) * RADIUS;
    let start_tangent = Vector2::new(-start_angle.sin(), start_angle.cos()) * RADIUS;
    let end_tangent = Vector2::new(-end_angle.sin(), end_angle.cos()) * RADIUS;
    source.curve4_vector(start + (start_tangent * k), end - (end_tangent * k), end);
}

fn rounded_rectangle(
    width: f64,
    height: f64,
    corner_radius: f64,
    round_segments: i32,
    side_segments: i32,
) -> Vec<Vector2> {
    let right = width / 2.0;
    let top = height / 2.0;
    let centers = [
        Vector2::new(right - corner_radius, -top + corner_radius),
        Vector2::new(right - corner_radius, top - corner_radius),
        Vector2::new(-right + corner_radius, top - corner_radius),
        Vector2::new(-right + corner_radius, -top + corner_radius),
    ];
    let mut points = Vec::new();
    for corner in 0..4 {
        for i in 0..=round_segments {
            let angle = math_helper::degrees_to_radians(
                -90.0 + (corner as f64 * 90.0) + (90.0 * i as f64 / round_segments as f64),
            );
            points.push(centers[corner] + (Vector2::new(angle.cos(), angle.sin()) * corner_radius));
        }
        let side_start = points[points.len() - 1];
        let next_center = centers[(corner + 1) % 4];
        let next_angle = math_helper::degrees_to_radians(corner as f64 * 90.0);
        let side_end =
            next_center + (Vector2::new(next_angle.cos(), next_angle.sin()) * corner_radius);
        for i in 1..side_segments {
            points.push(side_start + ((side_end - side_start) * (i as f64 / side_segments as f64)));
        }
    }
    points
}

fn s_curve(segments_per_arc: i32) -> Vec<Vector2> {
    let first = arc(Vector2::ZERO, 180.0, 90.0, segments_per_arc);
    let second = arc(
        Vector2::new(0.0, -2.0 * RADIUS),
        90.0,
        -90.0,
        segments_per_arc,
    );
    let mut points = first;
    points.extend(second.into_iter().skip(1));
    points
}

fn inscribed_polygon(arc_degrees: &[f64]) -> Vec<Vector2> {
    let mut points = Vec::with_capacity(arc_degrees.len());
    let mut angle = 0.0;
    for arc in arc_degrees {
        let radians = math_helper::degrees_to_radians(angle);
        points.push(Vector2::new(radians.cos(), radians.sin()) * RADIUS);
        angle += arc;
    }
    points
}

fn backward_count(run: &SegmentRun, seed_segment: i32) -> i32 {
    let segment_count = run.segment_count as i32;
    ((seed_segment - run.first_segment as i32) % segment_count + segment_count) % segment_count
}

fn forward_count(run: &SegmentRun, seed_segment: i32) -> i32 {
    run.count as i32 - 1 - backward_count(run, seed_segment)
}

fn arc_degrees_covered(points: &[Vector2], center: Vector2, run: &SegmentRun) -> f64 {
    let first = points[run.first_segment % points.len()] - center;
    let last = points[(run.first_segment + run.count) % points.len()] - center;
    math_helper::radians_to_degrees(last.y.atan2(last.x) - first.y.atan2(first.x)).abs()
}

fn largest_bend_degrees(points: &[Vector2]) -> f64 {
    let mut largest: f64 = 0.0;
    for i in 1..points.len() - 1 {
        let incoming = points[i] - points[i - 1];
        let outgoing = points[i + 1] - points[i];
        largest = largest.max(
            math_helper::radians_to_degrees(incoming.cross(outgoing).atan2(incoming.dot(outgoing)))
                .abs(),
        );
    }
    largest
}

fn indices(run: &SegmentRun) -> HashSet<usize> {
    run.segment_indices().collect()
}

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

#[test]
fn a_squares_edge_is_its_own_run_because_every_neighbour_is_across_a_corner() {
    let run = walk(&square(20.0), true, 0, 30.0, 30.0).unwrap();
    assert_eq!(run.count, 1);
    assert_near(run.turning_degrees, 0.0, 1e-9);
    assert_near(run.signed_turning_degrees, 0.0, 1e-9);
    assert_eq!(run.backward_stop, RunStopReason::Corner);
    assert_eq!(run.forward_stop, RunStopReason::Corner);
    assert!(!run.is_loop());
    assert_eq!(run.segment_indices().collect::<Vec<_>>(), vec![0]);
}

#[test]
fn a_corner_stops_a_run_that_has_budget_left_to_spare() {
    let run = walk(&square(20.0), true, 2, 400.0, 90.0).unwrap();
    assert_eq!(run.count, 1);
    assert_eq!(run.backward_stop, RunStopReason::Corner);
    assert_eq!(run.forward_stop, RunStopReason::Corner);
}

#[test]
fn the_same_budget_covers_the_same_arc_whichever_resolution_it_was_flattened_at() {
    const BUDGET: f64 = 30.0;
    let coarse = quarter_arc(10);
    let fine = quarter_arc(250);
    let flattened = flattened_quarter_circle();
    let coarse_run = walk(&coarse, false, 5, BUDGET, 30.0).unwrap();
    let fine_run = walk(&fine, false, 125, BUDGET, 30.0).unwrap();
    let flattened_run = walk(
        &flattened.points,
        flattened.is_closed,
        (flattened.points.len() / 2) as i32,
        BUDGET,
        30.0,
    )
    .unwrap();

    assert_eq!(
        coarse_run.count, 7,
        "three 9 degree steps fit in each end's 30"
    );
    assert_eq!(
        fine_run.count, 167,
        "83 steps of 0.36 degrees an end fit in 30"
    );

    let fixtures = [
        (
            arc_degrees_covered(&coarse, Vector2::ZERO, &coarse_run),
            largest_bend_degrees(&coarse),
        ),
        (
            arc_degrees_covered(&fine, Vector2::ZERO, &fine_run),
            largest_bend_degrees(&fine),
        ),
        (
            arc_degrees_covered(&flattened.points, Vector2::ZERO, &flattened_run),
            largest_bend_degrees(&flattened.points),
        ),
    ];
    for (covered, step) in fixtures {
        assert!(
            covered >= (2.0 * BUDGET) - (2.0 * step) - 1e-9,
            "each end spends its whole budget bar the step it could not afford"
        );
        assert!(
            covered <= (2.0 * BUDGET) + step + 1e-9,
            "and neither end may overspend"
        );
    }
    assert!((fixtures[0].0 - fixtures[1].0).abs() <= 3.0 * fixtures[0].1);
    assert!((fixtures[2].0 - fixtures[1].0).abs() <= 3.0 * fixtures[2].1);
}

#[test]
fn a_budget_larger_than_the_whole_arc_runs_off_both_ends_of_it() {
    let arc = quarter_arc(10);
    let run = walk(&arc, false, 5, 100.0, 30.0).unwrap();
    assert_eq!(run.count, 10);
    assert_eq!(run.first_segment, 0);
    assert_near(run.turning_degrees, 81.0, 1e-9);
    assert_eq!(run.backward_stop, RunStopReason::End);
    assert_eq!(run.forward_stop, RunStopReason::End);
    assert!(!run.is_loop());
}

#[test]
fn a_circle_closes_into_a_loop_once_each_end_can_afford_half_of_it() {
    let circle = circle72();
    assert_eq!(circle.len(), 72);
    for budget in [180.0, 400.0] {
        let run = walk(&circle, true, 17, budget, 30.0).unwrap();
        assert_eq!(run.count, 72);
        assert!(run.is_loop());
        assert_eq!(run.backward_stop, RunStopReason::Loop);
        assert_eq!(run.forward_stop, RunStopReason::Loop);
        assert_near(run.turning_degrees, 360.0, 1e-6);
        assert_near(run.signed_turning_degrees, 360.0, 1e-6);
        assert_eq!(indices(&run).len(), 72);
    }
}

#[test]
fn a_circle_run_reaches_equally_far_either_side_of_its_seed() {
    const SEED: i32 = 30;
    let circle = circle72();
    let run = walk(&circle, true, SEED, 45.0, 30.0).unwrap();
    assert_eq!(run.count, 19);
    assert!(!run.is_loop());
    assert_eq!(run.backward_stop, RunStopReason::BudgetSpent);
    assert_eq!(run.forward_stop, RunStopReason::BudgetSpent);
    assert_near(run.turning_degrees, 90.0, 1e-6);
    assert!((backward_count(&run, SEED) - forward_count(&run, SEED)).abs() <= 1);
}

#[test]
fn a_seed_on_a_flat_side_reaches_the_same_way_into_both_adjacent_rounds() {
    const ROUND_SEGMENTS: usize = 9;
    const SIDE_SEGMENTS: usize = 4;
    let points = rounded_rectangle(40.0, 30.0, 5.0, ROUND_SEGMENTS as i32, SIDE_SEGMENTS as i32);
    let run = walk(&points, true, 10, 45.0, 20.0).unwrap();
    let indices = indices(&run);
    assert_eq!(run.backward_stop, RunStopReason::BudgetSpent);
    assert_eq!(run.forward_stop, RunStopReason::BudgetSpent);
    assert_near(run.turning_degrees, 90.0, 1e-6);
    assert!((9..9 + SIDE_SEGMENTS).all(|i| indices.contains(&i)));
    assert_eq!(
        (0..ROUND_SEGMENTS).filter(|i| indices.contains(i)).count(),
        5
    );
    assert_eq!(
        (13..13 + ROUND_SEGMENTS)
            .filter(|i| indices.contains(i))
            .count(),
        5
    );
    assert!(!(0..ROUND_SEGMENTS).all(|i| indices.contains(&i)));
}

#[test]
fn a_budget_past_a_whole_round_carries_on_down_the_free_straight_to_the_next_round() {
    const ROUND_SEGMENTS: usize = 9;
    const SIDE_SEGMENTS: usize = 4;
    const SEED: i32 = 10;
    let points = rounded_rectangle(40.0, 30.0, 5.0, ROUND_SEGMENTS as i32, SIDE_SEGMENTS as i32);
    let run = walk(&points, true, SEED, 100.0, 20.0).unwrap();
    let indices = indices(&run);
    assert_eq!(run.backward_stop, RunStopReason::BudgetSpent);
    assert_eq!(run.forward_stop, RunStopReason::BudgetSpent);
    assert!(!run.is_loop());
    assert_near(run.turning_degrees, 190.0, 1e-6);
    assert!((0..ROUND_SEGMENTS).all(|i| indices.contains(&i)));
    assert!((13..13 + ROUND_SEGMENTS).all(|i| indices.contains(&i)));
    assert!((48..48 + SIDE_SEGMENTS).all(|i| indices.contains(&i)));
    assert!((22..22 + SIDE_SEGMENTS).all(|i| indices.contains(&i)));
    assert_eq!(
        (39..39 + ROUND_SEGMENTS)
            .filter(|i| indices.contains(i))
            .count(),
        1
    );
    assert_eq!(
        (26..26 + ROUND_SEGMENTS)
            .filter(|i| indices.contains(i))
            .count(),
        1
    );
    assert_eq!(backward_count(&run, SEED), 15);
    assert_eq!(forward_count(&run, SEED), 16);
}

#[test]
fn reversing_the_contour_grows_the_mirror_of_the_same_run() {
    const SEED: i32 = 10;
    let points = rounded_rectangle(40.0, 30.0, 5.0, 9, 4);
    let reversed: Vec<Vector2> = points.iter().rev().copied().collect();
    let n = points.len() as i32;
    let mirror = |segment: i32| ((n - 2 - segment) % n + n) % n;

    let run = walk(&points, true, SEED, 100.0, 20.0).unwrap();
    let mirrored_run = walk(&reversed, true, mirror(SEED), 100.0, 20.0).unwrap();

    let mut mirrored: Vec<i32> = mirrored_run.segment_indices().map(|i| i as i32).collect();
    mirrored.sort();
    let mut expected: Vec<i32> = run.segment_indices().map(|i| mirror(i as i32)).collect();
    expected.sort();
    assert_eq!(mirrored, expected);
    assert_near(mirrored_run.turning_degrees, run.turning_degrees, 1e-9);
    assert_near(
        mirrored_run.signed_turning_degrees,
        -run.signed_turning_degrees,
        1e-9,
    );
    assert_eq!(
        backward_count(&mirrored_run, mirror(SEED)),
        forward_count(&run, SEED)
    );
    assert_eq!(
        forward_count(&mirrored_run, mirror(SEED)),
        backward_count(&run, SEED)
    );
    assert_eq!(mirrored_run.backward_stop, run.forward_stop);
    assert_eq!(mirrored_run.forward_stop, run.backward_stop);
}

#[test]
fn a_seed_on_the_last_segment_of_an_open_contour_only_grows_backwards() {
    let arc = quarter_arc(10);
    let run = walk(&arc, false, 9, 30.0, 30.0).unwrap();
    assert_eq!(run.forward_stop, RunStopReason::End);
    assert_eq!(run.backward_stop, RunStopReason::BudgetSpent);
    assert_eq!(run.count, 4);
    assert_eq!(run.first_segment, 6);
    assert_near(run.turning_degrees, 27.0, 1e-9);
}

#[test]
fn a_duplicated_point_does_not_hide_the_corner_it_sits_on() {
    let points = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];
    let run = walk(&points, true, 0, 30.0, 30.0).unwrap();
    assert_eq!(run.count, 1);
    assert_eq!(run.backward_stop, RunStopReason::Corner);
    assert_eq!(run.forward_stop, RunStopReason::Corner);
    assert_near(run.turning_degrees, 0.0, 1e-9);
}

#[test]
fn an_exactly_straight_polyline_accumulates_no_phantom_turning() {
    const SEGMENTS: i32 = 10000;
    let points: Vec<Vector2> = (0..=SEGMENTS)
        .map(|i| Vector2::new(i as f64, 3.0 * i as f64))
        .collect();
    let run = walk(&points, false, SEGMENTS / 2, 30.0, 30.0).unwrap();
    assert_eq!(run.count, SEGMENTS as usize);
    assert!(run.turning_degrees < 1e-6);
    assert!(run.signed_turning_degrees.abs() < 1e-6);
}

#[test]
fn a_corner_at_the_seam_leaves_every_segment_in_a_run_that_is_still_not_a_loop() {
    let points = inscribed_polygon(&[90.0, 90.0, 30.0, 30.0, 30.0, 30.0, 40.0, 20.0]);
    let run = walk(&points, true, 4, 1000.0, 80.0).unwrap();
    assert_eq!(run.count, 8);
    assert!(!run.is_loop());
    assert_eq!(run.backward_stop, RunStopReason::Corner);
    assert_eq!(run.forward_stop, RunStopReason::Corner);
    assert_eq!(run.first_segment, 1);
    assert_near(run.turning_degrees, 270.0, 1e-6);
    assert_eq!(indices(&run).len(), 8);
}

#[test]
fn a_run_can_straddle_the_seam_without_closing_into_a_loop() {
    let circle = circle72();
    let run = walk(&circle, true, 30, 177.5, 30.0).unwrap();
    assert_eq!(run.count, 71);
    assert_eq!(run.first_segment, 67);
    assert!(!run.is_loop());
    assert_eq!(run.backward_stop, RunStopReason::BudgetSpent);
    assert_eq!(run.forward_stop, RunStopReason::BudgetSpent);
    assert_eq!(indices(&run).len(), 71);
    assert_near(run.turning_degrees, 350.0, 1e-6);
}

#[test]
fn an_s_curves_inflection_does_not_refund_the_budget_its_first_half_spent() {
    const PER_ARC: i32 = 9;
    let points = s_curve(PER_ARC);
    let whole = walk(&points, false, 8, 100.0, 30.0).unwrap();
    assert_eq!(whole.count, (2 * PER_ARC) as usize);
    assert_near(whole.turning_degrees, 160.0, 1e-6);
    assert!(whole.signed_turning_degrees.abs() < 1e-6);

    let budgeted = walk(&points, false, 8, 45.0, 30.0).unwrap();
    assert_eq!(budgeted.backward_stop, RunStopReason::BudgetSpent);
    assert_eq!(budgeted.forward_stop, RunStopReason::BudgetSpent);
    assert_near(budgeted.turning_degrees, 80.0, 1e-6);
    assert!(budgeted.signed_turning_degrees.abs() < 1e-6);
}

#[test]
fn a_contour_with_no_segments_or_a_seed_that_is_not_one_of_them_is_refused() {
    let arc = quarter_arc(10);
    assert!(matches!(
        walk(&arc, false, 10, 30.0, 30.0),
        Err(PathSegmentRunError::SeedOutOfRange { .. })
    ));
    assert!(matches!(
        walk(&arc, false, -1, 30.0, 30.0),
        Err(PathSegmentRunError::SeedOutOfRange { .. })
    ));
    assert_eq!(
        walk(&[Vector2::ZERO], false, 0, 30.0, 30.0),
        Err(PathSegmentRunError::TooFewPoints)
    );
}

#[test]
fn a_budget_or_corner_angle_that_is_not_a_number_is_refused() {
    let arc = quarter_arc(10);
    for (budget, corner) in [
        (f64::NAN, 30.0),
        (30.0, f64::NAN),
        (-1.0, 30.0),
        (30.0, -1.0),
    ] {
        assert!(matches!(
            walk(&arc, false, 5, budget, corner),
            Err(PathSegmentRunError::NotAnAngle { .. })
        ));
    }
}
