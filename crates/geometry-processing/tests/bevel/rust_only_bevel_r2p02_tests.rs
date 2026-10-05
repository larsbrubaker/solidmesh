//! Rust-only checks of the bevel core's R2.P02 package pieces no ported C# test reaches on its own:
//! the bounded searches over a segment and a triangle, their bound and their meter.

use geometry_processing::bevel::level_line::bounded_maximum;
use geometry_processing::bevel::level_line::{FieldCounters, FieldSample, WorkError};
use vector_math::Vector3;

/// The distance to the origin less 1: 1-Lipschitz, least at the origin.
fn ball(point: Vector3) -> Result<FieldSample, WorkError> {
    let distance = point.length();
    Ok(FieldSample {
        distance,
        value: distance - 1.0,
        nearest: Vector3::ZERO,
        ..FieldSample::default()
    })
}

/// A ridge: the distance to the nearer of two points 4 apart on x, peaking half way between.
fn two_balls(point: Vector3) -> Result<FieldSample, WorkError> {
    let left = (point - Vector3::new(-2.0, 0.0, 0.0)).length();
    let right = (point - Vector3::new(2.0, 0.0, 0.0)).length();
    let (distance, nearest, chain) = if left <= right {
        (left, Vector3::new(-2.0, 0.0, 0.0), 0)
    } else {
        (right, Vector3::new(2.0, 0.0, 0.0), 1)
    };
    Ok(FieldSample {
        distance,
        value: distance,
        nearest,
        chain,
        ..FieldSample::default()
    })
}

#[test]
fn rust_only_the_segment_search_finds_the_ridge_between_two_balls() {
    let found = bounded_maximum::over_segment(
        &two_balls,
        Vector3::new(-2.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        1e-6,
        f64::NEG_INFINITY,
        None,
    )
    .expect("no budget in force");
    assert!((found.value - 2.0).abs() <= 1e-6);
    assert!(found.point.x.abs() <= 1e-5);
    // Nothing beats a floor above the peak: the floor comes back at the segment's start.
    let floor = bounded_maximum::over_segment(
        &two_balls,
        Vector3::new(-2.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        1e-6,
        5.0,
        None,
    )
    .expect("no budget in force");
    assert_eq!(
        (floor.value, floor.point),
        (5.0, Vector3::new(-2.0, 0.0, 0.0))
    );
}

#[test]
fn rust_only_the_triangle_search_reads_corners_and_stops_above() {
    let a = Vector3::new(1.0, 0.0, 0.0);
    let b = Vector3::new(0.0, 3.0, 0.0);
    let c = Vector3::new(-1.0, 0.0, 0.0);
    // The ball's field is greatest at the far corner b.
    let found = bounded_maximum::over_triangle(
        &ball,
        a,
        b,
        c,
        1e-6,
        f64::INFINITY,
        f64::NEG_INFINITY,
        None,
        None,
    )
    .expect("no budget in force");
    assert_eq!((found.value, found.point), (2.0, b));
    // Asked only whether it rises above 0.5, the first cell already says so.
    let early =
        bounded_maximum::over_triangle(&ball, a, b, c, 1e-6, 0.5, f64::NEG_INFINITY, None, None)
            .expect("no budget in force");
    assert!(early.value > 0.5);
    // The island peak of the ridge over a triangle straddling it.
    let island = bounded_maximum::island_peak(
        &two_balls,
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        1e-3,
        1.0,
        None,
        None,
    )
    .expect("no budget in force");
    assert!(island.value > 1.0);
}

#[test]
#[should_panic(expected = "positive tolerance")]
fn rust_only_a_search_without_a_tolerance_is_a_callers_bug() {
    let _ = bounded_maximum::over_segment(&ball, Vector3::ZERO, Vector3::UNIT_X, 0.0, 0.0, None);
}

#[test]
fn rust_only_the_cell_bound_is_the_lesser_and_root_cells_are_tallied() {
    assert_eq!(bounded_maximum::sharpen(3.0, 2.0, 1.0, 0).ok(), Some(2.0));
    assert_eq!(bounded_maximum::sharpen(1.5, 4.0, 1.0, 0).ok(), Some(1.5));

    // The meter switch is process-wide, so this test leaves it as it is (off unless a meter test
    // has it on) and reads only what does not depend on it: the root-cell tally is per thread and
    // counts whenever it is called.
    let before = FieldCounters::with(|t| t.root_cells.iter().sum::<i64>());
    FieldCounters::note_root_cell(true, true, false);
    FieldCounters::note_root_cell(false, false, true);
    let (cells, both, retired) = FieldCounters::with(|t| {
        (
            t.root_cells.iter().sum::<i64>(),
            t.root_both_and_searched.iter().sum::<i64>(),
            t.root_retired.iter().sum::<i64>(),
        )
    });
    assert_eq!(cells, before + 2);
    assert!(both >= 1 && retired >= 1);
}
