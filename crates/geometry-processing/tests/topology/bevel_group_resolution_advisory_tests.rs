//! Ports `BevelGroupResolutionAdvisoryTests`: a group that fails to resolve says why as a
//! `BevelAdvisory` (a code and the values it quotes), never as a sentence.
//!
//! Only the four path-corner tests are ported so far (with `PathCornerSet` and
//! `BevelCornerGroupResolver`, Phase 5C gp3); the six mesh-edge tests of the same class
//! (`AMeshWithNoFeatureEdgesFailsNamingTheFlatTolerance` through
//! `AFaceRegionThatNeverTurnsFailsNamingTheCurveTolerance`) join this file when their cube and
//! tube fixtures are ported.

use agg_path::{VertexSource, VertexStorage};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::bevel_corner_group_resolver::{
    create_from_corner, resolve, DEFAULT_DIRECTION_TOLERANCE_DEGREES,
    DEFAULT_DRIFT_FRACTION_OF_EXTENT,
};
use geometry_processing::topology::{BevelCornerGroup, PathCornerSet};
use vector_math::Vector2;

const BEND_THRESHOLD: f64 = 10.0;

fn square(side: f64) -> Vec<Vector2> {
    vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(side, 0.0),
        Vector2::new(side, side),
        Vector2::new(0.0, side),
    ]
}

/// A sawtooth of identical teeth, the 2D reading of the two-cube fixture.
fn comb(teeth: i32) -> Vec<Vector2> {
    const PITCH: f64 = 6.0;
    let mut points = Vec::new();
    for i in 0..teeth {
        points.push(Vector2::new(i as f64 * PITCH, 5.0));
        points.push(Vector2::new((i as f64 * PITCH) + (PITCH / 2.0), 0.0));
    }
    // back along the bottom, far enough below the teeth that nothing down here is near one
    points.push(Vector2::new(
        ((teeth - 1) as f64 * PITCH) + (PITCH / 2.0),
        -10.0,
    ));
    points.push(Vector2::new(0.0, -10.0));
    points
}

fn closed_path(points: &[Vector2]) -> impl VertexSource {
    let mut storage = VertexStorage::new();
    storage.move_to_vector(points[0]);
    for point in &points[1..] {
        storage.line_to_vector(*point);
    }
    storage.close_polygon();
    storage
}

fn index_of_corner_at(corners: &PathCornerSet, position: Vector2) -> i32 {
    corners
        .corners
        .iter()
        .position(|corner| (corner.position - position).length() < 1e-9)
        .map_or(-1, |i| i as i32)
}

fn double_arg(arg: &BevelAdvisoryArg) -> f64 {
    match arg {
        BevelAdvisoryArg::Double(value) => *value,
        other => panic!("expected a double argument, got {other:?}"),
    }
}

#[test]
fn a_path_with_no_corners_fails_naming_the_bend_threshold() {
    const SHARPER_THAN_A_SQUARE: f64 = 100.0;
    let square = closed_path(&square(20.0));
    let group = create_from_corner(&PathCornerSet::create(&square, BEND_THRESHOLD), 0).unwrap();
    let no_corners = PathCornerSet::create(&square, SHARPER_THAN_A_SQUARE);
    assert_eq!(no_corners.count(), 0);

    let resolution = resolve(&group, &no_corners);
    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::PathHasNoCorners,
            vec![SHARPER_THAN_A_SQUARE.into()]
        ))
    );
}

#[test]
fn a_corner_group_with_no_anchor_directions_fails_saying_so() {
    let corners = PathCornerSet::create(&closed_path(&square(20.0)), BEND_THRESHOLD);
    let resolution = resolve(&BevelCornerGroup::default(), &corners);
    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::GroupHasNoAnchorDirections,
            Vec::new()
        ))
    );
}

#[test]
fn a_corner_nothing_matches_fails_quoting_the_gates_it_was_judged_by() {
    let square = square(20.0);
    let corners = PathCornerSet::create(&closed_path(&square), BEND_THRESHOLD);
    let group = create_from_corner(&corners, index_of_corner_at(&corners, Vector2::ZERO)).unwrap();

    let triangle: Vec<Vector2> = square[1..].to_vec();
    let remaining = PathCornerSet::create(&closed_path(&triangle), BEND_THRESHOLD);
    let resolution = resolve(&group, &remaining);
    assert!(!resolution.is_resolved());

    let reason = resolution.failure_reason.unwrap();
    assert_eq!(reason.code(), BevelAdvisoryCode::NoCornerMatchesAnchor);
    assert_eq!(reason.args().len(), 4);
    assert_eq!(
        reason.args()[0],
        BevelAdvisoryArg::Vector2(group.anchor_position)
    );
    assert_eq!(
        reason.args()[1],
        BevelAdvisoryArg::Int(remaining.count() as i32)
    );
    let expected_drift = remaining.extent * DEFAULT_DRIFT_FRACTION_OF_EXTENT;
    assert!((double_arg(&reason.args()[2]) - expected_drift).abs() <= 1e-9);
    assert_eq!(
        reason.args()[3],
        BevelAdvisoryArg::Double(DEFAULT_DIRECTION_TOLERANCE_DEGREES)
    );
}

#[test]
fn a_corner_two_corners_fit_equally_well_fails_quoting_both_scores() {
    let comb = comb(5);
    let corners = PathCornerSet::create(&closed_path(&comb), BEND_THRESHOLD);
    let middle_tooth = Vector2::new(12.0, 5.0);
    let group = create_from_corner(&corners, index_of_corner_at(&corners, middle_tooth)).unwrap();

    // the tooth it was made on filed off, leaving its two identical neighbours either side
    let filed_off: Vec<Vector2> = comb
        .iter()
        .copied()
        .filter(|point| (*point - middle_tooth).length() > 1e-9)
        .collect();
    let resolution = resolve(
        &group,
        &PathCornerSet::create(&closed_path(&filed_off), BEND_THRESHOLD),
    );
    assert!(!resolution.is_resolved());

    let reason = resolution.failure_reason.unwrap();
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::AnchorMatchesMoreThanOneCorner
    );
    assert_eq!(reason.args().len(), 3);
    assert_eq!(
        reason.args()[0],
        BevelAdvisoryArg::Vector2(group.anchor_position)
    );
    assert!((double_arg(&reason.args()[1]) - double_arg(&reason.args()[2])).abs() <= 1e-9);
}
