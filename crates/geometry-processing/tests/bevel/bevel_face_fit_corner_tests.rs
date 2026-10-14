//! The face-width check at corners - an obstruction standing
//! on the face is seen, runs meeting at a corner are the corner's business, a face narrowing to a
//! convex edge is admitted, and parallel runs too close, or a feature narrow along its whole
//! length, still refuse.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_face_fit_check::GroupCheck;
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode, BevelFaceFit};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::Mesh;
use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector3;

use crate::bevel_obstruction_test_meshes::create_rib_ending_at;
use crate::topology_test_meshes::create_isosceles_prism;

const CUBE_SIZE: f64 = 20.0;
const RADIUS: f64 = 5.0;
const APEX_DEGREES: f64 = 30.0;
const PRISM_LENGTH: f64 = 20.0;

fn spread() -> f64 {
    PRISM_LENGTH * (degrees_to_radians(APEX_DEGREES) / 2.0).tan()
}

fn fit_of(part: Mesh, groups: &[BevelEdgeGroup], size: f64) -> (BevelFaceFit, usize) {
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(groups, &part);
    let count = resolutions.len();
    (
        BevelFaceFit::create(
            Arc::new(part),
            Rc::new(resolutions),
            true,
            size,
            DEFAULT_SURFACE_TOLERANCE_DEGREES,
            None,
        ),
        count,
    )
}

fn fitted_double(reason: &geometry_processing::bevel::BevelAdvisory) -> f64 {
    match reason.args()[2] {
        BevelAdvisoryArg::Double(fits) => fits,
        ref other => panic!("{other:?}"),
    }
}

/// A rib is seen whether or not its crease ends on the sample line, for a rib ending at
/// `rib_ends_at` (`what` names the case).
fn a_rib_is_seen_whether_or_not_its_crease_ends_on_the_sample_line(rib_ends_at: f64, what: &str) {
    let part = create_rib_ending_at(rib_ends_at);
    assert!(part.is_manifold());
    let group = top_edge(&part);
    let (fit, _) = fit_of(part, &[group], RADIUS);
    let check: GroupCheck = fit.try_check_group_fully(0);
    assert!(check.fits(), "{what}: {:?}", check.failure);
    assert!(
        !check.standing_on_a_face.is_empty(),
        "{what}: the rib is not seen"
    );
}

#[test]
fn a_rib_is_seen_whether_or_not_its_crease_ends_on_the_sample_line_at_the_vertex() {
    a_rib_is_seen_whether_or_not_its_crease_ends_on_the_sample_line(
        10.0,
        "the walk aimed straight at the vertex where the crease dies",
    );
}

#[test]
fn a_rib_is_seen_whether_or_not_its_crease_ends_on_the_sample_line_clear_of_its_end() {
    a_rib_is_seen_whether_or_not_its_crease_ends_on_the_sample_line(
        9.99,
        "the walk crossed the crease's body a hundredth clear of its end",
    );
}

/// Two runs meeting at an acute corner are the corner's business and not a collision.
#[test]
fn two_runs_meeting_at_an_acute_corner_are_the_corners_business_and_not_a_collision() {
    let prism = create_isosceles_prism(APEX_DEGREES, PRISM_LENGTH, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&prism);
    let top = CUBE_SIZE / 2.0;
    let groups = [
        group(
            &features,
            Vector3::new(0.0, 0.0, top),
            Vector3::new(PRISM_LENGTH, spread(), top),
        ),
        group(
            &features,
            Vector3::new(0.0, 0.0, top),
            Vector3::new(PRISM_LENGTH, -spread(), top),
        ),
    ];
    let (fit, count) = fit_of(prism, &groups, RADIUS);
    for i in 0..count {
        let checked = fit.try_check_group(i);
        assert!(checked.is_ok(), "{checked:?}");
    }
}

/// A face narrowing to a convex edge at a shared corner is admitted.
#[test]
fn a_face_narrowing_to_a_convex_edge_at_a_shared_corner_is_admitted() {
    let prism = create_isosceles_prism(APEX_DEGREES, PRISM_LENGTH, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&prism);
    let top = CUBE_SIZE / 2.0;
    let groups = [group(
        &features,
        Vector3::new(0.0, 0.0, top),
        Vector3::new(PRISM_LENGTH, spread(), top),
    )];
    let (fit, _) = fit_of(prism, &groups, RADIUS);
    let checked = fit.try_check_group(0);
    assert!(checked.is_ok(), "{checked:?}");
}

/// Two parallel runs closer than their setbacks still refuse.
#[test]
fn two_parallel_runs_closer_than_their_setbacks_still_refuse() {
    let width = 4.0;
    let plate = create_cube(width, CUBE_SIZE, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&plate);
    let half = CUBE_SIZE / 2.0;
    let groups = [
        group(
            &features,
            Vector3::new(-width / 2.0, -half, half),
            Vector3::new(-width / 2.0, half, half),
        ),
        group(
            &features,
            Vector3::new(width / 2.0, -half, half),
            Vector3::new(width / 2.0, half, half),
        ),
    ];
    let (fit, _) = fit_of(plate, &groups, RADIUS);
    let reason = fit
        .try_check_group(0)
        .expect_err("two setbacks of 5 on a 4 wide face collide");
    assert_eq!(reason.code(), BevelAdvisoryCode::SizeCollidesAcrossFace);
    assert!((fitted_double(&reason) - width / 2.0).abs() <= 1e-6);
}

/// A feature narrow along its whole length is refused though every blocker shares its corner.
#[test]
fn a_feature_narrow_along_its_whole_length_is_refused_though_every_blocker_shares_its_corner() {
    let thin = 0.2;
    let post = create_cube(thin, thin, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&post);
    let half = CUBE_SIZE / 2.0;
    let groups = [group(
        &features,
        Vector3::new(-thin / 2.0, thin / 2.0, half),
        Vector3::new(thin / 2.0, thin / 2.0, half),
    )];
    let (fit, _) = fit_of(post, &groups, RADIUS);
    let reason = fit
        .try_check_group(0)
        .expect_err("a 0.2 post has no room for a radius of 5");
    assert!(fitted_double(&reason) < RADIUS);
}

fn top_edge(part: &Mesh) -> BevelEdgeGroup {
    group(
        &FeatureEdgeSet::create(part),
        Vector3::new(0.0, 0.0, CUBE_SIZE),
        Vector3::new(0.0, CUBE_SIZE, CUBE_SIZE),
    )
}

fn group(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    let i = features
        .edges()
        .iter()
        .position(|edge| {
            ((edge.start - from).length() < 1e-5 && (edge.end - to).length() < 1e-5)
                || ((edge.start - to).length() < 1e-5 && (edge.end - from).length() < 1e-5)
        })
        .unwrap_or_else(|| panic!("the fixture has no feature edge from {from:?} to {to:?}"));
    bevel_edge_group_resolver::create_from_edge(
        features,
        i as i32,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap()
}
