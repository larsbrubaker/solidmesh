//! The collinear run, what one picked edge means in a CAD package - the whole straight crease it
//! lies in, not the one mesh edge the triangulation happened to cut it into.

use std::sync::Arc;

use solidmesh::topology::{
    bevel_edge_group_resolver, bevel_edge_readings, pick_interpretations, BevelSelectionKind,
    ChainStopReason, FaceGroup, FeatureEdgeChain, FeatureEdgeSet, MatchTolerances,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids;

use crate::split_crease_test_meshes;

/// A tolerance loose enough to walk a brick yawed by five degrees, for the tests that need the
/// yaw to be the only thing in question.
const LOOSE_ANGLE_DEGREES: f64 = 20.0;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

/// A crease cut into three by two plain vertices - no other feature edge on either - is one run.
#[test]
fn a_crease_split_by_plain_vertices_is_one_run() {
    let mesh = split_crease_test_meshes::create_split_crease_bar();
    let features = FeatureEdgeSet::create(&mesh);
    let seed = seed_nearest(&features, Vector3::new(5.0, 0.0, 10.0));

    let run = FeatureEdgeChain::propagate_collinear(&features, seed).unwrap();

    assert_eq!(run.count(), 3);
    assert!(!run.is_closed());
    assert_within(span_in_x(&run), 30.0, 1e-4);
}

/// The split vertex is also a corner of another solid, so five more feature edges stand on it
/// and the tangent walk stops dead - even though exactly one of them carries the crease on.
#[test]
fn a_crease_carries_on_through_a_vertex_covered_in_other_feature_edges() {
    let mesh = split_crease_test_meshes::create_flush_brick_stack(&[0.0]);
    let features = FeatureEdgeSet::create(&mesh);
    let seed = seed_nearest(&features, Vector3::new(5.0, 0.0, 10.0));

    // the junction really is one the tangent walk cannot pass: the cube's own corner
    let end_vertex = features.edges()[seed as usize].end_vertex;
    assert!(features.edges_at_vertex(end_vertex).len() >= 3);
    assert_eq!(
        FeatureEdgeChain::propagate(&features, seed)
            .unwrap()
            .count(),
        1
    );

    let run = FeatureEdgeChain::propagate_collinear(&features, seed).unwrap();

    assert_eq!(run.count(), 2);

    // the cube's own 10 mm crease plus the brick's, one straight 20 mm crease
    assert_within(span_in_x(&run), 20.0, 1e-4);
}

/// The same crease through a REAL boolean union: the run covers all 20 mm of it, however many
/// edges the CSG kernel cut it into and whatever it left lying against it.
#[test]
fn a_crease_split_by_a_real_boolean_resolves_whole() {
    let mesh = split_crease_test_meshes::create_booleaned_flush_bricks();
    let features = FeatureEdgeSet::create(&mesh);
    let seed = seed_containing(&features, Vector3::new(5.0, 0.0, 10.0));

    let run = FeatureEdgeChain::propagate_collinear(&features, seed).unwrap();

    assert!(
        run.count() > 1,
        "the union puts at least one vertex on this crease, so it is more than one mesh edge"
    );

    // the crease of the SHAPE runs the whole 20 mm, whatever the triangulation did to it
    assert_within(span_in_x(&run), 20.0, 1e-4);

    for edge in run.edges() {
        assert_within(edge.from().y, 0.0, 1e-4);
        assert_within(edge.from().z, 10.0, 1e-4);
    }
}

/// A continuation that bends stops the run, and the tolerance is the only thing deciding it.
#[test]
fn a_bend_beyond_the_tolerance_stops_the_run() {
    let mesh = split_crease_test_meshes::create_flush_brick_stack(&[-5.0]);
    let features = FeatureEdgeSet::create(&mesh);
    let seed = seed_nearest(&features, Vector3::new(5.0, 0.0, 10.0));

    assert_eq!(
        FeatureEdgeChain::propagate_collinear(&features, seed)
            .unwrap()
            .count(),
        1
    );
    assert_eq!(
        FeatureEdgeChain::propagate_collinear_with(&features, seed, LOOSE_ANGLE_DEGREES)
            .unwrap()
            .count(),
        2
    );
}

/// Two edges that both carry the crease on stop it, as a corner.
#[test]
fn two_continuations_at_once_stop_the_run_as_a_corner() {
    let mesh = split_crease_test_meshes::create_flush_brick_stack(&[0.0, -5.0]);
    let features = FeatureEdgeSet::create(&mesh);
    let seed = seed_nearest(&features, Vector3::new(5.0, 0.0, 10.0));

    // at the tight tolerance the yawed brick is not a continuation at all
    assert_eq!(
        FeatureEdgeChain::propagate_collinear(&features, seed)
            .unwrap()
            .count(),
        2
    );

    let ambiguous =
        FeatureEdgeChain::propagate_collinear_with(&features, seed, LOOSE_ANGLE_DEGREES).unwrap();

    assert_eq!(ambiguous.count(), 1);
    assert_eq!(ambiguous.end().reason, ChainStopReason::Corner);
}

/// A cube edge is its own run: nothing at either end continues it.
#[test]
fn a_cube_edge_is_its_own_run() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let seed = seed_nearest(&features, Vector3::new(0.0, -10.0, 10.0));

    let run = FeatureEdgeChain::propagate_collinear(&features, seed).unwrap();

    assert_eq!(run.count(), 1);
    assert_eq!(run.start().reason, ChainStopReason::Corner);
    assert_eq!(run.end().reason, ChainStopReason::Corner);
}

/// A run stored at any rung of the ladder is one of the readings its row offers: the readings
/// are seeded from the finest rung while the resolver walks the group's own set, so the list
/// emits the collinear run per rung to be able to contain what the group resolved to.
#[test]
fn a_stored_run_is_found_among_the_readings_whichever_rung_it_was_picked_at() {
    let mesh = Arc::new(split_crease_test_meshes::create_flush_brick_stack(&[0.0]));

    for picked_at_degrees in pick_interpretations::default_flat_angle_ladder_degrees() {
        let features = FeatureEdgeSet::create_with_angle(&mesh, picked_at_degrees);
        let seed = seed_containing(&features, Vector3::new(5.0, 0.0, 10.0));
        let group = bevel_edge_group_resolver::create_from_edge(
            &features,
            seed,
            BevelSelectionKind::Edge,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .unwrap();

        let readings = bevel_edge_readings::for_group_on_mesh(&mesh, &group).unwrap();
        let resolution = bevel_edge_group_resolver::resolve_in(
            &group,
            &mesh,
            None,
            Arc::new(FeatureEdgeSet::create_with_angle(&mesh, picked_at_degrees)),
            MatchTolerances::default(),
        );

        assert_eq!(
            resolution.edges.len(),
            2,
            "the whole crease is two mesh edges, read at {picked_at_degrees} degrees"
        );

        assert!(
            bevel_edge_readings::index_of_current_reading(&readings, &resolution.edges) >= 0,
            "a group picked at {picked_at_degrees} degrees has to be able to say which reading it is on"
        );
    }
}

/// The feature edge whose segment passes through a position.
fn seed_containing(features: &FeatureEdgeSet, position: Vector3) -> i32 {
    for (i, edge) in features.edges().iter().enumerate() {
        let along = edge.end - edge.start;
        let length_squared = along.length_squared();

        if length_squared <= 0.0 {
            continue;
        }

        let t = ((position - edge.start).dot(along) / length_squared).clamp(0.0, 1.0);

        if (edge.start + (along * t) - position).length() < 1e-4 {
            return i as i32;
        }
    }

    panic!("no feature edge passes through {position}");
}

/// How far the run reaches in x, which is what says it covers the whole crease.
fn span_in_x(run: &FeatureEdgeChain) -> f64 {
    let mut low = f64::MAX;
    let mut high = f64::MIN;

    for edge in run.edges() {
        low = low.min(edge.from().x.min(edge.to().x));
        high = high.max(edge.from().x.max(edge.to().x));
    }

    high - low
}

/// The feature edge whose midpoint is closest to a position.
fn seed_nearest(features: &FeatureEdgeSet, midpoint: Vector3) -> i32 {
    let mut best = -1;
    let mut best_distance = f64::MAX;

    for (i, edge) in features.edges().iter().enumerate() {
        let distance = (((edge.start + edge.end) / 2.0) - midpoint).length();

        if distance < best_distance {
            best_distance = distance;
            best = i as i32;
        }
    }

    assert!(
        best_distance <= 1e-3,
        "no feature edge has its midpoint at {midpoint}; the nearest is {best_distance} away"
    );

    best
}
