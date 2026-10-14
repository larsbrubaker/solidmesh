//! Rust-only tests for slice 12's units that have no test class of their own (`BevelCornerVoids`,
//! `BevelMixedCorners`, `BevelPartneredBead.ConcaveRunsAcrossGroups`): the corner void's memo and
//! refusals, the crowded mixed corner's warning and its count by mesh edge, and the concave runs a
//! rebuild's partners are read from.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_mixed_corners::{capped_corner, describe};
use geometry_processing::bevel::bevel_partnered_bead_partners::concave_runs_across_groups;
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelCornerVoids};
use geometry_processing::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, ChainEdge, EdgeConvexity, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3;

/// A resolved group selecting `edges`, read among a cube's features (which nothing here asks).
fn resolution(edges: Vec<FeatureEdge>) -> BevelEdgeGroupResolution {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    BevelEdgeGroupResolution {
        group: BevelEdgeGroup::default(),
        features: Arc::new(FeatureEdgeSet::create(&cube)),
        failure_reason: None,
        seed_edge_index: -1,
        edges,
        chain: None,
        face_group: None,
    }
}

/// An edge from the shared vertex 0 at `corner` out to vertex `to`, as mesh edge `mesh_edge`.
fn edge_at_corner(mesh_edge: i32, to: i32, convexity: EdgeConvexity) -> FeatureEdge {
    FeatureEdge {
        mesh_edge_index: mesh_edge,
        start_vertex: 0,
        end_vertex: to,
        start: Vector3::new(10.0, 0.0, 10.0),
        end: Vector3::new(10.0 + to as f64, 0.0, 10.0),
        convexity,
        turn_angle_degrees: 90.0,
        ..FeatureEdge::default()
    }
}

#[test]
fn rust_only_a_four_edged_mixed_corner_warns_every_group_at_it() {
    let resolutions: Vec<_> = [
        EdgeConvexity::Convex,
        EdgeConvexity::Convex,
        EdgeConvexity::Convex,
        EdgeConvexity::Concave,
    ]
    .iter()
    .enumerate()
    .map(|(i, &convexity)| resolution(vec![edge_at_corner(i as i32 + 1, i as i32 + 1, convexity)]))
    .collect();

    let warnings = describe(&resolutions);

    assert_eq!(warnings.len(), 4);
    let expected = vec![
        capped_corner(4, Vector3::new(10.0, 0.0, 10.0)),
        BevelAdvisory::new(BevelAdvisoryCode::MixedCornerTooManyEdges, Vec::new()),
    ];
    for warning in &warnings {
        assert_eq!(warning.as_ref(), Some(&expected));
    }
}

#[test]
fn rust_only_a_corner_of_one_convexity_or_of_three_mesh_edges_is_not_warned_about() {
    let one_kind: Vec<_> = (1..=4)
        .map(|i| resolution(vec![edge_at_corner(i, i, EdgeConvexity::Convex)]))
        .collect();
    assert!(describe(&one_kind).iter().all(Option::is_none));

    // Four pickings, three mesh edges: the same edge picked into two groups is one edge.
    let picked_twice: Vec<_> = [
        (1, EdgeConvexity::Convex),
        (2, EdgeConvexity::Convex),
        (3, EdgeConvexity::Concave),
        (3, EdgeConvexity::Concave),
    ]
    .iter()
    .map(|&(i, convexity)| resolution(vec![edge_at_corner(i, i, convexity)]))
    .collect();
    assert!(describe(&picked_twice).iter().all(Option::is_none));
}

#[test]
fn rust_only_an_unresolved_group_counts_nothing_at_the_corner() {
    let mut resolutions: Vec<_> = [
        EdgeConvexity::Convex,
        EdgeConvexity::Convex,
        EdgeConvexity::Convex,
        EdgeConvexity::Concave,
    ]
    .iter()
    .enumerate()
    .map(|(i, &convexity)| resolution(vec![edge_at_corner(i as i32 + 1, i as i32 + 1, convexity)]))
    .collect();
    resolutions[3].failure_reason = Some(BevelAdvisory::new(
        BevelAdvisoryCode::MixedCornerCapped,
        Vec::new(),
    ));

    assert!(describe(&resolutions).iter().all(Option::is_none));
}

#[test]
fn rust_only_concave_runs_across_groups_keeps_only_the_concave_runs_of_eligible_groups() {
    let convex = resolution(vec![edge_at_corner(1, 1, EdgeConvexity::Convex)]);
    let concave = resolution(vec![edge_at_corner(2, 2, EdgeConvexity::Concave)]);
    let resolutions = vec![convex, concave];

    let runs = concave_runs_across_groups(&resolutions, None);
    assert_eq!(runs.len(), 1);
    assert_eq!(runs[0].edges[0].edge.mesh_edge_index, 2);

    assert!(concave_runs_across_groups(&resolutions, Some(&[true, false])).is_empty());
    assert!(
        concave_runs_across_groups(&resolutions, Some(&[true])).is_empty(),
        "a group past the end of the eligibility list is not eligible"
    );
}

#[test]
fn rust_only_a_cube_corner_void_is_read_once_and_shared() {
    let cube = Arc::new(platonic_solids::create_cube(10.0, 10.0, 10.0));
    let features = FeatureEdgeSet::create(&cube);
    let first = features.edges()[0];
    let corner = first.start_vertex;
    let segment = ChainEdge::new(0, first, false);
    let voids = BevelCornerVoids::new(cube.clone(), None);

    let cone = voids
        .try_read(Some(corner), &segment, first.start, 1.0, 8, true)
        .expect("a cube corner closes a ring of three faces and bounds a convex cone");
    assert!(!cone.vertices.is_empty());

    let again = voids
        .try_read(Some(corner), &segment, first.start, 1.0, 8, true)
        .expect("the memo answers the same");
    assert!(
        Rc::ptr_eq(&cone, &again),
        "both runs at a corner clip with the one cone"
    );

    assert!(voids
        .try_read(None, &segment, first.start, 1.0, 8, true)
        .is_none());
}
