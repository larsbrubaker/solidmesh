//! What a stored `BevelSelectionKind::Edge` group comes to on a mesh whose crease is cut into
//! several mesh edges: all of it. The walk itself is `FeatureEdgeChain::propagate_collinear`,
//! pinned by `feature_edge_collinear_run_tests.rs`; this is the resolver end of it.

use std::collections::HashMap;

use solidmesh::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids;

use crate::split_crease_test_meshes;

const POSITION_TOLERANCE: f64 = 0.001;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

/// A group anchored on HALF a crease resolves to all of it. The other half is behind a vertex that
/// is a corner of the mesh - another solid's corner landed on it - but not a corner of the shape.
#[test]
fn an_edge_group_anchored_on_half_a_crease_resolves_to_all_of_it() {
    let mesh = split_crease_test_meshes::create_flush_brick_stack(&[0.0]);
    let features = FeatureEdgeSet::create(&mesh);
    let seed = index_of_edge_with_midpoint(&features, Vector3::new(5.0, 0.0, 10.0));

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        seed,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap();

    let resolved = bevel_edge_group_resolver::resolve(&group, &mesh);

    assert!(resolved.is_resolved());
    assert_eq!(resolved.edges.len(), 2);

    // the cube's own half of the crease and the brick's: one straight 20 mm run along y = 0, z = 10
    for edge in &resolved.edges {
        assert_within(edge.start.y, 0.0, POSITION_TOLERANCE);
        assert_within(edge.start.z, 10.0, POSITION_TOLERANCE);
        assert_within(edge.end.y, 0.0, POSITION_TOLERANCE);
        assert_within(edge.end.z, 10.0, POSITION_TOLERANCE);
    }

    let total_length: f64 = resolved
        .edges
        .iter()
        .map(|edge| (edge.end - edge.start).length())
        .sum();
    assert_within(total_length, 20.0, POSITION_TOLERANCE);
}

/// A cube edge still resolves to itself - the reading a plain pick has always had.
#[test]
fn a_cube_edge_group_still_resolves_to_the_one_edge() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let seed = index_of_edge_with_midpoint(&features, Vector3::new(0.0, -10.0, 10.0));

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        seed,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap();

    let resolved = bevel_edge_group_resolver::resolve(&group, &cube);

    assert!(resolved.is_resolved());
    assert_eq!(resolved.edges.len(), 1);
}

/// "Bevel everything" makes one group per CREASE, not one per mesh edge. Every feature edge is
/// still covered, and covered once.
#[test]
fn bevel_everything_makes_one_group_per_crease_rather_than_per_edge() {
    let mesh = split_crease_test_meshes::create_flush_brick_stack(&[0.0]);
    let features =
        FeatureEdgeSet::create_with_angle(&mesh, FeatureEdge::default_flat_angle_degrees());

    let groups = bevel_edge_group_resolver::create_groups_for_all_edges(
        &mesh,
        FeatureEdge::default_flat_angle_degrees(),
        None,
    )
    .unwrap();

    assert_eq!(
        groups.len(),
        features.count() - 1,
        "exactly one crease on this part is two mesh edges, so exactly one group is saved"
    );

    let mut times_covered: HashMap<i32, i32> = HashMap::new();

    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    for resolution in &resolutions {
        assert!(resolution.is_resolved());

        for edge in &resolution.edges {
            *times_covered.entry(edge.mesh_edge_index).or_insert(0) += 1;
        }
    }

    assert_eq!(
        times_covered.len(),
        features.count(),
        "dropping a duplicate group must not drop an edge with it"
    );

    assert_eq!(
        times_covered.values().copied().max(),
        Some(1),
        "two groups selecting one edge is the duplicate sweep this rule exists to stop"
    );
}

/// The feature edge with a given midpoint - how a test names the edge it means without depending
/// on the order detection happens to find them in.
fn index_of_edge_with_midpoint(features: &FeatureEdgeSet, midpoint: Vector3) -> i32 {
    features
        .edges()
        .iter()
        .position(|edge| (((edge.start + edge.end) / 2.0) - midpoint).length() < POSITION_TOLERANCE)
        .unwrap_or_else(|| panic!("no feature edge has its midpoint at {midpoint}")) as i32
}
