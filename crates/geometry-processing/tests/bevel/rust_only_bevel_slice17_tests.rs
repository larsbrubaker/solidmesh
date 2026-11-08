//! Rust-only checks on slice 17's production units that no other test reaches on its own: the join
//! closing a ring it assembled from single picks, the three-plane point the mitre sector cuts with,
//! the empty chord table, and the interrupted-crease reading's quick answers.

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_chain_join, surface_interrupted_crease, surface_mitre_sector,
};
use solidmesh::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids;
use std::sync::Arc;

/// Four edges round a cube's top picked one at a time are four chains; the join puts them end to
/// end and, being the one who assembled the ring, publishes it CLOSED.
#[test]
fn rust_only_the_join_closes_a_ring_it_assembled_from_single_picks() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let top: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - 10.0).abs() < 1e-6 && (edge.end.z - 10.0).abs() < 1e-6)
        .copied()
        .collect();
    assert_eq!(top.len(), 4);

    let singles: Vec<_> = top
        .iter()
        .flat_map(|edge| bevel_chain_linker::link(std::slice::from_ref(edge)))
        .collect();
    assert_eq!(singles.len(), 4);

    let joined = surface_chain_join::join(&singles);

    assert_eq!(joined.len(), 1, "four ends meeting in pairs are one run");
    assert!(
        joined[0].is_closed,
        "and the run that comes back to its start is a loop"
    );
    assert_eq!(joined[0].edges.len(), 4);

    for i in 0..4 {
        assert_eq!(
            joined[0].edges[i].to_vertex(),
            joined[0].edges[(i + 1) % 4].from_vertex(),
            "every edge runs head to tail into the next"
        );
    }
}

/// A run that does not come back is joined but left open.
#[test]
fn rust_only_the_join_leaves_an_open_run_open() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let top: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - 10.0).abs() < 1e-6 && (edge.end.z - 10.0).abs() < 1e-6)
        .copied()
        .collect();

    // Three of the four: an open U.
    let mut singles = Vec::new();
    let mut kept = vec![top[0]];
    for edge in &top[1..] {
        if kept.len() < 3
            && kept.iter().any(|k| {
                k.start_vertex == edge.start_vertex
                    || k.start_vertex == edge.end_vertex
                    || k.end_vertex == edge.start_vertex
                    || k.end_vertex == edge.end_vertex
            })
        {
            kept.push(*edge);
        }
    }
    if kept.len() < 3 {
        kept.push(
            *top.iter()
                .find(|edge| !kept.contains(edge))
                .expect("a fourth edge"),
        );
    }
    for edge in &kept {
        singles.extend(bevel_chain_linker::link(std::slice::from_ref(edge)));
    }

    let joined = surface_chain_join::join(&singles);

    assert_eq!(joined.len(), 1);
    assert!(!joined[0].is_closed);
    assert_eq!(joined[0].edges.len(), 3);
}

/// The three planes of a cube corner meet at the corner; two parallel planes meet nowhere.
#[test]
fn rust_only_three_planes_meet_at_the_corner_or_nowhere() {
    let at = surface_mitre_sector::try_three_planes(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 3.0),
        Vector3::new(0.0, 0.0, -5.0),
    )
    .expect("three square planes meet in a point");
    assert!((at - Vector3::new(1.0, 2.0, 3.0)).length() < 1e-12);

    assert!(surface_mitre_sector::try_three_planes(
        Vector3::ZERO,
        Vector3::UNIT_X,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::UNIT_X,
        Vector3::ZERO,
        Vector3::UNIT_Z,
    )
    .is_none());

    assert!(surface_mitre_sector::try_three_planes(
        Vector3::ZERO,
        Vector3::ZERO,
        Vector3::ZERO,
        Vector3::UNIT_Y,
        Vector3::ZERO,
        Vector3::UNIT_Z,
    )
    .is_none());
}

/// One chord fewer than rails, each empty, and none at all for a ring of one.
#[test]
fn rust_only_nothing_along_is_one_empty_chord_per_gap() {
    let between = surface_mitre_sector::nothing_along(4);
    assert_eq!(between.len(), 3);
    assert!(between.iter().all(Vec::is_empty));
    assert!(surface_mitre_sector::nothing_along(1).is_empty());
    assert!(surface_mitre_sector::nothing_along(0).is_empty());
}

/// A single pick cannot resume itself, and convex edges are no crease: neither is named.
#[test]
fn rust_only_the_interrupted_crease_reading_is_silent_on_one_chain_and_on_convex_edges() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let regions = FaceRegionCache::new(
        Arc::new(cube.clone()),
        Some(cube.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );

    let one = bevel_chain_linker::link(&features.edges()[0..1]);
    assert!(surface_interrupted_crease::try_read(&one, &features, &regions, 1.0, true).is_none());

    let mut two = bevel_chain_linker::link(&features.edges()[0..1]);
    two.extend(bevel_chain_linker::link(&features.edges()[5..6]));
    assert_eq!(two.len(), 2);
    assert!(surface_interrupted_crease::try_read(&two, &features, &regions, 1.0, true).is_none());
}
