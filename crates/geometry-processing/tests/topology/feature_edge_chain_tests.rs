//! Ports `MatterCADTests/Standard/FeatureEdgeChainTests.cs`: tangent-chain propagation, extending
//! one picked feature edge into the run of edges a CAD package would treat as one selection.

use geometry_processing::topology::{
    ChainStopReason, FeatureEdge, FeatureEdgeChain, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector2;

use crate::topology_test_meshes;

const CYLINDER_SIDES: i32 = 64;
const CYLINDER_RADIUS: f64 = 10.0;
const CYLINDER_HEIGHT: f64 = 10.0;
const ARC_RADIUS: f64 = 10.0;
const ARC_HEIGHT: f64 = 10.0;

/// A tolerance that reads the cylinder wall as one curved surface, so the wall facet edges are not
/// feature edges and the rim vertices are plain two-edge joints rather than corners.
const SMOOTH_WALL_TOLERANCE: f64 = 10.0;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

fn cylinder() -> Mesh {
    topology_test_meshes::create_cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, CYLINDER_SIDES)
}

#[test]
fn cylinder_rim_walks_the_whole_circle_as_a_closed_loop() {
    let cylinder = cylinder();
    let features = FeatureEdgeSet::create_with_angle(&cylinder, SMOOTH_WALL_TOLERANCE);

    let chain = FeatureEdgeChain::propagate(&features, 0).unwrap();

    assert_eq!(chain.count(), CYLINDER_SIDES as usize);
    assert!(chain.is_closed());
    assert_eq!(chain.start().reason, ChainStopReason::Loop);
    assert_eq!(chain.end().reason, ChainStopReason::Loop);

    // one rim, not a mix of both
    let rim_height = chain.edges()[0].from().z;
    for edge in chain.edges() {
        assert_within(edge.from().z, rim_height, 0.001);
        assert_within(edge.to().z, rim_height, 0.001);
    }

    assert_head_to_tail(&chain);
}

/// Every vertex of a cube has three feature edges, so a picked edge is the whole selection and
/// both of its ends are corner-treatment sites for the sweep.
#[test]
fn cube_edge_chain_is_the_seed_alone_between_two_corners() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);

    for seed in 0..features.count() {
        let chain = FeatureEdgeChain::propagate(&features, seed as i32).unwrap();

        assert_eq!(chain.count(), 1);
        assert!(!chain.is_closed());
        assert_eq!(chain.start().reason, ChainStopReason::Corner);
        assert_eq!(chain.end().reason, ChainStopReason::Corner);
        assert_eq!(chain.start().vertex, features.edges()[seed].start_vertex);
        assert_eq!(chain.end().vertex, features.edges()[seed].end_vertex);
    }
}

/// The rim of a half disc is the open chain case: the walk runs the whole tessellated arc and
/// stops at the two corners where the arc meets the straight chord, whichever edge was picked.
#[test]
fn arc_prism_rim_is_an_open_chain_ending_at_corners() {
    const SEGMENTS: i32 = 32;
    let arc_prism = topology_test_meshes::create_arc_prism(ARC_RADIUS, ARC_HEIGHT, SEGMENTS);
    let features = FeatureEdgeSet::create_with_angle(&arc_prism, SMOOTH_WALL_TOLERANCE);

    let top_arc = top_arc_edge_indices(&features);
    assert_eq!(top_arc.len(), SEGMENTS as usize);

    // seeded mid-arc, so the walk has to grow in both directions
    let chain = FeatureEdgeChain::propagate(&features, top_arc[top_arc.len() / 2]).unwrap();

    assert_eq!(chain.count(), SEGMENTS as usize);
    assert!(!chain.is_closed());
    assert_eq!(chain.start().reason, ChainStopReason::Corner);
    assert_eq!(chain.end().reason, ChainStopReason::Corner);
    assert_ne!(chain.start().vertex, chain.end().vertex);

    assert_head_to_tail(&chain);

    // the same arc however it was picked, orientation included
    let from_one_end = FeatureEdgeChain::propagate(&features, top_arc[0]).unwrap();
    assert_eq!(from_one_end.count(), SEGMENTS as usize);
    assert_eq!(from_one_end.start().reason, ChainStopReason::Corner);
    assert_eq!(from_one_end.end().reason, ChainStopReason::Corner);
    assert_head_to_tail(&from_one_end);
}

/// Pins the direction tolerance both ways: a 64 sided rim steps 5.625 degrees, which is inside
/// the 30 degree default and outside a 2 degree one.
#[test]
fn direction_tolerance_decides_whether_the_rim_is_one_run() {
    let cylinder = cylinder();
    let features = FeatureEdgeSet::create_with_angle(&cylinder, SMOOTH_WALL_TOLERANCE);

    let at_default = FeatureEdgeChain::propagate(&features, 0).unwrap();
    assert_eq!(at_default.count(), CYLINDER_SIDES as usize);

    let too_tight = FeatureEdgeChain::propagate_with(
        &features,
        0,
        2.0,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
    )
    .unwrap();
    assert_eq!(too_tight.count(), 1);
    assert!(!too_tight.is_closed());
    assert_eq!(too_tight.start().reason, ChainStopReason::NoCandidate);
    assert_eq!(too_tight.end().reason, ChainStopReason::NoCandidate);

    // the face test is separate from the direction test, and stops the walk on its own
    let faces_too_tight = FeatureEdgeChain::propagate_with(
        &features,
        0,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        2.0,
    )
    .unwrap();
    assert_eq!(faces_too_tight.count(), 1);
    assert_eq!(faces_too_tight.end().reason, ChainStopReason::NoCandidate);
}

/// At the default half-degree tolerance the cylinder's wall facet edges are feature edges too, so
/// every rim vertex has three of them and the rim stops being a chain at all.
#[test]
fn wall_facets_at_the_tight_default_make_every_rim_vertex_a_corner() {
    let cylinder = cylinder();
    let features = FeatureEdgeSet::create(&cylinder);

    let rim_edge_index = features
        .edges()
        .iter()
        .position(|edge| edge.direction.z.abs() < 0.001)
        .expect("the cylinder has a rim edge") as i32;

    let chain = FeatureEdgeChain::propagate(&features, rim_edge_index).unwrap();

    assert_eq!(chain.count(), 1);
    assert_eq!(chain.start().reason, ChainStopReason::Corner);
    assert_eq!(chain.end().reason, ChainStopReason::Corner);
}

/// Consecutive edges have to meet head to tail: the sweep carries its cross-section frame along
/// the chain, and an edge stored the other way round would flip the frame at that joint.
fn assert_head_to_tail(chain: &FeatureEdgeChain) {
    let edges = chain.edges();
    for i in 0..edges.len().saturating_sub(1) {
        assert_eq!(edges[i].to_vertex(), edges[i + 1].from_vertex());
    }

    if chain.is_closed() {
        assert_eq!(edges[edges.len() - 1].to_vertex(), edges[0].from_vertex());
    }
}

/// The feature edges making up the curved part of the top rim of an arc prism, in set order. The
/// chord closing the arc has its ends on the same circle, so it is told apart by its midpoint.
fn top_arc_edge_indices(features: &FeatureEdgeSet) -> Vec<i32> {
    features
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, edge)| is_on_top_arc(edge))
        .map(|(index, _)| index as i32)
        .collect()
}

fn is_on_top_arc(edge: &FeatureEdge) -> bool {
    if (edge.start.z - ARC_HEIGHT).abs() > 0.001 || (edge.end.z - ARC_HEIGHT).abs() > 0.001 {
        return false;
    }

    let mid_point = (edge.start + edge.end) / 2.0;

    Vector2::new(mid_point.x, mid_point.y).length() > ARC_RADIUS * 0.9
}
