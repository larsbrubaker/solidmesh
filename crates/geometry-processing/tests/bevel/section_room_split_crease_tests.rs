//! Ports `Tests/GeometryProcessingTests/SectionRoomSplitCreaseTests.cs` 1:1: a crease split in two
//! is read to its far end and not to the first mesh edge - splitting a bore's wall folds into two
//! collinear mesh edges moves no geometry, so it may not move the room.

use geometry_processing::bevel::sections::section_room;
use geometry_processing::topology::{FeatureEdgeChain, FeatureEdgeSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::bore_test_meshes;

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const LAND: f64 = 6.0;
const SPLIT_AT: f64 = 1.0;

#[test]
fn a_crease_split_in_two_is_read_to_its_far_end_and_not_to_the_first_mesh_edge() {
    let whole =
        bore_test_meshes::create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, LAND);
    let whole_graph = MeshEdgeGraph::create(&whole);

    let mouth = the_upper_mouth(&whole, &whole_graph);
    let (_, unsplit, _) =
        section_room::try_fit(Some(&whole), Some(&whole_graph), Some(&mouth), 0.0);
    assert!(
        (unsplit - LAND).abs() <= 1e-3,
        "the unsplit plate's room is its land - it read {unsplit}"
    );

    let part = split_every_wall_fold();
    let graph = MeshEdgeGraph::create(&part);

    let mouth = the_upper_mouth(&part, &graph);
    let (fits, largest, failure) =
        section_room::try_fit(Some(&part), Some(&graph), Some(&mouth), 0.0);
    assert!(fits, "the split mouth reads at all - instead {failure:?}");

    // Splitting a fold into two collinear mesh edges moves no geometry, so it may not move the room.
    assert!(
        (largest - unsplit).abs() <= 1e-3,
        "the whole part reads {unsplit} and the split one {largest}, where the first piece of the split fold is only {SPLIT_AT} long"
    );
}

fn split_every_wall_fold() -> Mesh {
    let mut part =
        bore_test_meshes::create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, LAND);
    for i in 0..SIDES {
        let turn = 2.0 * std::f64::consts::PI * f64::from(i) / f64::from(SIDES);
        let around = Vector3::new(BORE_RADIUS * turn.cos(), BORE_RADIUS * turn.sin(), 0.0);
        bore_test_meshes::split_the_edge_between(
            &mut part,
            around + Vector3::new(0.0, 0.0, THICKNESS / 2.0),
            around - Vector3::new(0.0, 0.0, THICKNESS / 2.0),
            SPLIT_AT / THICKNESS,
        );
    }
    part
}

fn the_upper_mouth(part: &Mesh, graph: &MeshEdgeGraph) -> FeatureEdgeChain {
    let features = FeatureEdgeSet::create_with_graph(part, graph, 20.0);
    for i in 0..features.count() {
        let edge = &features.edges()[i];
        if on_the_mouth(edge.start) && on_the_mouth(edge.end) {
            let chain = FeatureEdgeChain::propagate(&features, i as i32).expect("the mouth walks");
            assert!(
                chain.is_closed(),
                "the mouth walks closed, and it walked {} open",
                chain.edges().len()
            );
            return chain;
        }
    }
    panic!("the fixture has no upper mouth");
}

fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}
