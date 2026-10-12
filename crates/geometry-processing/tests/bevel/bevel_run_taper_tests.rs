//! Port of `R/Tests/GeometryProcessingTests/BevelRunTaperTests.cs`: the run-out a selected open
//! run walks onto the unselected edges past its free ends ([`BevelRunTaper`]). The C# tests that
//! sweep the taper (`BevelTaperedSweep`) or build through `BevelGroupTools` land with those types.

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::{BevelRunTaper, BevelSurfaceLabels};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::solids::platonic_solids;

use crate::topology_test_meshes::create_cylinder;

#[test]
fn selected_neighbor_or_branch_blocks_only_its_end() {
    for branch in [false, true] {
        let mesh = create_cylinder(10.0, 10.0, 24);
        let features = FeatureEdgeSet::create(&mesh);
        let edge = *features
            .edges()
            .iter()
            .find(|e| e.start.z == 5.0 && e.end.z == 5.0)
            .expect("a top rim edge");
        let chain = bevel_chain_linker::link(&[edge]).remove(0);
        let to_vertex = chain.edges[0].to_vertex();
        let neighbour = *features
            .edges()
            .iter()
            .find(|e| {
                e.mesh_edge_index != edge.mesh_edge_index
                    && (e.start_vertex == to_vertex || e.end_vertex == to_vertex)
                    && ((e.start.z != e.end.z) == branch)
            })
            .expect("a neighbour at the run's end");
        let mut labels = BevelSurfaceLabels::new(&mesh, None);
        let taper = BevelRunTaper::create(
            &chain,
            &features,
            Some(labels.at(30.0)),
            &[edge, neighbour],
            30.0,
            0.5,
            0.5,
        );
        let taper = taper.expect("the free start still runs out");
        assert_eq!(taper.end_length, 0.0, "branch {branch}");
        assert_eq!(taper.start_length, 0.5, "branch {branch}");
    }
}

#[test]
fn hard_corners_keep_the_conventional_sweep() {
    let mesh = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let edge = features.edges()[0];
    let mut labels = BevelSurfaceLabels::new(&mesh, None);
    let chain = bevel_chain_linker::link(&[edge]).remove(0);
    assert!(BevelRunTaper::create(
        &chain,
        &features,
        Some(labels.at(30.0)),
        &[edge],
        30.0,
        0.5,
        0.5
    )
    .is_none());
}
