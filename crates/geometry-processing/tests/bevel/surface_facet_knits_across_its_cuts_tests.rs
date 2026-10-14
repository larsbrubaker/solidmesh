//! One facet cut by more than one of its region's
//! cuts, which is where the pieces have to be knitted ACROSS the cuts and not just within each one
//! (`surface_face_walkback`). Read on a cylinder's WALL facet, which is flat: an L chain of the top
//! rim edge and the vertical crease below its start gives that one facet two cuts, and with a round
//! profile the second cut's arc chords cross a segment the first cut left behind.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{surface_face_walkback, SurfaceBevelPlan};
use geometry_processing::topology::{FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::surface_walkback_checks::{
    assert_no_new_t_junctions, boundary_loops, regions_of, rounded,
};
use crate::topology_test_meshes;

#[test]
fn a_facet_cut_twice_carries_the_second_cuts_crossing_into_both_of_the_first_cuts_pieces() {
    let cylinder = Arc::new(topology_test_meshes::create_cylinder(10.0, 20.0, 12));
    let (regions, features) = regions_of(&cylinder);
    let regions = Arc::new(regions);

    let rim = edge_between(
        &features,
        Vector3::new(10.0, 0.0, 10.0),
        Vector3::new(8.660254, 5.0, 10.0),
    );
    let crease = edge_between(
        &features,
        Vector3::new(10.0, 0.0, 10.0),
        Vector3::new(10.0, 0.0, -10.0),
    );

    let chains = bevel_chain_linker::link(&[rim, crease]);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, 1.0, true)
        .unwrap_or_else(|reason| panic!("{reason}"));

    let walked = surface_face_walkback::try_apply_one(
        &cylinder,
        &cylinder.get_mesh_edge_graph(),
        &regions,
        &plan,
    )
    .unwrap_or_else(|reason| panic!("{reason}"));

    // The generic reading: no vertex may stand part way along an edge the mesh did not already
    // carry one on, and no edge may be used by more than two faces.
    assert_no_new_t_junctions(&cylinder, &walked.mesh);

    // And the region is still closed against its neighbours - the only edges used once are the rim
    // of the gap the strip will fill, and they make whole loops.
    for gap in boundary_loops(&walked.mesh) {
        assert!(
            gap.len() >= 3,
            "a gap boundary is a loop, not a stray segment: {}",
            describe(&gap, &walked.mesh)
        );
    }
}

/// The one feature edge running between these two points, either way round.
fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-3 && (edge.end - to).length() < 1e-3)
                || ((edge.start - to).length() < 1e-3 && (edge.end - from).length() < 1e-3)
        })
        .collect();

    assert_eq!(found.len(), 1, "one feature edge between the two points");

    found[0]
}

fn describe(gap: &[i32], mesh: &Mesh) -> String {
    gap.iter()
        .map(|&vertex| rounded(mesh.vertices[vertex as usize].as_vector3()))
        .collect::<Vec<_>>()
        .join(", ")
}
