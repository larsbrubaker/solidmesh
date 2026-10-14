//! The walkback's own output is read for FOLDS before anything is built on it - an edge two of its
//! faces walk the same way, or an edge three of them use at all
//! (`surface_mesh_closure::folded_edge_count`) - and the lobe wall's walkback really is fold-free
//! at every size the sweep reads it at (`surface_face_walkback::try_apply_one`).

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_face_walkback, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::topology::FaceRegionCache;
use polygon_mesh::Mesh;

use crate::lobed_prism_meshes;

/// Two triangles walking their shared edge the same way are a fold, however they are wound.
#[test]
fn two_faces_walking_one_edge_the_same_way_are_a_fold() {
    let folded = [(0, 1, 2), (0, 1, 3)];

    let (count, at) = surface_mesh_closure::folded_edge_count(&folded);

    assert_eq!(
        count, 1,
        "both faces walk 0 -> 1, so they are one side of a surface twice over"
    );
    assert_eq!(
        at,
        (0, 1),
        "and the fold is named by the edge it stands on - instead {at:?}"
    );
}

/// And three faces on one edge are a fold whichever ways they walk it.
#[test]
fn three_faces_on_one_edge_are_a_fold() {
    let fin = [(0, 1, 2), (1, 0, 3), (1, 0, 4)];

    assert_eq!(
        surface_mesh_closure::folded_edge_count(&fin).0,
        1,
        "nothing downstream takes a face away, so a third face on an edge is a fold in the answer"
    );
}

/// An ordinary pair is not, and neither is an edge with ONE face on it - which is what the whole
/// rim of the gap the walkback just opened looks like.
#[test]
fn a_pair_and_an_open_rim_are_not() {
    let sound = [(0, 1, 2), (1, 0, 3)];

    assert_eq!(
        surface_mesh_closure::folded_edge_count(&sound).0,
        0,
        "the two faces walk the shared edge opposite ways, and the other four edges have one face each"
    );
}

/// And the walkback of the lobe wall's rim really is fold-free, at every size the sweep reads it
/// at - including the ones whose BUILD refuses later, which is how the refusal is known not to be
/// the walkback's.
#[test]
fn the_lobe_walls_walkback_has_no_fold_in_it() {
    for radius in [0.02, 0.04, 0.05, 0.09, 0.1, 0.11, 0.25] {
        the_lobe_walls_walkback_has_no_fold_in_it_at(radius);
    }
}

fn the_lobe_walls_walkback_has_no_fold_in_it_at(radius: f64) {
    let prism = Arc::new(lobed_prism_meshes::create());
    let graph = prism.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        prism.clone(),
        Some(graph.clone()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));
    let region = regions
        .region(a_facet_of_the_lobes_wall(&prism))
        .expect("the lobe wall's region");
    let chains = bevel_chain_linker::link(region.boundary_edges());
    assert_eq!(chains.len(), 1, "the wall's rim is one chain");

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, radius, true)
        .unwrap_or_else(|why| panic!("the wall's rim plans at {radius}: {:?}", why.code()));

    let walked = surface_face_walkback::try_apply_one(&prism, &graph, &regions, &plan)
        .unwrap_or_else(|failure| {
            panic!(
                "the walkback has to apply at {radius} - instead {:?}",
                failure.code()
            )
        });

    let faces: Vec<(i32, i32, i32)> = walked
        .mesh
        .faces
        .iter()
        .map(|face| (face.v0, face.v1, face.v2))
        .collect();

    let (count, at) = surface_mesh_closure::folded_edge_count(&faces);

    assert_eq!(
        count,
        0,
        "instead the walked mesh folds on {at:?} at {radius}: {:?} -> {:?}",
        walked.mesh.vertices[at.0 as usize].as_vector3(),
        walked.mesh.vertices[at.1 as usize].as_vector3()
    );
}

/// Any one facet of the lobe's own wall - a side face standing well outside the flat side.
fn a_facet_of_the_lobes_wall(prism: &Mesh) -> i32 {
    for face in 0..prism.faces.len() {
        let facet = &prism.faces[face];
        let middle = (prism.vertices[facet.v0 as usize].as_vector3()
            + prism.vertices[facet.v1 as usize].as_vector3()
            + prism.vertices[facet.v2 as usize].as_vector3())
            / 3.0;

        if middle.y < -lobed_prism_meshes::RADIUS / 2.0
            && middle.z > 1.0
            && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        {
            return face as i32;
        }
    }

    panic!("the prism has lost the lobe it pokes through its side");
}
