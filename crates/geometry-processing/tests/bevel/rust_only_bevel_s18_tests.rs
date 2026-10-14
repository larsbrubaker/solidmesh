//! Rust-only checks on slice 18's pass bookkeeping that no other test reaches on its own: groups
//! sharing edges linked as one network (`surface_shared_edge_network`) and the seam re-read's
//! candidates (`surface_seam_re_read`).

use geometry_processing::bevel::bevel_chain_linker::link_resolution;
use geometry_processing::bevel::surface::{surface_seam_re_read, surface_shared_edge_network};

use crate::tapered_barrel_test_meshes;

/// One group alone is linked exactly as it always was, and two groups naming the same edges are
/// linked once, as one selection, rather than each keeping a copy of every run.
#[test]
fn rust_only_groups_sharing_their_edges_link_as_one_network() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let wall = tapered_barrel_test_meshes::the_wall(&part);

    let alone = surface_shared_edge_network::link_together(std::slice::from_ref(&wall));
    let as_before = link_resolution(&wall);

    assert_eq!(alone.len(), as_before.len());
    assert_eq!(alone.len(), 6);

    let twice = surface_shared_edge_network::link_together(&[wall.clone(), wall.clone()]);

    assert_eq!(twice.len(), 6, "the doubly named edges are one network");
    assert_eq!(
        twice.iter().map(|chain| chain.edges.len()).sum::<usize>(),
        wall.edges.len()
    );
}

/// The seam re-read looks only at vertices made after the walkback that a face still carries, each
/// once, in the order the faces first use them.
#[test]
fn rust_only_later_splits_are_the_new_vertices_a_face_carries() {
    let faces = [(0, 5, 7), (7, 5, 9), (1, 2, 3)];

    assert_eq!(
        surface_seam_re_read::later_splits_carried_by_a_face(&faces, 5),
        vec![5, 7, 9]
    );
    assert!(surface_seam_re_read::later_splits_carried_by_a_face(&faces, 10).is_empty());
}
