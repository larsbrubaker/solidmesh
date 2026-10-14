//! The FOLDED FLAP repair - two faces on the same three vertices wound opposite ways, which a
//! boolean can leave behind where two solids met flush. A flap encloses no volume, but every edge
//! it lies along is used by FOUR faces, and the surface construction republishes the source's own
//! faces - so it is repaired once, on the working copy
//! (`bevel_working_mesh::create_from_with_advisories`), and reported.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{Face, Mesh};

use geometry_processing::bevel::bevel_working_mesh::{
    create_from_with_advisories, signed_volume, StampedVerdict,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode};

use crate::nose_in_notch_test_meshes;

#[test]
fn a_face_duplicated_back_to_back_is_dropped_from_the_working_copy() {
    let source = nose_in_notch_test_meshes::create();
    let face_count = source.faces.len();

    let flapped = with_the_first_face_flapped(&source);

    // Stated first: without the flap there is nothing here to repair.
    assert!(
        !flapped.is_manifold(),
        "the flap's three edges are each used by four faces, which is what non-manifold means"
    );
    assert_eq!(
        edges_used_by_too_many_faces(&flapped),
        3,
        "a flap on one triangle folds exactly its own three edges"
    );

    let (working, advisories) =
        create_from_with_advisories(&flapped, &mut StampedVerdict::default());

    assert!(
        working.is_manifold(),
        "the flap is the only thing wrong with this mesh, so dropping it closes the part"
    );
    assert_eq!(
        working.faces.len(),
        face_count,
        "both faces of the PAIR go and the copy that tiles the part stays"
    );
    assert!(
        (signed_volume(&working) - nose_in_notch_test_meshes::VOLUME).abs() <= 1e-3,
        "a flap encloses nothing, so removing it may not move the solid"
    );
    assert_eq!(
        advisories.len(),
        1,
        "a face the file carried has been dropped, and the user is told so"
    );
    assert_eq!(
        advisories[0],
        BevelAdvisory::new(BevelAdvisoryCode::SourceMeshHadFoldedFlaps, vec![1.into()]),
        "one pair went, and the sentence counts them"
    );
}

/// A face duplicated the SAME way round is left alone: a different defect with a different cure.
#[test]
fn a_face_duplicated_the_same_way_round_is_left_alone() {
    let source = nose_in_notch_test_meshes::create();

    let mut doubled = source.copy();
    let face = doubled.faces[0];
    let added = Face::new(face.v0, face.v1, face.v2, &doubled.vertices);
    doubled.faces.push(added);

    let (working, advisories) =
        create_from_with_advisories(&doubled, &mut StampedVerdict::default());

    assert_eq!(
        working.faces.len(),
        doubled.faces.len(),
        "nothing here is a fold, so this repair has no opinion about it"
    );
    assert_eq!(
        advisories.len(),
        0,
        "nothing was dropped, so there is nothing to report"
    );
}

/// A pair that is the ONLY thing covering its triangle is left alone, because cancelling it would
/// open a hole where the part had a surface.
#[test]
fn a_pair_that_is_the_only_surface_there_is_left_alone() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let flapped = with_the_first_face_flapped_once(&cube);
    let face_count = flapped.faces.len();

    let (working, advisories) =
        create_from_with_advisories(&flapped, &mut StampedVerdict::default());

    assert_eq!(
        working.faces.len(),
        face_count,
        "nothing else covers this triangle, so cancelling the pair would leave a hole"
    );
    assert!(
        advisories.is_empty(),
        "nothing was dropped, so there is nothing to report"
    );
}

/// A copy of the mesh with face 0 published once more the other way round.
fn with_the_first_face_flapped_once(mesh: &Mesh) -> Mesh {
    let mut flapped = mesh.copy();
    let face = flapped.faces[0];
    let added = Face::new(face.v2, face.v1, face.v0, &flapped.vertices);
    flapped.faces.push(added);
    flapped
}

/// A copy of the mesh with face 0 published twice more, once each way round.
fn with_the_first_face_flapped(mesh: &Mesh) -> Mesh {
    let mut flapped = mesh.copy();
    let face = flapped.faces[0];
    let same = Face::new(face.v0, face.v1, face.v2, &flapped.vertices);
    let back = Face::new(face.v2, face.v1, face.v0, &flapped.vertices);
    flapped.faces.push(same);
    flapped.faces.push(back);
    flapped
}

/// How many edges of the mesh are used by more than two faces.
fn edges_used_by_too_many_faces(mesh: &Mesh) -> i32 {
    let graph = mesh.get_mesh_edge_graph();
    (0..graph.edge_count())
        .filter(|&edge| graph.get_face_count(edge) > 2)
        .count() as i32
}
