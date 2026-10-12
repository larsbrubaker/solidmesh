//! Port of `R/Tests/GeometryProcessingTests/BevelPublishedMeshTests.cs`: the publish-time cleanup,
//! what a bevel hands the viewport after the last boolean has run (`bevel_published_mesh::clean`).
//!
//! The case it exists for is the one the "5mm Bevel Around Center" fixture shows - four beads of
//! one radius unioned onto one feature, meeting almost tangentially, and the kernel's output
//! shredded where they meet. Here that is reproduced in the small: a closed box with one of its
//! faces split by a vertex duplicated a fraction of a micron away, and a fin of two back-to-back
//! triangles standing off it. C#'s "the same instance comes back" is `clean` answering `None`.

use geometry_processing::bevel::{bevel_published_mesh, bevel_working_mesh};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector3, Vector3Float};

/// A closed 2 mm box, twelve triangles, wound outwards.
fn clean_box() -> Mesh {
    platonic_solids::create_cube(2.0, 2.0, 2.0)
}

fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;

    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let v1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let v2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        total += v0.dot(v1.cross(v2));
    }

    total / 6.0
}

#[test]
fn a_clean_box_is_handed_straight_back() {
    let r#box = clean_box();

    let cleaned = bevel_published_mesh::clean(&r#box, &CancelToken::new()).expect("not cancelled");

    // a mesh with nothing to weld and nothing to drop must come out of the cleanup as the very
    // object that went in - no copy, so face colors and every other per-face datum are the
    // boolean's own
    assert!(cleaned.is_none());
}

/// The shredded case: a duplicate of one box vertex, sitting a tenth of the seam tolerance away,
/// with a fin of two back-to-back near-zero-area triangles hung between the two of them. That is
/// exactly what a near-tangent union leaves behind, and it is what the user reads as a hard seam.
#[test]
fn a_sliver_fin_and_its_duplicate_vertex_are_welded_away() {
    let mut shredded = clean_box();

    // A tenth of the tolerance the cleanup welds at, so this vertex is the same point as the
    // corner it was copied from by anybody's reading.
    let tolerance = bevel_working_mesh::seam_tolerance_for(&shredded);
    let nudge = (tolerance / 10.0) as f32;

    let corner = shredded.vertices[0];
    let duplicate = shredded.vertices.len() as i32;

    shredded.vertices.push(Vector3Float::new_f32(
        corner.x + nudge,
        corner.y + nudge,
        corner.z + nudge,
    ));

    // A neighbour of the corner to hang the fin from, and the fin itself: the same triangle
    // twice, wound both ways, which encloses nothing and has all but no area.
    let neighbour = 1;

    shredded
        .faces
        .add(0, neighbour, duplicate, &shredded.vertices);
    shredded
        .faces
        .add(0, duplicate, neighbour, &shredded.vertices);

    let box_volume = signed_volume(&clean_box());

    let cleaned = bevel_published_mesh::clean(&shredded, &CancelToken::new())
        .expect("not cancelled")
        .expect(
            "there was a duplicate vertex and a fin to take out, so the cleanup had work to do",
        );

    assert_eq!(
        cleaned.vertices.len(),
        clean_box().vertices.len(),
        "the duplicate vertex is the same point as the corner it sits a tenth of a tolerance from, \
         so the cleaned mesh has the box's own vertices and no more"
    );

    assert_eq!(
        cleaned.faces.len(),
        clean_box().faces.len(),
        "the fin's two triangles collapse when their duplicate end is welded onto the corner, \
         leaving the box's own twelve"
    );

    assert!(
        cleaned.is_manifold(),
        "the cleanup only ever publishes a closed solid"
    );

    let moved = (signed_volume(&cleaned) - box_volume).abs() / box_volume.abs();
    assert!(
        moved < 1e-6,
        "welding a seam and dropping slivers moves no material: {moved}"
    );
}

/// The fallback. A weld that would fuse two vertices a real feature holds apart changes the shape,
/// and a change of shape is not a tidy-up - the mesh the boolean computed is published instead.
#[test]
fn a_mesh_the_weld_would_reshape_is_handed_straight_back() {
    // An open surface: two triangles sharing an edge, which welds to nothing and is not manifold,
    // so the cleanup has no closed solid to publish and must fall back.
    let mut open = Mesh::new();

    open.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    open.vertices.push(Vector3Float::new(10.0, 0.0, 0.0));
    open.vertices.push(Vector3Float::new(0.0, 10.0, 0.0));
    open.vertices.push(Vector3Float::new(10.0, 10.0, 0.0));

    // A fourth vertex duplicated onto the first, so the weld does have something to do and the
    // fallback is reached on the manifold test rather than on the nothing-changed test.
    open.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));

    open.faces.add(0, 1, 2, &open.vertices);
    open.faces.add(4, 2, 3, &open.vertices);

    let cleaned = bevel_published_mesh::clean(&open, &CancelToken::new()).expect("not cancelled");

    // the cleaned copy is not a closed solid, so it is no improvement on what the boolean
    // produced and the uncleaned mesh is published
    assert!(cleaned.is_none());
}
