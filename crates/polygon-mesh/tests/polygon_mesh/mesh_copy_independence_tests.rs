//! A copied mesh owns its faces, so no in place face edit on the copy (reverse, recalculated
//! normals, texture UVs, face colors) reaches the source.

use polygon_mesh::{FaceTextureData, Mesh};
use vector_math::Color;
use vector_math::{Vector2Float, Vector3Float};

type Snapshot = Vec<(i32, i32, i32, Vector3Float)>;

/// Two triangles with distinguishable winding, so a reversal anywhere is visible.
fn make_two_triangles() -> Mesh {
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(10.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(10.0, 10.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 10.0, 0.0));
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    mesh.faces.add(0, 2, 3, &mesh.vertices);
    mesh
}

fn snapshot(mesh: &Mesh) -> Snapshot {
    mesh.faces
        .iter()
        .map(|f| (f.v0, f.v1, f.v2, f.normal))
        .collect()
}

fn assert_faces_match(mesh: &Mesh, expected: &Snapshot) {
    assert_eq!(mesh.faces.len(), expected.len());
    for (face, &(v0, v1, v2, normal)) in mesh.faces.iter().zip(expected) {
        assert_eq!(face.v0, v0);
        assert_eq!(face.v1, v1);
        assert_eq!(face.v2, v2);
        assert_eq!(face.normal, normal);
    }
}

#[test]
fn reversing_a_copy_leaves_the_source_alone() {
    let source = make_two_triangles();
    let before = snapshot(&source);
    let mut copy = source.copy();
    copy.reverse_faces();
    assert_faces_match(&source, &before);
}

#[test]
fn reversing_a_mesh_built_from_anothers_face_list_leaves_the_source_alone() {
    let source = make_two_triangles();
    let before = snapshot(&source);
    // The same hazard one level down - this is what Copy is built on.
    let mut copy = Mesh::from_vertices_and_faces(&source.vertices, &source.faces);
    copy.reverse_faces();
    assert_faces_match(&source, &before);
}

#[test]
fn recalculating_normals_on_a_copy_leaves_the_source_alone() {
    let source = make_two_triangles();
    let before = snapshot(&source);
    let mut copy = source.copy();
    // Move the copy's vertices without going through Transform, then recompute - the new
    // normals must not land on the source's faces.
    for v in copy.vertices.iter_mut() {
        *v = Vector3Float::new_f32(v.x, v.z, v.y);
    }
    copy.calculate_normals();
    assert_faces_match(&source, &before);
}

#[test]
fn a_copy_still_equals_its_source() {
    let source = make_two_triangles();
    let copy = source.copy();
    assert!(copy == source);
    assert_faces_match(&copy, &snapshot(&source));
}

#[test]
fn reversing_a_copy_does_not_swap_the_sources_texture_uvs() {
    let mut source = make_two_triangles();
    source.face_textures.insert(
        0,
        FaceTextureData::new(
            None,
            Vector2Float::new(0.0, 0.0),
            Vector2Float::new(1.0, 0.0),
            Vector2Float::new(1.0, 1.0),
        ),
    );

    let mut copy = source.copy();
    copy.reverse_faces();

    // reverse_face replaces the entry rather than editing the FaceTextureData in place, which
    // is what lets the copy share the values safely.
    assert_eq!(source.face_textures[&0].uv0, Vector2Float::new(0.0, 0.0));
    assert_eq!(source.face_textures[&0].uv2, Vector2Float::new(1.0, 1.0));
    assert_eq!(copy.face_textures[&0].uv0, Vector2Float::new(1.0, 1.0));
    assert_eq!(copy.face_textures[&0].uv2, Vector2Float::new(0.0, 0.0));
}

#[test]
fn coloring_a_copys_faces_leaves_the_source_uncolored() {
    let mut source = make_two_triangles();
    source.face_colors = Some(vec![Color::RED, Color::RED]);

    let mut copy = source.copy();
    copy.face_colors.as_mut().expect("copied colors")[0] = Color::BLUE;

    assert_eq!(
        source.face_colors.as_ref().expect("source colors")[0],
        Color::RED
    );
}
