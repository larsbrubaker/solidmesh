//! Face textures are keyed by face index, so every operation that rebuilds the face list has to
//! move the entries with the faces - a texture entry must always describe the face it is filed
//! under.

use std::sync::{Arc, OnceLock};

use agg_path::image::ImageBuffer;
use agg_path::Color;
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{FaceTextureData, Mesh};
use vector_math::{Matrix4X4, Plane, Vector2Float, Vector3, Vector3Float};

/// One shared 4 x 4 image, so every texture entry in a test points at the same buffer.
fn test_texture() -> Arc<ImageBuffer> {
    static TEXTURE: OnceLock<Arc<ImageBuffer>> = OnceLock::new();
    Arc::clone(TEXTURE.get_or_init(|| Arc::new(ImageBuffer::new(4, 4))))
}

fn uv(x: f64, y: f64) -> Vector2Float {
    Vector2Float::new(x, y)
}

fn texture(uv0: Vector2Float, uv1: Vector2Float, uv2: Vector2Float) -> FaceTextureData {
    FaceTextureData::new(Some(test_texture()), uv0, uv1, uv2)
}

fn unit_texture() -> FaceTextureData {
    texture(uv(0.0, 0.0), uv(1.0, 0.0), uv(0.0, 1.0))
}

/// A triangle in the XZ plane whose UVs are the (x, z) position scaled by 1/size. Any operation
/// that preserves texturing correctly leaves every face corner's UV equal to that same function
/// of its position, no matter how the triangle was cut up.
fn make_uv_is_position_triangle(size: f64) -> Mesh {
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(size, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, size));
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    mesh.face_textures.insert(0, unit_texture());
    mesh
}

fn assert_uvs_match_positions(mesh: &Mesh, size: f32) {
    for i in 0..mesh.faces.len() {
        let data = mesh
            .face_textures
            .get(&(i as i32))
            .unwrap_or_else(|| panic!("face {i} has a texture"));
        let face = mesh.faces[i];
        let uvs = [data.uv0, data.uv1, data.uv2];
        let verts = [
            mesh.vertices[face.v0 as usize],
            mesh.vertices[face.v1 as usize],
            mesh.vertices[face.v2 as usize],
        ];
        for corner in 0..3 {
            let expected_x = f64::from(verts[corner].x / size);
            let expected_y = f64::from(verts[corner].z / size);
            assert!((f64::from(uvs[corner].x) - expected_x).abs() <= 1e-4);
            assert!((f64::from(uvs[corner].y) - expected_y).abs() <= 1e-4);
        }
    }
}

#[test]
fn copy_preserves_face_textures() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    mesh.face_textures.insert(2, unit_texture());

    let mut copy = mesh.copy();

    assert_eq!(copy.face_textures.len(), 1);
    assert!(copy.face_textures.contains_key(&2));
    assert!(copy.face_textures[&2].same_image(Some(&test_texture())));

    // The map itself has to be a copy, or texturing one mesh textures the other
    copy.face_textures.remove(&2);
    assert!(mesh.face_textures.contains_key(&2));
}

#[test]
fn clean_and_merge_moves_face_textures_with_their_faces() {
    // Face 0 is degenerate once the duplicate vertices weld, so face 1 becomes face 0 and its
    // texture has to come with it.
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(1.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(1.0, 0.0, 0.0)); // exact duplicate of vertex 0
    mesh.vertices.push(Vector3Float::new(1.0, 1.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(5.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 5.0, 0.0));
    mesh.faces.add(0, 1, 2, &mesh.vertices); // degenerate after the weld
    mesh.faces.add(3, 4, 5, &mesh.vertices);
    mesh.face_textures
        .insert(0, texture(uv(9.0, 9.0), uv(9.0, 9.0), uv(9.0, 9.0)));
    mesh.face_textures.insert(1, unit_texture());

    mesh.clean_and_merge();

    assert_eq!(mesh.faces.len(), 1);
    assert_eq!(mesh.face_textures.len(), 1);
    assert!(mesh.face_textures.contains_key(&0));
    assert_eq!(mesh.face_textures[&0].uv1.x, 1.0f32);
}

#[test]
fn merge_vertices_moves_face_colors_and_textures_with_their_faces() {
    // The first face collapses when its two near vertices weld, so the second face moves to
    // index 0 - both its color and its texture have to move with it.
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new_f32(0.0001, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 1.0, 0.0));
    mesh.vertices.push(Vector3Float::new(10.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(20.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(10.0, 10.0, 0.0));
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    mesh.faces.add(3, 4, 5, &mesh.vertices);
    mesh.face_colors = Some(vec![Color::BLUE, Color::RED]);
    mesh.face_textures.insert(1, unit_texture());

    mesh.merge_vertices(0.01, 0.0, None);

    assert_eq!(mesh.faces.len(), 1);
    let colors = mesh.face_colors.as_ref().expect("colors kept");
    assert_eq!(colors.len(), 1);
    assert_eq!(colors[0], Color::RED);
    assert_eq!(mesh.face_textures.len(), 1);
    assert!(mesh.face_textures.contains_key(&0));
}

#[test]
fn reverse_faces_swaps_texture_uvs_so_they_stay_with_their_vertices() {
    let mut mesh = make_uv_is_position_triangle(10.0);
    mesh.reverse_faces();
    assert_uvs_match_positions(&mesh, 10.0);
}

#[test]
fn split_on_planes_interpolates_textures_onto_the_cut_pieces() {
    let mut mesh = make_uv_is_position_triangle(10.0);
    mesh.split_on_planes(Vector3::new(0.0, 0.0, 1.0), &[3.0, 6.0], 0.01);
    assert!(mesh.faces.len() > 1);
    assert_uvs_match_positions(&mesh, 10.0);
}

#[test]
fn split_interpolates_textures_onto_the_cut_pieces() {
    let mut mesh = make_uv_is_position_triangle(10.0);
    // The usual split: on-plane distance .001, no clip face, clean and merge, keep the negative
    // side
    mesh.split(
        Plane::new(Vector3::new(0.0, 0.0, 1.0), 4.0),
        0.001,
        None,
        true,
        false,
    );
    assert!(mesh.faces.len() > 1);
    assert_uvs_match_positions(&mesh, 10.0);
}

#[test]
fn copy_all_faces_carries_textures_at_their_new_indices() {
    let mut destination = create_cube(10.0, 10.0, 10.0);
    let destination_face_count = destination.faces.len() as i32;
    let mut source = create_cube(5.0, 5.0, 5.0);
    source.face_textures.insert(1, unit_texture());

    destination.copy_all_faces(&source, Matrix4X4::IDENTITY);

    assert_eq!(destination.face_textures.len(), 1);
    assert!(destination
        .face_textures
        .contains_key(&(destination_face_count + 1)));
}

#[test]
fn copy_all_faces_keeps_textures_on_their_triangles_across_a_dropped_face() {
    // create_face drops a source face whose corners are not all distinct, so the textured face
    // arrives one index earlier than a flat f_start + source_index offset would predict.
    let mut source = Mesh::new();
    source.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    source.vertices.push(Vector3Float::new(1.0, 0.0, 0.0));
    source.vertices.push(Vector3Float::new(1.0, 0.0, 0.0)); // same position as vertex 1
    source.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    source.vertices.push(Vector3Float::new(5.0, 0.0, 0.0));
    source.vertices.push(Vector3Float::new(0.0, 0.0, 5.0));
    source
        .faces
        .add_with_normal(0, 1, 2, Vector3Float::new(0.0, 1.0, 0.0));
    source.faces.add(3, 4, 5, &source.vertices);
    source.face_textures.insert(1, unit_texture());

    let mut destination = create_cube(10.0, 10.0, 10.0);
    let destination_face_count = destination.faces.len();

    destination.copy_all_faces(&source, Matrix4X4::IDENTITY);

    assert_eq!(
        destination.faces.len(),
        destination_face_count + 1,
        "the degenerate source face emits no face"
    );
    assert_eq!(destination.face_textures.len(), 1);
    assert!(
        destination
            .face_textures
            .contains_key(&(destination_face_count as i32)),
        "the texture belongs to the one triangle that was actually emitted"
    );

    // And it really is the triangle the UVs were authored for
    let copied_face = destination.faces[destination_face_count];
    assert_eq!(destination.vertices[copied_face.v1 as usize].x, 5.0f32);
    assert_eq!(destination.vertices[copied_face.v2 as usize].z, 5.0f32);
}

#[test]
fn remove_degenerate_faces_moves_face_textures_with_their_faces() {
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(1.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(2.0, 0.0, 0.0)); // collinear, so face 0 has no area
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(5.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 5.0, 0.0));
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    mesh.faces.add(3, 4, 5, &mesh.vertices);
    mesh.face_textures.insert(1, unit_texture());

    mesh.remove_degenerate_faces(0.001);

    assert_eq!(mesh.faces.len(), 1);
    assert_eq!(mesh.face_textures.len(), 1);
    assert!(mesh.face_textures.contains_key(&0));
}

/// `PlaceTextureOnFaces(face, image)` (what MatterCAD's image object textures its plate with)
/// covers face 0 and every face coplanar with it, stretched so their UVs span exactly the unit
/// square, and leaves the other faces untextured.
#[test]
fn rust_only_place_texture_on_coplanar_faces_spans_the_unit_square() {
    let mut cube = create_cube(30.0, 15.0, 0.2);
    let coplanar = cube.get_coplanar_faces_of_face(0);
    cube.place_texture_on_coplanar_faces(0, Some(test_texture()));

    assert_eq!(cube.face_textures.len(), coplanar.len());
    let (mut min, mut max) = ((f32::MAX, f32::MAX), (f32::MIN, f32::MIN));
    for face in &coplanar {
        let data = &cube.face_textures[&(*face as i32)];
        for corner in [data.uv0, data.uv1, data.uv2] {
            min = (min.0.min(corner.x), min.1.min(corner.y));
            max = (max.0.max(corner.x), max.1.max(corner.y));
        }
    }
    let close = |a: f32, b: f32| (a - b).abs() < 1e-5;
    assert!(close(min.0, 0.0) && close(min.1, 0.0), "min {min:?}");
    assert!(close(max.0, 1.0) && close(max.1, 1.0), "max {max:?}");
}
