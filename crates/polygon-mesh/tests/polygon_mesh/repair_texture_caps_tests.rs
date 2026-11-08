//! A repaired textured mesh stays textured everywhere: the caps the automatic repair
//! (`Mesh::auto_repaired`) puts over a hole take the picture and UVs of the textured faces around
//! them, so a generated model sent on for a colour print has a picture on every face. Faces that
//! were bare before the repair stay bare, a cap between two pictures takes the one most of its
//! corners carry, and `Mesh::texture_every_face` reaches a piece no textured face touches.

use std::sync::Arc;

use crate::test_picture::TestPicture;
use solidmesh_math::{Vector2Float, Vector3Float};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{FaceTextureData, Mesh};

fn picture() -> Arc<TestPicture> {
    Arc::new(TestPicture::new(4, 4))
}

/// The UV every face gives a corner at `p`: one value per position, so a cap that took its
/// corners' UVs from the faces around it shows exactly this too.
fn uv_at(p: Vector3Float) -> Vector2Float {
    Vector2Float::new(
        f64::from((p.x + 5.0) / 10.0),
        f64::from(((p.y + 5.0) / 10.0) + ((p.z + 5.0) / 20.0)),
    )
}

/// `mesh`'s face `index` textured with `image`, each corner's UV `uv_at` its position.
fn texture_by_position(mesh: &mut Mesh, index: usize, image: &Arc<TestPicture>) {
    let face = mesh.faces[index];
    let uv = |v: i32| uv_at(mesh.vertices[v as usize]);
    let data = FaceTextureData::new(Some(image.clone()), uv(face.v0), uv(face.v1), uv(face.v2));
    mesh.face_textures.insert(index as i32, data);
}

/// The 10 mm box with its top side (z = 5) removed, a square hole the repair caps.
fn open_box() -> Mesh {
    let full = create_cube(10.0, 10.0, 10.0);
    let mut mesh = Mesh::new();
    mesh.vertices = full.vertices.clone();
    for face in full.faces.iter() {
        let corners = [face.v0, face.v1, face.v2];
        if corners.iter().all(|&v| full.vertices[v as usize].z == 5.0) {
            continue;
        }
        mesh.faces.add(face.v0, face.v1, face.v2, &mesh.vertices);
    }
    mesh
}

fn is_top(mesh: &Mesh, index: usize) -> bool {
    let face = mesh.faces[index];
    [face.v0, face.v1, face.v2]
        .iter()
        .all(|&v| mesh.vertices[v as usize].z == 5.0)
}

fn is_bottom(mesh: &Mesh, index: usize) -> bool {
    let face = mesh.faces[index];
    [face.v0, face.v1, face.v2]
        .iter()
        .all(|&v| mesh.vertices[v as usize].z == -5.0)
}

/// Asserts face `index` carries `image` with each corner's UV `uv_at` its position.
fn assert_textured_by_position(mesh: &Mesh, index: usize, image: &Arc<TestPicture>) {
    let data = mesh
        .face_textures
        .get(&(index as i32))
        .unwrap_or_else(|| panic!("face {index} has no texture"));
    assert!(
        data.same_image(Some(image)),
        "face {index}: another picture"
    );
    let face = mesh.faces[index];
    for (v, uv) in [
        (face.v0, data.uv0),
        (face.v1, data.uv1),
        (face.v2, data.uv2),
    ] {
        assert_eq!(
            uv,
            uv_at(mesh.vertices[v as usize]),
            "face {index} corner {v}"
        );
    }
}

#[test]
fn a_textured_box_missing_a_side_repairs_with_a_textured_cap() {
    let image = picture();
    let mut mesh = open_box();
    for index in 0..mesh.faces.len() {
        texture_by_position(&mut mesh, index, &image);
    }

    let repaired = mesh.auto_repaired().expect("the repair runs");
    assert!(repaired.is_closed_consistently_wound(), "the box is closed");
    let caps: Vec<usize> = (0..repaired.faces.len())
        .filter(|&i| is_top(&repaired, i))
        .collect();
    assert_eq!(caps.len(), 2, "the hole is capped by two triangles");
    assert_eq!(repaired.face_textures.len(), repaired.faces.len());
    for index in 0..repaired.faces.len() {
        assert_textured_by_position(&repaired, index, &image);
    }
}

#[test]
fn faces_bare_before_the_repair_stay_bare_while_the_cap_is_textured() {
    let image = picture();
    let mut mesh = open_box();
    for index in 0..mesh.faces.len() {
        if !is_bottom(&mesh, index) {
            texture_by_position(&mut mesh, index, &image);
        }
    }

    let repaired = mesh.auto_repaired().expect("the repair runs");
    for index in 0..repaired.faces.len() {
        if is_bottom(&repaired, index) {
            assert!(
                !repaired.face_textures.contains_key(&(index as i32)),
                "bottom face {index} was bare and stays bare"
            );
        } else {
            assert_textured_by_position(&repaired, index, &image);
        }
    }
}

#[test]
fn an_advanced_repair_textures_its_caps_too() {
    let image = picture();
    let mut mesh = open_box();
    for index in 0..mesh.faces.len() {
        texture_by_position(&mut mesh, index, &image);
    }
    let options = solidmesh_mesh::AdvancedRepairOptions {
        fill_holes: true,
        ..Default::default()
    };
    let repaired = mesh
        .advanced_repaired(&options, &solidmesh_mesh::CancelToken::new())
        .expect("the repair runs");
    assert!(repaired.is_closed_consistently_wound(), "the box is closed");
    for index in 0..repaired.faces.len() {
        assert_textured_by_position(&repaired, index, &image);
    }
}

fn v(x: f64, y: f64, z: f64) -> Vector3Float {
    Vector3Float::new(x, y, z)
}

fn mesh_of(vertices: Vec<Vector3Float>, faces: &[[i32; 3]]) -> Mesh {
    let mut mesh = Mesh::new();
    mesh.vertices = vertices;
    for f in faces {
        mesh.faces.add(f[0], f[1], f[2], &mesh.vertices);
    }
    mesh
}

fn uv(x: f64, y: f64) -> Vector2Float {
    Vector2Float::new(x, y)
}

#[test]
fn a_cap_between_two_pictures_takes_the_one_most_corners_carry() {
    let (a, b) = (picture(), picture());
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(-1.0, -1.0, 0.0),
        v(2.0, 1.0, 0.0),
        v(1.0, 2.0, 0.0),
    ];
    // Face 0 is the bare cap. Corner 0 touches only picture B (face 1, met first); corners 1 and 2
    // touch picture A (faces 2 and 3), so A covers more of the cap.
    let mut mesh = mesh_of(vertices, &[[0, 1, 2], [0, 3, 4], [1, 3, 5], [2, 4, 5]]);
    let with = |image: &Arc<TestPicture>, uvs: [Vector2Float; 3]| {
        FaceTextureData::new(Some(image.clone()), uvs[0], uvs[1], uvs[2])
    };
    mesh.face_textures
        .insert(1, with(&b, [uv(0.9, 0.9), uv(0.8, 0.8), uv(0.7, 0.7)]));
    mesh.face_textures
        .insert(2, with(&a, [uv(0.2, 0.0), uv(0.5, 0.5), uv(0.5, 0.6)]));
    mesh.face_textures
        .insert(3, with(&a, [uv(0.0, 0.4), uv(0.6, 0.6), uv(0.6, 0.5)]));

    assert_eq!(mesh.texture_faces_from_neighbors(&[0]), 1);
    let cap = &mesh.face_textures[&0];
    assert!(cap.same_image(Some(&a)), "the cap takes picture A");
    assert_eq!(cap.uv1, uv(0.2, 0.0), "corner 1 as face 2 has it");
    assert_eq!(cap.uv2, uv(0.0, 0.4), "corner 2 as face 3 has it");
    // Corner 0 has no UV on picture A: the mean of the two that do.
    assert_eq!(cap.uv0, (uv(0.2, 0.0) + uv(0.0, 0.4)) / 2.0);
}

#[test]
fn new_faces_fill_inward_from_the_faces_touching_the_texture() {
    let image = picture();
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(2.0, 1.0, 0.0),
        v(2.0, 2.0, 0.0),
    ];
    // Face 1 touches the textured face only through face 2, which comes after it.
    let mut mesh = mesh_of(vertices, &[[0, 1, 2], [3, 4, 5], [2, 1, 3]]);
    mesh.face_textures.insert(
        0,
        FaceTextureData::new(
            Some(image.clone()),
            uv(0.0, 0.0),
            uv(1.0, 0.0),
            uv(0.0, 1.0),
        ),
    );

    assert_eq!(mesh.texture_faces_from_neighbors(&[1, 2]), 2);
    let middle = &mesh.face_textures[&2];
    assert_eq!((middle.uv0, middle.uv1), (uv(0.0, 1.0), uv(1.0, 0.0)));
    assert_eq!(middle.uv2, uv(0.5, 0.5), "the new corner takes the mean");
    let far = &mesh.face_textures[&1];
    assert!(far.same_image(Some(&image)));
    assert_eq!(far.uv0, uv(0.5, 0.5), "corner 3 as face 2 gave it");
}

#[test]
fn a_separate_bare_piece_is_textured_only_when_every_face_must_be() {
    let image = picture();
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 0.1),
        v(1.0, 0.0, 0.1),
        v(0.0, 1.0, 0.1),
    ];
    let mut mesh = mesh_of(vertices, &[[0, 1, 2], [3, 4, 5]]);
    mesh.face_textures.insert(
        0,
        FaceTextureData::new(
            Some(image.clone()),
            uv(0.1, 0.2),
            uv(0.3, 0.4),
            uv(0.5, 0.6),
        ),
    );
    assert_eq!(
        mesh.texture_faces_from_neighbors(&[1]),
        0,
        "no shared corner"
    );
    assert!(!mesh.face_textures.contains_key(&1));

    mesh.texture_every_face();
    let piece = &mesh.face_textures[&1];
    assert!(piece.same_image(Some(&image)));
    assert_eq!(
        [piece.uv0, piece.uv1, piece.uv2],
        [uv(0.1, 0.2), uv(0.3, 0.4), uv(0.5, 0.6)],
        "each corner the UV of the textured corner just below it"
    );
}

#[test]
fn a_mesh_without_textures_is_left_untextured() {
    let mut mesh = open_box();
    let repaired = mesh.auto_repaired().expect("the repair runs");
    assert!(repaired.face_textures.is_empty());
    mesh.texture_every_face();
    assert!(mesh.face_textures.is_empty());
}
