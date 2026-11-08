//! Taking a color every face wears off the mesh (`take_uniform_face_color`): one color painted on
//! every face moves off the mesh; several colors, unpainted faces or no face colors leave the mesh
//! as it was.

use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::take_uniform_face_color;
use vector_math::Color;

#[test]
fn one_color_on_every_face_moves_off_the_mesh() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    mesh.face_colors = Some(vec![Color::RED; mesh.faces.len()]);

    assert_eq!(take_uniform_face_color(&mut mesh), Some(Color::RED));
    assert!(mesh.face_colors.is_none());
}

#[test]
fn two_colors_stay_on_the_faces() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let mut colors = vec![Color::RED; mesh.faces.len()];
    colors[5] = Color::BLUE;
    mesh.face_colors = Some(colors.clone());

    assert_eq!(take_uniform_face_color(&mut mesh), None);
    assert_eq!(mesh.face_colors, Some(colors));
}

#[test]
fn unpainted_faces_stay_unpainted() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let clear = Color::default();
    mesh.face_colors = Some(vec![clear; mesh.faces.len()]);

    assert_eq!(take_uniform_face_color(&mut mesh), None);
    assert_eq!(mesh.face_colors, Some(vec![clear; mesh.faces.len()]));
}

#[test]
fn a_mesh_without_face_colors_has_none_to_take() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    assert_eq!(take_uniform_face_color(&mut mesh), None);

    mesh.face_colors = Some(Vec::new());
    assert_eq!(take_uniform_face_color(&mut mesh), None);
    assert_eq!(mesh.face_colors, Some(Vec::new()));
}
