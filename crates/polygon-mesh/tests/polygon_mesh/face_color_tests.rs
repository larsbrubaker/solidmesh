//! Port of `Agg.PolygonMesh/FaceColorTests.cs`: per-face colors through copies, welds,
//! booleans and face merging. The boolean cases run the contract in
//! `face_color_boolean_scenarios.rs`.

use agg_path::Color;
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3Float};

use crate::face_color_boolean_scenarios as scenarios;

fn half_red_half_blue(count: usize) -> Vec<Color> {
    (0..count)
        .map(|i| {
            if i < count / 2 {
                Color::RED
            } else {
                Color::BLUE
            }
        })
        .collect()
}

#[test]
fn copy_preserves_face_colors() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let colors = half_red_half_blue(mesh.faces.len());
    mesh.face_colors = Some(colors.clone());

    let copy = mesh.copy();

    let copy_colors = copy.face_colors.as_ref().expect("copy has colors");
    assert_eq!(copy_colors.len(), colors.len());
    let source_colors = mesh.face_colors.as_ref().expect("source colors");
    for (copied, source) in copy_colors.iter().zip(source_colors) {
        assert_eq!(copied, source);
    }

    // Verify it's a deep copy - modifying original doesn't affect copy
    mesh.face_colors.as_mut().unwrap()[0] = Color::GREEN;
    assert_eq!(copy.face_colors.as_ref().unwrap()[0], Color::RED);
}

#[test]
fn copy_without_face_colors_returns_null() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let copy = mesh.copy();
    assert!(copy.face_colors.is_none());
}

#[test]
fn boolean_union_preserves_face_colors() {
    scenarios::boolean_union_preserves_face_colors();
}

#[test]
fn boolean_subtract_preserves_face_colors() {
    scenarios::boolean_subtract_preserves_face_colors();
}

#[test]
fn do_array_with_colors_preserves_face_colors() {
    scenarios::do_array_with_colors_preserves_face_colors();
}

#[test]
fn short_mesh_colors_leaves_the_extra_operand_unattributed() {
    scenarios::short_mesh_colors_leaves_the_extra_operand_unattributed();
}

#[test]
fn boolean_without_colors_returns_null_face_colors() {
    scenarios::boolean_without_colors_returns_null_face_colors();
}

#[test]
fn clean_and_merge_preserves_face_colors() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    mesh.face_colors = Some(half_red_half_blue(mesh.faces.len()));

    let original_face_count = mesh.faces.len();
    mesh.clean_and_merge();

    // Face count should be unchanged for a valid cube
    assert_eq!(mesh.faces.len(), original_face_count);
    let colors = mesh.face_colors.as_ref().expect("colors kept");
    assert_eq!(colors.len(), mesh.faces.len());

    let has_red = colors
        .iter()
        .any(|c| c.red == 255 && c.green == 0 && c.blue == 0);
    let has_blue = colors
        .iter()
        .any(|c| c.red == 0 && c.green == 0 && c.blue == 255);
    assert!(has_red);
    assert!(has_blue);
}

#[test]
fn clean_and_merge_with_degenerate_faces_preserves_face_colors() {
    // A mesh with a degenerate face that clean_and_merge will remove
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(1.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 1.0, 0.0));
    mesh.vertices.push(Vector3Float::new(1.0, 1.0, 0.0)); // duplicate position for degenerate
    mesh.vertices.push(Vector3Float::new(1.0, 0.0, 0.0)); // exact duplicate of vertex 1

    // Normal triangle
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    // Degenerate triangle (vertex 1 and 4 share the same position, so after merge iv1 == iv4)
    mesh.faces.add(1, 4, 3, &mesh.vertices);

    mesh.face_colors = Some(vec![Color::RED, Color::BLUE]);

    mesh.clean_and_merge();

    // The degenerate face should be removed but the surviving face's color preserved
    let colors = mesh.face_colors.as_ref().expect("colors kept");
    assert_eq!(colors.len(), mesh.faces.len());

    // First face (Red) should survive
    assert_eq!(colors[0].red, 255);
}

#[test]
fn manifold_run_data_extracts_face_colors_correctly() {
    scenarios::manifold_run_data_extracts_face_colors_correctly();
}

#[test]
fn face_colors_survive_full_cleanup_pipeline() {
    scenarios::face_colors_survive_full_cleanup_pipeline();
}

#[test]
fn intersect_preserves_both_face_colors() {
    scenarios::intersect_preserves_both_face_colors();
}

#[test]
fn remove_unused_vertices_preserves_face_colors() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    mesh.face_colors = Some(half_red_half_blue(mesh.faces.len()));

    mesh.remove_unused_vertices();

    let colors = mesh.face_colors.as_ref().expect("colors kept");
    assert_eq!(colors.len(), mesh.faces.len());
}

#[test]
fn subtract_from_mesh_with_face_colors_preserves_colors() {
    scenarios::subtract_from_mesh_with_face_colors_preserves_colors();
}

#[test]
fn copy_all_faces_of_an_uncolored_mesh_into_a_colored_one_leaves_no_transparent_faces() {
    // Color::default() is rgba(0,0,0,0) - a face left at it draws as nothing, so an uncolored
    // part merged into a painted result would silently disappear.
    let mut colored = create_cube(10.0, 10.0, 10.0);
    colored.face_colors = Some(vec![Color::RED; colored.faces.len()]);

    let uncolored = create_cube(5.0, 5.0, 5.0);
    let colored_face_count = colored.faces.len();

    colored.copy_all_faces(&uncolored, Matrix4X4::IDENTITY);

    let colors = colored.face_colors.as_ref().expect("colors kept");
    assert_eq!(colors.len(), colored.faces.len());
    assert!(!colors.iter().any(|c| c.alpha == 0));

    // The mesh that had colors keeps them, the merged-in one comes in as unpainted white
    for color in &colors[..colored_face_count] {
        assert_eq!(*color, Color::RED);
    }
    for color in &colors[colored_face_count..] {
        assert_eq!(*color, Color::WHITE);
    }
}

#[test]
fn copy_all_faces_of_a_colored_mesh_into_an_uncolored_one_keeps_the_colors() {
    // The mirror case: without an array for the destination's own faces the source's colors
    // are dropped entirely, so a painted set merged into an unpainted one loses its paint.
    let mut uncolored = create_cube(10.0, 10.0, 10.0);
    let uncolored_face_count = uncolored.faces.len();

    let mut colored = create_cube(5.0, 5.0, 5.0);
    colored.face_colors = Some(vec![Color::BLUE; colored.faces.len()]);

    uncolored.copy_all_faces(&colored, Matrix4X4::IDENTITY);

    let colors = uncolored.face_colors.as_ref().expect("colors created");
    assert_eq!(colors.len(), uncolored.faces.len());
    assert!(!colors.iter().any(|c| c.alpha == 0));

    for color in &colors[..uncolored_face_count] {
        assert_eq!(*color, Color::WHITE);
    }
    for color in &colors[uncolored_face_count..] {
        assert_eq!(*color, Color::BLUE);
    }
}

#[test]
fn copy_all_faces_between_uncolored_meshes_stays_uncolored() {
    // Nothing is painted, so the objects' own colors are doing the work - starting a face color
    // array here would override them with white.
    let mut destination = create_cube(10.0, 10.0, 10.0);
    destination.copy_all_faces(&create_cube(5.0, 5.0, 5.0), Matrix4X4::IDENTITY);

    assert!(destination.face_colors.is_none());
}
