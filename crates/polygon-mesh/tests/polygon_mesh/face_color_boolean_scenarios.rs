//! The face-colour behaviour a CSG backend has to deliver, stated once as a contract.
//! `face_color_tests.rs` runs each scenario through the public `boolean_processing` entry
//! points.

use agg_path::Color;
use polygon_mesh::csg::{boolean_processing, BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Matrix4X4;

fn translation(x: f64, y: f64, z: f64) -> Matrix4X4 {
    Matrix4X4::create_translation(x, y, z)
}

fn colors(mesh_colors: &[Color]) -> BooleanOptions {
    BooleanOptions {
        mesh_colors: Some(mesh_colors.to_vec()),
        ..BooleanOptions::default()
    }
}

fn do_boolean(
    a: &Mesh,
    matrix_a: Matrix4X4,
    b: &Mesh,
    matrix_b: Matrix4X4,
    operation: CsgModes,
    options: &BooleanOptions,
) -> Mesh {
    boolean_processing::do_boolean(
        a,
        matrix_a,
        b,
        matrix_b,
        operation,
        &CancelToken::new(),
        options,
    )
    .expect("boolean")
}

fn do_array(items: &[(&Mesh, Matrix4X4)], options: &BooleanOptions) -> Mesh {
    boolean_processing::do_array(items, CsgModes::Union, &CancelToken::new(), options)
        .expect("boolean")
}

fn face_colors(mesh: &Mesh) -> &[Color] {
    mesh.face_colors.as_deref().expect("result has face colors")
}

fn is_red(c: &Color) -> bool {
    c.red == 255 && c.green == 0 && c.blue == 0
}

fn is_green(c: &Color) -> bool {
    c.red == 0 && c.green == 255 && c.blue == 0
}

fn is_blue(c: &Color) -> bool {
    c.red == 0 && c.green == 0 && c.blue == 255
}

fn has_red(mesh: &Mesh) -> bool {
    face_colors(mesh).iter().any(is_red)
}

fn has_green(mesh: &Mesh) -> bool {
    face_colors(mesh).iter().any(is_green)
}

fn has_blue(mesh: &Mesh) -> bool {
    face_colors(mesh).iter().any(is_blue)
}

fn assert_parallel(mesh: &Mesh) {
    assert_eq!(face_colors(mesh).len(), mesh.faces.len());
}

pub fn boolean_union_preserves_face_colors() {
    let mesh_a = create_cube(10.0, 10.0, 10.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);

    let result = do_boolean(
        &mesh_a,
        translation(-3.0, 0.0, 0.0),
        &mesh_b,
        translation(3.0, 0.0, 0.0),
        CsgModes::Union,
        &colors(&[Color::RED, Color::BLUE]),
    );

    assert_parallel(&result);
    assert!(has_red(&result));
    assert!(has_blue(&result));
}

pub fn boolean_subtract_preserves_face_colors() {
    let mesh_a = create_cube(20.0, 20.0, 20.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);

    let result = do_boolean(
        &mesh_a,
        Matrix4X4::IDENTITY,
        &mesh_b,
        translation(5.0, 5.0, 5.0),
        CsgModes::Subtract,
        &colors(&[Color::RED, Color::BLUE]),
    );

    assert_parallel(&result);
    // Subtract result should have faces from both meshes
    assert!(has_red(&result));
    assert!(has_blue(&result));
}

pub fn do_array_with_colors_preserves_face_colors() {
    let mesh_a = create_cube(10.0, 10.0, 10.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);
    let mesh_c = create_cube(10.0, 10.0, 10.0);

    let result = do_array(
        &[
            (&mesh_a, translation(-8.0, 0.0, 0.0)),
            (&mesh_b, translation(0.0, 0.0, 0.0)),
            (&mesh_c, translation(8.0, 0.0, 0.0)),
        ],
        &colors(&[Color::RED, Color::GREEN, Color::BLUE]),
    );

    assert_parallel(&result);
    assert!(has_red(&result));
    assert!(has_green(&result));
    assert!(has_blue(&result));
}

pub fn short_mesh_colors_leaves_the_extra_operand_unattributed() {
    // Three disjoint cubes but only two colours. The operand the array does not reach has no
    // colour of its own, so it falls into the unattributed path and takes the first operand's
    // colour, which is at least a colour the caller actually supplied.
    let cube = create_cube(10.0, 10.0, 10.0);
    let result = do_array(
        &[
            (&cube, translation(-30.0, 0.0, 0.0)),
            (&cube, Matrix4X4::IDENTITY),
            (&cube, translation(30.0, 0.0, 0.0)),
        ],
        &colors(&[Color::RED, Color::BLUE]),
    );

    assert_parallel(&result);
    assert!(
        !face_colors(&result).contains(&Mesh::UNKNOWN_FACE_COLOR),
        "no face may be painted a colour the caller never supplied"
    );

    // A cube is 12 triangles: the first operand's own 12, plus the 12 of the operand the short
    // array left unattributed.
    assert_eq!(
        face_colors(&result).iter().filter(|c| is_red(c)).count(),
        24
    );
    assert_eq!(
        face_colors(&result).iter().filter(|c| is_blue(c)).count(),
        12
    );
}

pub fn boolean_without_colors_returns_null_face_colors() {
    let mesh_a = create_cube(10.0, 10.0, 10.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);

    let result = do_boolean(
        &mesh_a,
        translation(-3.0, 0.0, 0.0),
        &mesh_b,
        translation(3.0, 0.0, 0.0),
        CsgModes::Union,
        &BooleanOptions::default(),
    );

    assert!(result.face_colors.is_none());
}

pub fn manifold_run_data_extracts_face_colors_correctly() {
    // Two non-overlapping cubes - every face must come from exactly one source mesh
    let mesh_a = create_cube(10.0, 10.0, 10.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);

    let result = do_array(
        &[
            (&mesh_a, translation(-20.0, 0.0, 0.0)),
            (&mesh_b, translation(20.0, 0.0, 0.0)),
        ],
        &colors(&[Color::RED, Color::BLUE]),
    );

    assert_parallel(&result);
    // Count red and blue faces - should be 12 each (cube = 12 triangles)
    let red_count = face_colors(&result).iter().filter(|c| is_red(c)).count();
    let blue_count = face_colors(&result).iter().filter(|c| is_blue(c)).count();
    assert_eq!(red_count, 12);
    assert_eq!(blue_count, 12);
    assert_eq!(red_count + blue_count, result.faces.len());
}

pub fn face_colors_survive_full_cleanup_pipeline() {
    // The full pipeline CombineParticipants does: DoArray -> CleanAndMerge ->
    // RemoveUnusedVertices
    let mesh_a = create_cube(10.0, 10.0, 10.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);

    let mut result = do_array(
        &[
            (&mesh_a, translation(-3.0, 0.0, 0.0)),
            (&mesh_b, translation(3.0, 0.0, 0.0)),
        ],
        &colors(&[Color::RED, Color::BLUE]),
    );
    assert!(result.face_colors.is_some());

    result.clean_and_merge();
    result.remove_unused_vertices();
    // Combine() also calls CleanAndMerge a second time
    result.clean_and_merge();

    assert_parallel(&result);
    assert!(has_red(&result));
    assert!(has_blue(&result));
}

pub fn intersect_preserves_both_face_colors() {
    let mesh_a = create_cube(10.0, 10.0, 10.0);
    let mesh_b = create_cube(10.0, 10.0, 10.0);

    let result = do_boolean(
        &mesh_a,
        translation(-3.0, 0.0, 0.0),
        &mesh_b,
        translation(3.0, 0.0, 0.0),
        CsgModes::Intersect,
        &colors(&[Color::RED, Color::BLUE]),
    );

    assert_parallel(&result);
    assert!(has_red(&result));
    assert!(has_blue(&result));
}

pub fn subtract_from_mesh_with_face_colors_preserves_colors() {
    // First, create a combined mesh with face colors via boolean union
    let cube_a = create_cube(15.0, 15.0, 15.0);
    let cube_b = create_cube(15.0, 15.0, 15.0);
    let combined_mesh = do_boolean(
        &cube_a,
        translation(3.0, 0.0, 0.0),
        &cube_b,
        translation(-3.0, 0.0, 0.0),
        CsgModes::Union,
        &colors(&[Color::BLUE, Color::GREEN]),
    );

    let combined_colors = combined_mesh
        .face_colors
        .as_deref()
        .expect("Union with mesh_colors should produce face colors");
    assert!(
        combined_colors.iter().any(|c| c.blue == 255 && c.red == 0),
        "Combined mesh should have blue faces"
    );
    assert!(
        combined_colors.iter().any(|c| c.green == 255 && c.red == 0),
        "Combined mesh should have green faces"
    );

    // Now subtract this combined mesh (which has face colors) from a larger cube
    let keep_cube = create_cube(40.0, 40.0, 40.0);
    let result_mesh = do_boolean(
        &keep_cube,
        Matrix4X4::IDENTITY,
        &combined_mesh,
        Matrix4X4::IDENTITY,
        CsgModes::Subtract,
        // White is placeholder for remove
        &colors(&[Color::RED, Color::WHITE]),
    );

    assert!(result_mesh.faces.len() > 12);
    let result_colors = result_mesh
        .face_colors
        .as_deref()
        .expect("Subtract result should have face colors");
    assert!(
        result_colors.iter().any(is_red),
        "Keep surfaces should be red"
    );
    assert!(
        result_colors.iter().any(|c| c.blue == 255 && c.red == 0),
        "Cavity should have blue faces from combined mesh"
    );
    assert!(
        result_colors.iter().any(|c| c.green == 255 && c.red == 0),
        "Cavity should have green faces from combined mesh"
    );
}
