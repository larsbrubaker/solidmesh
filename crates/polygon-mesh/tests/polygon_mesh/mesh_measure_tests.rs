//! Mesh measurements: volume and surface area.

use polygon_mesh::solids::platonic_solids;
use vector_math::Matrix4X4;

const EPSILON: f64 = 1e-9;

fn assert_within(actual: f64, expected: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= EPSILON,
        "expected {expected}, got {actual}: {because}"
    );
}

#[test]
fn scaled_cube_has_box_volume_and_area() {
    let cube = platonic_solids::create_cube(2.0, 3.0, 4.0);
    assert_within(cube.get_volume(), 24.0, "");
    assert_within(cube.get_surface_area(), 52.0, "");
}

#[test]
fn flipped_cube_has_negative_volume_and_the_same_area() {
    let mut cube = platonic_solids::create_cube(2.0, 3.0, 4.0);
    cube.reverse_faces();
    assert_within(
        cube.get_volume(),
        -24.0,
        "an inward-wound closed mesh encloses its volume with the opposite sign",
    );
    assert_within(
        cube.get_surface_area(),
        52.0,
        "area does not depend on winding",
    );
}

#[test]
fn open_box_volume_depends_on_where_the_origin_is() {
    // Remove the two z = +0.5 triangles (create_cube's "front", its first two faces) of a
    // unit cube centered on the origin.
    let mut centered = platonic_solids::create_cube(1.0, 1.0, 1.0);
    centered.faces.drain(0..2);
    // The missing z = +0.5 face would have added area * distance-from-origin / 3 = 1 * 0.5 / 3.
    assert_within(centered.get_volume(), 1.0 - 1.0 / 6.0, "");
    assert_within(centered.get_surface_area(), 5.0, "");

    // Moving the same open box up by 0.5 puts its missing face at z = 1 and changes the sum,
    // which is why the sum is not a volume on an open mesh.
    let raised = centered.get_volume_transformed(Matrix4X4::create_translation(0.0, 0.0, 0.5));
    assert_within(raised, 1.0 - 1.0 / 3.0, "");
}

#[test]
fn transform_overload_matches_manual_scaling() {
    let unit_cube = platonic_solids::create_cube(1.0, 1.0, 1.0);
    let scaled_cube = platonic_solids::create_cube(2.0, 3.0, 4.0);
    let transform = Matrix4X4::create_scale(2.0, 3.0, 4.0)
        * Matrix4X4::create_rotation_z(0.7)
        * Matrix4X4::create_translation(5.0, -6.0, 7.0);

    assert_within(
        unit_cube.get_volume_transformed(transform),
        scaled_cube.get_volume(),
        "",
    );
    assert_within(
        unit_cube.get_surface_area_transformed(transform),
        scaled_cube.get_surface_area(),
        "",
    );

    // A mirror turns the mesh inside out, so its volume changes sign with the determinant.
    let mirror = Matrix4X4::create_scale(-2.0, 3.0, 4.0);
    assert_within(unit_cube.get_volume_transformed(mirror), -24.0, "");
    assert_within(unit_cube.get_surface_area_transformed(mirror), 52.0, "");
}
