//! The gate every level-line answer passes (`part::self_intersection_check`): an answer that
//! crosses itself is caught only where the part it was built on did not already cross itself, a
//! clean answer passes, and a cancel stops the check.

use geometry_processing::bevel::part::answer_mesh::the_mesh_of;
use geometry_processing::bevel::part::self_intersection_check::the_answer_intersects_itself_where_the_source_did_not as crosses_itself;
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Cancelled, Mesh};
use vector_math::{Matrix4X4, Vector3};

/// A mesh as the kernel hands one over: positions and triangles.
fn the_kernels_view(mesh: &Mesh) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    (
        mesh.vertices
            .iter()
            .map(|vertex| vertex.as_vector3())
            .collect(),
        mesh.faces
            .iter()
            .map(|face| (face.v0, face.v1, face.v2))
            .collect(),
    )
}

/// Two 10 mm cubes, the second moved 5 mm along x so they pass through each other, as one mesh.
fn two_cubes_through_each_other() -> Mesh {
    let cube = create_cube(10.0, 10.0, 10.0);
    let (mut positions, mut indices) = the_kernels_view(&cube);
    let offset = positions.len() as i32;
    positions.extend(
        cube.vertices
            .iter()
            .map(|vertex| vertex.as_vector3() + Vector3::new(5.0, 0.0, 0.0)),
    );
    indices.extend(
        cube.faces
            .iter()
            .map(|face| (face.v0 + offset, face.v1 + offset, face.v2 + offset)),
    );
    the_mesh_of(&positions, &indices)
}

#[test]
fn an_answer_that_newly_crosses_itself_is_caught() {
    let crossing = two_cubes_through_each_other();
    assert_eq!(
        classify_boolean_operand(&crossing, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::SelfIntersecting,
        "the answer under test has to cross itself"
    );

    let (positions, indices) = the_kernels_view(&create_cube(10.0, 10.0, 10.0));
    assert_eq!(
        crosses_itself(&crossing, &positions, &indices, &CancelToken::new()),
        Ok(true)
    );
}

#[test]
fn an_answer_crossing_itself_where_its_source_did_too_is_not_the_answers_fault() {
    let crossing = two_cubes_through_each_other();
    let (positions, indices) = the_kernels_view(&crossing);
    assert_eq!(
        crosses_itself(&crossing, &positions, &indices, &CancelToken::new()),
        Ok(false)
    );
}

#[test]
fn a_clean_answer_passes() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let (positions, indices) = the_kernels_view(&cube);
    assert_eq!(
        crosses_itself(&cube, &positions, &indices, &CancelToken::new()),
        Ok(false)
    );
}

#[test]
fn a_cancel_stops_the_check() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let (positions, indices) = the_kernels_view(&cube);
    let cancelled = CancelToken::new();
    cancelled.cancel();
    assert_eq!(
        crosses_itself(&cube, &positions, &indices, &cancelled),
        Err(Cancelled)
    );
}
