//! Booleans on inputs that can kill a native boolean engine (identical operands, whose result is
//! empty or degenerate). The kernel is the only boolean backend
//! (`boolean_processing::do_array` over manifold-rust), so answering at all - with a result, not
//! an error - is the assertion.

use polygon_mesh::csg::{
    boolean_processing, BooleanOptions, CsgModes, ProcessingModes, ProcessingResolution,
};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Matrix4X4;

/// `do_array` with polygon processing at 64-bit input and output resolution, no reporter and no
/// cancellation.
fn do_array(items: &[(&Mesh, Matrix4X4)], mode: CsgModes) -> Mesh {
    let options = BooleanOptions {
        processing_mode: ProcessingModes::Polygons,
        input_resolution: ProcessingResolution::R64,
        output_resolution: ProcessingResolution::R64,
        ..BooleanOptions::default()
    };
    boolean_processing::do_array(items, mode, &CancelToken::new(), &options)
        .expect("the boolean returns a result")
}

/// Subtracting an identical cube from itself is a degenerate case: the result is empty, and must
/// come back (possibly empty) without crashing.
#[test]
fn subtract_identical_meshes_does_not_crash() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let items = [(&cube, Matrix4X4::IDENTITY), (&cube, Matrix4X4::IDENTITY)];

    // Should not fail or crash the process
    let _result = do_array(&items, CsgModes::Subtract);
}

/// Combining two identical overlapping meshes is another degenerate case.
#[test]
fn combine_identical_meshes_does_not_crash() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let items = [(&cube, Matrix4X4::IDENTITY), (&cube, Matrix4X4::IDENTITY)];

    let _result = do_array(&items, CsgModes::Union);
}

/// A normal subtract between non-degenerate cubes must still work correctly.
#[test]
fn subtract_overlapping_cubes_produces_non_empty_result() {
    let big_cube = create_cube(20.0, 20.0, 20.0);
    let small_cube = create_cube(5.0, 5.0, 5.0);
    let items = [
        (&big_cube, Matrix4X4::IDENTITY),
        (&small_cube, Matrix4X4::IDENTITY),
    ];

    let result = do_array(&items, CsgModes::Subtract);

    assert!(!result.faces.is_empty());
}
