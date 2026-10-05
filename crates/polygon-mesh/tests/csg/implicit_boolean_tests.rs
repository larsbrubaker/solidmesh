//! Port of `Agg.PolygonMesh/ImplicitBooleanTests.cs`: the implicit-surface processing modes
//! (marching cubes, dual contouring) resample the operands instead of running the kernel, so
//! their answers are only checked by volume. C#'s parameterized
//! `NarySubtractRemovesEveryCutterFromTheFirstOperand(mode)` is one test per mode here.

use polygon_mesh::csg::boolean_processing::{do_array, do_boolean};
use polygon_mesh::csg::minkowski_processing::sphere_mesh;
use polygon_mesh::csg::{BooleanOptions, CsgModes, ProcessingModes, ProcessingResolution};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Matrix4X4;

// A 20 mm cube with two 10 x 10 columns punched through opposite quarters: 8000 - 2 x 2000.
const CUBE_SIZE: f64 = 20.0;
const EXPECTED_VOLUME: f64 = 4000.0;

fn implicit_options(mode: ProcessingModes) -> BooleanOptions {
    BooleanOptions {
        processing_mode: mode,
        input_resolution: ProcessingResolution::R64,
        output_resolution: ProcessingResolution::R64,
        ..BooleanOptions::default()
    }
}

fn nary_subtract_removes_every_cutter_from_the_first_operand(mode: ProcessingModes) {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let column = create_cube(10.0, 10.0, 30.0);
    let items: [(&Mesh, Matrix4X4); 3] = [
        (&cube, Matrix4X4::IDENTITY),
        (&column, Matrix4X4::create_translation(5.0, 5.0, 0.0)),
        (&column, Matrix4X4::create_translation(-5.0, -5.0, 0.0)),
    ];

    let result = do_array(
        &items,
        CsgModes::Subtract,
        &CancelToken::new(),
        &implicit_options(mode),
    )
    .expect("the implicit modes do not fail");

    let volume = result.get_volume();
    assert!(
        (volume - EXPECTED_VOLUME).abs() < EXPECTED_VOLUME * 0.05,
        "{mode:?}: volume {volume}"
    );
}

#[test]
fn nary_subtract_removes_every_cutter_from_the_first_operand_marching_cubes() {
    nary_subtract_removes_every_cutter_from_the_first_operand(ProcessingModes::MarchingCubes);
}

#[test]
fn nary_subtract_removes_every_cutter_from_the_first_operand_dual_contouring() {
    nary_subtract_removes_every_cutter_from_the_first_operand(ProcessingModes::DualContouring);
}

#[test]
fn two_operand_subtract_samples_at_the_requested_resolution() {
    // A sphere's signed distance is curved, so a grid of a handful of cells (the resolution's
    // exponent read as a cell count) visibly loses volume where 64 cells do not.
    let sphere = sphere_mesh(10.0, 64).expect("sphere");
    let expected = sphere.get_volume() - sphere.get_volume() / 2.0;

    let result = do_boolean(
        &sphere,
        Matrix4X4::IDENTITY,
        &create_cube(30.0, 30.0, 30.0),
        Matrix4X4::create_translation(0.0, 0.0, -15.0),
        CsgModes::Subtract,
        &CancelToken::new(),
        &implicit_options(ProcessingModes::MarchingCubes),
    )
    .expect("the implicit modes do not fail");

    let volume = result.get_volume();
    assert!(
        (volume - expected).abs() < expected * 0.03,
        "volume {volume} expected {expected}"
    );
}
