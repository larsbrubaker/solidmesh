//! Bevel tool plane accuracy: a 2 mm chamfer on one 20 mm cube edge must remove 40 mm^3 exactly,
//! and a hand-built wedge subtracted from the cube does - the control that exonerates the kernel.

use solidmesh_math::{Matrix4X4, Vector3Float};
use solidmesh_mesh::csg::boolean_processing::do_array;
use solidmesh_mesh::csg::{BooleanOptions, CsgModes};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::signed_volume;

const CUBE_SIZE: f64 = 20.0;

const DISTANCE: f64 = 2.0;

/// The analytic answer: a 2 mm chamfer of one 20 mm edge is the right prism of leg 2 run the whole
/// length, so 20 * 2 * 2 / 2.
const ANALYTIC_CUT: f64 = CUBE_SIZE * DISTANCE * DISTANCE / 2.0;

/// The control that exonerates the kernel. The exact chamfer wedge, built by hand at coordinates
/// every one of which is exact in a float, subtracts to the analytic volume with nothing left over
/// - whether its faces are coplanar with the cube's or it overshoots them in every direction.
#[test]
fn a_hand_built_wedge_subtracts_exactly() {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);

    // The wedge for the edge at y = z = 10: the material where y + z > 18, drawn in (y, z) and run
    // along x. Coplanar with the cube on two faces and both ends.
    let flush = prism(-10.0, 10.0, &[(8.0, 10.0), (10.0, 8.0), (10.0, 10.0)]);

    // The same cut from a tool that overshoots the part on all six sides, which is the shape
    // every bevel tool really is. Only the region inside the cube can differ, and it does not.
    let proud = prism(-11.0, 11.0, &[(6.0, 12.0), (12.0, 6.0), (12.0, 12.0)]);

    let flush_removed = removed(&cube, &flush);
    assert!(
        (flush_removed - ANALYTIC_CUT).abs() <= 1e-9,
        "the kernel cuts the plane it was given, with no tolerance of its own to lose: \
         {flush_removed}"
    );
    let proud_removed = removed(&cube, &proud);
    assert!(
        (proud_removed - ANALYTIC_CUT).abs() <= 1e-9,
        "overshooting the part changes nothing the kernel takes out of it: {proud_removed}"
    );
}

/// How much material a tool takes out of the cube, measured off the result rather than off the
/// tool.
fn removed(cube: &Mesh, tool: &Mesh) -> f64 {
    let result = do_array(
        &[(cube, Matrix4X4::IDENTITY), (tool, Matrix4X4::IDENTITY)],
        CsgModes::Subtract,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("the subtraction builds");

    CUBE_SIZE.powi(3) - signed_volume(&result)
}

/// A triangular prism from a cross section given in (y, z) and run along x, wound so the solid is
/// positive. Hand built rather than swept, so nothing in the bevel code decides where its faces
/// are.
fn prism(x0: f64, x1: f64, section: &[(f64, f64)]) -> Mesh {
    let mut mesh = Mesh::new();

    for point in section {
        mesh.vertices.push(Vector3Float::new(x0, point.0, point.1));
    }

    for point in section {
        mesh.vertices.push(Vector3Float::new(x1, point.0, point.1));
    }

    let sides = section.len() as i32;

    for i in 0..sides {
        let next = (i + 1) % sides;

        let vertices = mesh.vertices.clone();
        mesh.faces.add(i, next, sides + next, &vertices);
        mesh.faces.add(i, sides + next, sides + i, &vertices);
    }

    // The two caps, wound to face out of their own ends of the run rather than along it
    for i in 2..sides {
        let vertices = mesh.vertices.clone();
        mesh.faces.add(0, i, i - 1, &vertices);
        mesh.faces.add(sides, sides + i - 1, sides + i, &vertices);
    }

    mesh
}
