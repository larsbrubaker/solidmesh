//! How far a bead's tangent line carries on past the end of its
//! crease, and what stops it there.

use std::sync::Arc;

use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::Mesh;

use geometry_processing::bevel::bevel_edge_tool_builder::tangent_distance;
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::{BevelFaceRunOut, BevelRunOutBound};
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::nose_in_notch_test_meshes as nose;

const RADIUS: f64 = 5.0;

fn tangent_offset() -> f64 {
    let wedge =
        radians_to_degrees((-nose::left_slope_normal().dot(nose::upper_nose_normal())).acos());

    tangent_distance(RADIUS, wedge, true)
}

fn tangent_line_at(crease_end: Vector3) -> (Vector3, Vector3) {
    let along = (nose::upper_crease_end() - nose::left_ridge_end()).get_normal();
    let mut into_the_face = nose::left_slope_normal().cross(along).get_normal();

    // Away from the nose, which on this slope is up.
    if into_the_face.z < 0.0 {
        into_the_face = -into_the_face;
    }

    (crease_end + (into_the_face * tangent_offset()), along)
}

fn run_out_of(mesh: Mesh) -> BevelFaceRunOut {
    BevelFaceRunOut::create(Arc::new(mesh), None, DEFAULT_SURFACE_TOLERANCE_DEGREES)
}

fn slope_face(part: &Mesh) -> i32 {
    for face in 0..part.faces.len() {
        let mesh_face = part.faces[face];
        let a = Vector3::from_vector3_float(part.vertices[mesh_face.v0 as usize]);
        let b = Vector3::from_vector3_float(part.vertices[mesh_face.v1 as usize]);
        let c = Vector3::from_vector3_float(part.vertices[mesh_face.v2 as usize]);
        let normal = (b - a).cross(c - a).get_normal();

        if normal.dot(nose::left_slope_normal()) > 0.999 {
            return face as i32;
        }
    }

    panic!("the fixture has to have a left slope to ride");
}

/// Past the ridge end the run carries on to the cube's convex corner edge.
#[test]
fn past_the_ridge_end_the_run_carries_on_to_the_cubes_convex_corner_edge() {
    let part = nose::create();
    let slope = slope_face(&part);
    let run_out = run_out_of(part);
    let (at, along) = tangent_line_at(nose::left_ridge_end());

    let measured = run_out
        .try_measure(slope, at, -along, 40.0)
        .expect("the tangent line starts on the slope the bead rides");

    assert!(
        (measured.distance - 6.2600).abs() <= 0.005,
        "the control's upper-left tangent line reaches the cube's front corner edge 6.26 mm on: {}",
        measured.distance
    );
    assert_eq!(
        measured.bound,
        BevelRunOutBound::ConvexEdge,
        "the slope ENDS there, meeting the cube's left face at 45 degrees"
    );
}

/// Past the crease end the run carries on to the notch's concave centre crease.
#[test]
fn past_the_crease_end_the_run_carries_on_to_the_notches_concave_centre_crease() {
    let part = nose::create();
    let slope = slope_face(&part);
    let run_out = run_out_of(part);
    let (at, along) = tangent_line_at(nose::upper_crease_end());

    let measured = run_out
        .try_measure(slope, at, along, 40.0)
        .unwrap_or_default();

    assert!(
        (measured.distance - 1.5556).abs() <= 0.002,
        "the control's tangent line reaches the centre crease 1.556 mm past its own crease end: {}",
        measured.distance
    );
    assert_eq!(
        measured.bound,
        BevelRunOutBound::ConcaveEdge,
        "the other slope RISES there, a wall the bead runs out flat against"
    );
}

/// A run that ends where its own face ends has nothing to carry on into.
#[test]
fn a_run_that_ends_where_its_own_face_ends_has_nothing_to_carry_on_into() {
    let cube = create_cube(20.0, 20.0, 20.0);

    let mut top = -1;

    for face in 0..cube.faces.len() {
        if top >= 0 {
            break;
        }

        let mesh_face = cube.faces[face];
        let z = |v: i32| f64::from(cube.vertices[v as usize].z);

        if z(mesh_face.v0) > 9.9 && z(mesh_face.v1) > 9.9 && z(mesh_face.v2) > 9.9 {
            top = face as i32;
        }
    }

    let run_out = run_out_of(cube);
    let measured = run_out
        .try_measure(
            top,
            Vector3::new(5.0, 9.999, 10.0),
            Vector3::new(0.0, 1.0, 0.0),
            40.0,
        )
        .unwrap_or_default();

    assert!(
        measured.distance < 0.01,
        "the walk starts a thousandth from the top face's own rim and heads straight at it"
    );
    assert_eq!(
        measured.bound,
        BevelRunOutBound::ConvexEdge,
        "a cube's top edge is convex, which is the ending that tapers"
    );
}
