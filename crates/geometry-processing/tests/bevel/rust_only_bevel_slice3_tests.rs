//! Rust-only checks of the third bevel leaf cluster (`FloatGrid`, and the level line's
//! `ChainSetback`, `CurvedEndFace` and `BandGroup`). The whole-bevel tests (the hole-row bridge
//! tests and friends, which need the level-line build) also cover them; these pin the leaf
//! behaviour alone.

use geometry_processing::bevel::float_grid;
use geometry_processing::bevel::level_line::{chain_setback, curved_end_face, BandGroup};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}

#[test]
fn rust_only_the_float_grid_step_is_one_ulp_at_the_largest_coordinate() {
    assert_eq!(float_grid::step_at(0.0), 0.0);
    assert_eq!(float_grid::step_at(1.0), f64::from(f32::EPSILON));
    assert_eq!(float_grid::step_at(-1.0), f64::from(f32::EPSILON));
    // 20 lies in [16, 32), where a float's step is 2^-19 - the 1.9e-6 the seam ring's remarks cite.
    assert_eq!(
        float_grid::step_at_point(Vector3::new(3.0, -20.0, 1.0)),
        2f64.powi(-19)
    );
}

#[test]
fn rust_only_a_square_edge_sets_back_by_the_radius_and_a_chamfer_by_its_size() {
    let square = chain_setback::for_fillet(3.0, Vector3::UNIT_X, Vector3::UNIT_Y * 5.0);
    assert!((square - 3.0).abs() < 1e-12);

    // A 45 degree turn (a blunt edge) asks less, a 135 degree one (a sharp edge) more.
    let blunt = chain_setback::for_fillet(3.0, Vector3::UNIT_X, Vector3::new(1.0, 1.0, 0.0));
    let sharp = chain_setback::for_fillet(3.0, Vector3::UNIT_X, Vector3::new(-1.0, 1.0, 0.0));
    assert!((blunt - 3.0 * (std::f64::consts::PI / 8.0).tan()).abs() < 1e-12);
    assert!(blunt < 3.0 && sharp > 3.0);

    assert_eq!(chain_setback::for_chamfer(2.5), 2.5);
    assert!(chain_setback::is_uniform(2.0, 2.0));
    assert!(!chain_setback::is_uniform(2.0, 2.1));
    assert!(
        !chain_setback::is_uniform(0.0, 0.0),
        "a zero setback is never uniform"
    );
}

#[test]
fn rust_only_a_cube_face_is_flat_and_a_shallow_fold_is_a_round() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let positions: Vec<Vector3> = (0..cube.vertices.len() as i32)
        .map(|v| position(&cube, v))
        .collect();
    let indices: Vec<(i32, i32, i32)> = cube.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let top: Vec<(i32, i32, i32)> = cube
        .faces
        .iter()
        .filter(|f| {
            positions[f.v0 as usize].z > 9.0
                && positions[f.v1 as usize].z > 9.0
                && positions[f.v2 as usize].z > 9.0
        })
        .map(|f| (f.v0, f.v1, f.v2))
        .collect();
    assert_eq!(top.len(), 2);

    assert!(!curved_end_face::bends_on_softly(
        &positions,
        &indices,
        Vector3::UNIT_Z,
        top.clone(),
        30.0
    ));
    assert!(curved_end_face::bends_on_softly(
        &positions,
        &indices,
        Vector3::UNIT_Z,
        top,
        91.0
    ));

    // Two triangles folded 10 degrees along their shared edge: the second is a soft turn.
    let rise = 10f64.to_radians().tan();
    let fold = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, rise),
    ];
    let tris = vec![(0, 1, 2), (0, 3, 1)];
    assert!(curved_end_face::bends_on_softly(
        &fold,
        &tris,
        Vector3::UNIT_Z,
        [(0, 1, 2)],
        30.0
    ));
    assert!(
        !curved_end_face::bends_on_softly(&fold, &tris, Vector3::UNIT_Z, [(9, 9, 9)], 30.0),
        "a seed that is not a triangle grows nothing"
    );
}

#[test]
fn rust_only_a_band_group_is_a_side_group_until_told_otherwise() {
    let group = BandGroup::default();
    assert_eq!(group.end_of, -1);
    assert!(group.members.is_empty() && group.carried_ends.is_empty());
}
