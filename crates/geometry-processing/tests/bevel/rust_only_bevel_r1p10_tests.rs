//! Rust-only checks of bevel-core package R1.P10's working-mesh conditioning: the folded-flap drop,
//! the seam readings, openness, the signed volume and the inside-out test. Their C# coverage comes
//! through the whole-bevel test classes of later packages.

use geometry_processing::bevel::bevel_working_mesh::{
    folded_flaps_dropped, is_inside_out, is_usable_operand, open_edge_count,
    seam_epsilon_for_positions, seam_tolerance_for, signed_volume,
};
use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3;

#[test]
fn rust_only_a_cube_reads_closed_outward_and_usable() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    assert!((signed_volume(&cube) - 8000.0).abs() < 1e-6);
    assert!(!is_inside_out(&cube));
    assert_eq!(open_edge_count(&cube.faces), 0);
    assert!(is_usable_operand(Some(&cube), None));
    assert!(!is_usable_operand(None, None));
    // The diagonal (20 sqrt 3) is the greater term for a cube centred on the origin.
    assert!((seam_tolerance_for(&cube) - 20.0 * 3f64.sqrt() * 1e-5).abs() < 1e-12);
    assert_eq!(
        seam_epsilon_for_positions(&[Vector3::ZERO, Vector3::new(3.0, 4.0, 0.0)]),
        5.0 * 5e-4
    );

    let mut inside_out = cube.clone();
    inside_out.reverse_faces();
    assert!(is_inside_out(&inside_out));
}

#[test]
fn rust_only_a_folded_flap_is_dropped_in_pairs() {
    let mut cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let face = cube.faces[0];
    let vertices = cube.vertices.clone();
    // A face and its reverse on the same three vertices: a flap that cancels.
    cube.faces.add(face.v0, face.v1, face.v2, &vertices);
    cube.faces.add(face.v0, face.v2, face.v1, &vertices);
    let before = cube.faces.len();
    assert_eq!(folded_flaps_dropped(&mut cube), 1);
    assert_eq!(cube.faces.len(), before - 2);
    assert_eq!(open_edge_count(&cube.faces), 0);
}
