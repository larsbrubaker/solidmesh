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

#[test]
fn rust_only_a_cap_plane_half_space_stands_past_the_plane_and_clips_or_withdraws() {
    use geometry_processing::bevel::bevel_bead_clip::is_usable_keep_set;
    use geometry_processing::bevel::bevel_cap_clip::{apply, half_space, Candidate, ClippedCap};
    use geometry_processing::bevel::BevelEndCap;
    use polygon_mesh::csg::{BooleanError, CsgModes};
    use polygon_mesh::Mesh;
    use vector_math::Matrix4X4;

    let cutter = platonic_solids::create_cube(2.0, 2.0, 2.0);
    let bounds = cutter.get_axis_aligned_bounding_box();
    let cap = BevelEndCap::new(Vector3::new(1.0, 0.0, 0.0), 0.5);
    let box_bounds = half_space(&cap, &bounds).get_axis_aligned_bounding_box();
    // The box's near face is on the plane x = 0.5 and it reaches out along +x.
    assert!((box_bounds.min_xyz.x - 0.5).abs() < 1e-6);
    assert!(box_bounds.max_xyz.x > 10.0);
    assert!(is_usable_keep_set(Some(&cutter)));
    assert!(!is_usable_keep_set(None));

    let candidates = vec![Candidate::new(
        0,
        7,
        1.5,
        vec![ClippedCap::new(cap.clone(), Vector3::new(1.0, 2.0, 3.0))],
    )];

    // A kernel that clips: the cutter is replaced.
    let clips = |operands: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
        assert_eq!(mode, CsgModes::Subtract);
        assert_eq!(operands.len(), 2);
        Ok(platonic_solids::create_cube(1.0, 1.0, 1.0))
    };
    let mut cutters = vec![cutter.clone()];
    let withdrawals = apply(&mut cutters, &candidates, &clips).expect("not cancelled");
    assert!(withdrawals.is_empty());
    assert_eq!(cutters.len(), 1);

    // A kernel that refuses: the cutter is withdrawn and its group named.
    let refuses = |_: &[(Mesh, Matrix4X4)], _: CsgModes| -> Result<Mesh, BooleanError> {
        Err(BooleanError::Kernel {
            message: "no".into(),
        })
    };
    let mut cutters = vec![cutter.clone()];
    let withdrawals = apply(&mut cutters, &candidates, &refuses).expect("not cancelled");
    assert_eq!(withdrawals.len(), 1);
    assert_eq!(withdrawals[0].group_index, 7);
    assert!(cutters.is_empty());

    // A cancel passes up.
    let cancels = |_: &[(Mesh, Matrix4X4)], _: CsgModes| -> Result<Mesh, BooleanError> {
        Err(BooleanError::Cancelled)
    };
    let mut cutters = vec![cutter];
    assert!(matches!(
        apply(&mut cutters, &candidates, &cancels),
        Err(BooleanError::Cancelled)
    ));
}

#[test]
fn rust_only_beads_are_gathered_by_the_corner_void_they_share() {
    use geometry_processing::bevel::bevel_bead_clip::Clippable;
    use geometry_processing::bevel::bevel_bead_corner_pieces::beads_by_corner;
    use std::rc::Rc;
    let cube = |size: f64| platonic_solids::create_cube(size, size, size);
    let corner = Rc::new(cube(1.0));
    // Equal geometry, a different corner.
    let twin = Rc::new(cube(1.0));
    let beads = vec![
        Clippable::new(
            cube(2.0),
            cube(9.0),
            Some(vec![Rc::clone(&corner)]),
            None,
            None,
        ),
        Clippable::new(cube(3.0), cube(9.0), None, None, None),
        Clippable::new(
            cube(4.0),
            cube(9.0),
            Some(vec![Rc::clone(&corner), Rc::clone(&twin)]),
            None,
            None,
        ),
    ];
    let grouped = beads_by_corner(&beads);
    assert_eq!(grouped.len(), 2);
    assert!(Rc::ptr_eq(&grouped[0].0, &corner));
    assert_eq!(grouped[0].1.len(), 2);
    assert!(Rc::ptr_eq(&grouped[1].0, &twin));
    assert_eq!(grouped[1].1.len(), 1);
}
