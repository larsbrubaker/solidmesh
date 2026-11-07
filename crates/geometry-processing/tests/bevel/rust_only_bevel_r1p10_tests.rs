//! Rust-only checks of bevel-core package R1.P10's working-mesh conditioning: the folded-flap drop,
//! the seam readings, openness, the signed volume and the inside-out test. The whole-build tests
//! also cover them.

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

#[test]
fn rust_only_sliver_readings_and_a_clean_cube_left_alone() {
    use geometry_processing::bevel::bevel_t_vertex_slivers::{
        dropped_from, stands_on, try_read_t_vertex,
    };
    use geometry_processing::bevel::bevel_thin_slivers::{collapsed_from, the_one_it_is};
    use polygon_mesh::{Face, Mesh};
    use vector_math::Vector3Float;

    let v = |x: f64, y: f64, z: f64| Vector3::new(x, y, z);
    assert!(stands_on(
        v(0.0, 0.0, 0.0),
        v(10.0, 0.0, 0.0),
        v(4.0, 1e-9, 0.0),
        1e-6
    ));
    assert!(!stands_on(
        v(0.0, 0.0, 0.0),
        v(10.0, 0.0, 0.0),
        v(4.0, 1e-3, 0.0),
        1e-6
    ));
    assert!(!stands_on(
        v(0.0, 0.0, 0.0),
        v(10.0, 0.0, 0.0),
        v(1e-8, 0.0, 0.0),
        1e-6
    ));

    let mut sliver = Mesh::default();
    sliver.vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(10.0, 0.0, 0.0),
        Vector3Float::new(4.0, 0.0, 0.0),
    ];
    let face = Face::new(0, 1, 2, &sliver.vertices);
    assert_eq!(try_read_t_vertex(&sliver, &face, 1e-4), Some(2));

    let mut groups = vec![0, 0, 1, 2];
    assert_eq!(the_one_it_is(&mut groups, 3), 0);

    let mut cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let faces = cube.faces.len();
    assert_eq!(dropped_from(&mut cube), 0);
    assert_eq!(collapsed_from(&mut cube), 0);
    assert_eq!(cube.faces.len(), faces);
}

#[test]
fn rust_only_the_bead_clips_run_through_the_combiner_and_the_padded_hull() {
    use geometry_processing::bevel::bevel_bead_clip::{clip_to, join};
    use geometry_processing::bevel::bevel_bead_corner_pieces::piece;
    use geometry_processing::bevel::bevel_source_hull::{build, lies_inside};
    use polygon_mesh::csg::{BooleanError, CsgModes};
    use polygon_mesh::Mesh;
    use std::cell::RefCell;
    use vector_math::Matrix4X4;

    let part = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let hull = build(Some(&part)).expect("a cube has a hull");
    let size = hull.get_axis_aligned_bounding_box().size();
    assert!(
        size.x > 20.0 && size.x < 20.001,
        "padded a hair outward: {size:?}"
    );
    assert!(lies_inside(Some(&hull), Some(&part)));
    assert!(!lies_inside(
        Some(&hull),
        Some(&platonic_solids::create_cube(30.0, 30.0, 30.0))
    ));
    assert!(lies_inside(None, Some(&part)));

    // A combiner that records each call's mode and operand count and answers with a small cube.
    let calls = RefCell::new(Vec::new());
    let combine = |operands: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
        calls.borrow_mut().push((mode, operands.len()));
        Ok(platonic_solids::create_cube(2.0, 2.0, 2.0))
    };

    let bead = platonic_solids::create_cube(30.0, 30.0, 30.0);
    let clipped =
        clip_to(&part, Matrix4X4::IDENTITY, &bead, &[], &combine, None).expect("not cancelled");
    assert!(clipped.is_some());
    // The keep set (the part alone), then the bead with it and the hull it reaches past.
    assert_eq!(
        *calls.borrow(),
        vec![(CsgModes::Union, 1), (CsgModes::Intersect, 3)]
    );

    calls.borrow_mut().clear();
    let lone = join(vec![part.clone()], &combine).expect("not cancelled");
    assert!(
        lone.is_some() && calls.borrow().is_empty(),
        "a lone piece needs no union"
    );

    let refuses = |_: &[(Mesh, Matrix4X4)], _: CsgModes| -> Result<Mesh, BooleanError> {
        Err(BooleanError::Kernel {
            message: "no".into(),
        })
    };
    assert!(piece(
        &bead,
        Matrix4X4::IDENTITY,
        Some(&hull),
        &part,
        &part,
        &refuses,
        None
    )
    .expect("not cancelled")
    .is_none());
    let cancels = |_: &[(Mesh, Matrix4X4)], _: CsgModes| -> Result<Mesh, BooleanError> {
        Err(BooleanError::Cancelled)
    };
    assert!(join(vec![part.clone(), part], &cancels).is_err());
}

#[test]
fn rust_only_the_shadow_body_and_clip_geometry() {
    use geometry_processing::bevel::bevel_shadow_body::{
        convex_region, enclosed_volume, round_the_centroid, try_meet,
    };
    use geometry_processing::bevel::bevel_shadow_clip::{self, clip_to_half_space, Obstruction};
    use polygon_mesh::MeshEdgeGraph;

    let v = |x: f64, y: f64, z: f64| Vector3::new(x, y, z);

    // Six planes boxing [0,2] x [0,3] x [0,4], cut out of a big cube about the box.
    let planes = [
        (v(1.0, 0.0, 0.0), 0.0),
        (v(-1.0, 0.0, 0.0), -2.0),
        (v(0.0, 1.0, 0.0), 0.0),
        (v(0.0, -1.0, 0.0), -3.0),
        (v(0.0, 0.0, 1.0), 0.0),
        (v(0.0, 0.0, -1.0), -4.0),
    ];
    let region = convex_region(&planes, v(1.0, 1.5, 2.0), 40.0).expect("a box");
    assert!((enclosed_volume(&region) - 24.0).abs() < 1e-6);

    let square = [
        v(0.0, 0.0, 0.0),
        v(2.0, 0.0, 0.0),
        v(2.0, 2.0, 0.0),
        v(0.0, 2.0, 0.0),
    ];
    let half = clip_to_half_space(&square, v(1.0, 0.0, 0.0), 1.0);
    assert_eq!(half.len(), 4);
    assert!(half.iter().all(|p| p.x >= 1.0 - 1e-12));

    assert_eq!(
        try_meet((v(1.0, 0.0, 0.0), 2.0), (v(0.0, 1.0, 0.0), 3.0)),
        Some(v(2.0, 3.0, 0.0))
    );
    assert_eq!(
        try_meet((v(1.0, 0.0, 0.0), 2.0), (v(1.0, 0.0, 0.0), 3.0)),
        None
    );

    // Four corners of a square in z = 0 come back once each, round the centroid.
    let ring = round_the_centroid(
        vec![
            v(1.0, 1.0, 0.0),
            v(-1.0, -1.0, 0.0),
            v(1.0, -1.0, 0.0),
            v(-1.0, 1.0, 0.0),
            v(1.0, 1.0, 0.0),
        ],
        v(0.0, 0.0, 1.0),
        1e-9,
    );
    assert_eq!(ring.len(), 4);

    let obstruction = Obstruction::new(
        v(0.0, 0.0, 1.0),
        10.0,
        v(0.0, 0.0, 10.0),
        v(1.0, 0.0, 0.0),
        20.0,
        v(0.0, 1.0, 0.0),
        2.0,
        1e-6,
        -1,
        0.0,
    );
    assert!(bevel_shadow_clip::standing_on(&obstruction) >= 1e-6);
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let graph = MeshEdgeGraph::create(&cube);
    // A seed face off the mesh stands on nothing, so no column and no body.
    let (columns, bodies) =
        bevel_shadow_clip::build(&cube, &graph, &[obstruction], true, 1.0).expect("clipper");
    assert!(columns.is_empty());
    assert_eq!(bodies, 0);
}
