//! Rust-only checks of the bevel core's R1.P09 package pieces no other test reaches alone: the
//! interrupted crease's leaning and the valley-fold mitre.

use solidmesh::bevel::bevel_interrupted_crease::{crosses_the_run, Leaning};
use solidmesh_math::Vector3;

#[test]
fn rust_only_an_end_closes_over_the_run_only_where_nothing_falls_away() {
    assert!(crosses_the_run(0.5) && !crosses_the_run(0.1));

    let mut closing = Leaning::default();
    closing.saw(-0.9);
    assert!(closing.closes_over_the_run(true));
    assert!(!closing.closes_over_the_run(false));

    closing.saw(0.9);
    assert!(
        !closing.closes_over_the_run(true),
        "a face falling away opens the end"
    );

    let mut grazing = Leaning::default();
    grazing.saw(-0.05);
    assert!(
        !grazing.closes_over_the_run(true),
        "a face along the run does not cross it"
    );
}

#[test]
fn rust_only_a_joint_mitres_a_square_turn_and_refuses_a_doubling_back() {
    use solidmesh::bevel::bevel_chain_sweep_mesh::SweptSegment;
    use solidmesh::bevel::bevel_chain_sweep_rings::try_build_joint;
    use solidmesh::bevel::BevelAdvisoryCode;
    use solidmesh_math::Vector2;

    let up = Vector3::new(0.0, 0.0, 1.0);
    let segment = |from: Vector3, to: Vector3| {
        let direction = (to - from).get_normal();
        SweptSegment::new(
            from,
            to,
            direction,
            up,
            direction.cross(up),
            vec![Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)],
            1.0,
        )
    };
    let corner = Vector3::new(10.0, 0.0, 0.0);
    let mut before = segment(Vector3::new(0.0, 0.0, 0.0), corner);
    let mut after = segment(corner, Vector3::new(10.0, 10.0, 0.0));

    let ring = try_build_joint(&mut before, &mut after, 2).unwrap();

    // The profile's origin stands on the corner; its other point sits on the mitre, stretched
    // by 1 / cos(45) along the turn, so sqrt(2) from the corner.
    assert!((ring[0] - corner).length() < 1e-12);
    assert!(((ring[1] - corner).length() - 2f64.sqrt()).abs() < 1e-12);
    assert!(before.end_slide.is_some() && after.start_slide.is_some());

    let mut back = segment(corner, Vector3::new(0.0, 0.0, 0.0));
    let doubled = try_build_joint(&mut before, &mut back, 2).unwrap_err();
    assert_eq!(doubled.code(), BevelAdvisoryCode::ChainDoublesBack);
}
