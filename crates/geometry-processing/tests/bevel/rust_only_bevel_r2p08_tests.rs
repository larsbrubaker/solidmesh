//! Rust-only checks of the bevel core's R2.P08 package: the growing runs' live seeds (in the R2.P05
//! crease tests), and the pieces the whole-build tests reach only through the rest of the build.

use solidmesh_mesh::solids::platonic_solids::create_cube;

use solidmesh_math::Vector3;

#[test]
fn rust_only_a_mixed_corner_is_refused_off_two_plans_of_one_sense() {
    use solidmesh::bevel::surface::SurfaceMixedCorner;
    use solidmesh::bevel::BevelAdvisoryCode;

    // Two convex plans: no concave bead to roll round.
    let convex = crate::rust_only_bevel_r1p06_tests::square_edge_plan(1.0, true);
    let refused = SurfaceMixedCorner::try_read(&convex, &convex, Vector3::ZERO, 1.0, 1e-6)
        .expect_err("two convex edges are no mixed corner");
    assert_eq!(refused.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);

    // A chamfer carries no centre line for the ring to touch the bead along.
    let chamfer = crate::rust_only_bevel_r1p06_tests::square_edge_plan(1.0, false);
    assert!(SurfaceMixedCorner::try_read_carried(
        &chamfer,
        &chamfer,
        None,
        &chamfer,
        &chamfer,
        Vector3::ZERO,
        1.0,
        1e-6
    )
    .is_err());
}

#[test]
fn rust_only_a_side_that_is_not_an_offset_or_reaches_nowhere_is_not_walked() {
    use solidmesh::bevel::sections::bevel_section::{BevelSection, SectionSide};
    use solidmesh::bevel::sections::section_builder_walked_side::try_lay_one_side;
    use solidmesh::bevel::sections::section_fan_walk::EdgeAtPair;
    use solidmesh::bevel::sections::SectionPool;
    use solidmesh::topology::{ChainEdge, FeatureEdge};

    let cube = create_cube(2.0, 2.0, 2.0);
    let graph = cube.get_mesh_edge_graph();
    let mut pool = SectionPool::new(&cube.vertices);
    let station = Vector3::from_vector3_float(cube.vertices[0]);
    let off = pool.add(station + Vector3::new(0.5, 0.0, 0.0)).unwrap();
    let section = BevelSection::new(
        0,
        station,
        Vector3::new(0.0, 0.0, 1.0),
        off,
        0,
        vec![off, 0],
        station,
        station,
        SectionSide::Slide,
        SectionSide::Offset,
        -1,
        -1,
        0.5,
    )
    .unwrap();
    let edge = ChainEdge::new(0, FeatureEdge::default(), false);

    for on_the_left in [true, false] {
        let walked = try_lay_one_side(
            &section,
            &edge,
            &edge,
            on_the_left,
            &cube,
            &graph,
            &EdgeAtPair::new(),
            &mut pool,
            None,
        )
        .unwrap();
        assert_eq!(
            (walked.boundary, walked.face, walked.edge, walked.vertex),
            (-1, -1, -1, -1)
        );
    }
}
