//! Rust-only checks of the bevel core's R2.P08 package: split chords put back together, the
//! growing runs' live seeds (in the R2.P05 crease tests), and the pieces whose C# test classes
//! wait on the rest of the package.

use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::Mesh;

use geometry_processing::bevel::bevel_split_chords;
use vector_math::Vector3;

fn tent(m: Vector3) -> Mesh {
    // A crease A-B with a vertex M standing on it, each side's flat face fanned through M.
    let (a, b) = (Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0));
    let (p, q) = (Vector3::new(1.0, 1.0, 0.0), Vector3::new(1.0, 0.0, -1.0));
    let mut mesh = Mesh::new();
    mesh.create_face(&[a, m, p]);
    mesh.create_face(&[m, b, p]);
    mesh.create_face(&[m, a, q]);
    mesh.create_face(&[b, m, q]);
    mesh.clean_and_merge();
    mesh
}

#[test]
fn rust_only_a_split_chord_is_put_back_together() {
    let mut split = tent(Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(split.faces.len(), 4);
    assert_eq!(bevel_split_chords::removed_from(&mut split), 1);
    assert_eq!(split.faces.len(), 2, "each side is one triangle again");

    // A vertex that turns the crease is no split: nothing is removed.
    let mut bent = tent(Vector3::new(1.0, 0.3, 0.3));
    assert_eq!(bevel_split_chords::removed_from(&mut bent), 0);
    assert_eq!(bent.faces.len(), 4);

    // A cube has no split chord.
    let mut cube = create_cube(2.0, 2.0, 2.0);
    let before = cube.faces.len();
    assert_eq!(bevel_split_chords::removed_from(&mut cube), 0);
    assert_eq!(cube.faces.len(), before);
}

#[test]
fn rust_only_a_mixed_corner_is_refused_off_two_plans_of_one_sense() {
    use geometry_processing::bevel::surface::SurfaceMixedCorner;
    use geometry_processing::bevel::BevelAdvisoryCode;

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
    use geometry_processing::bevel::sections::bevel_section::{BevelSection, SectionSide};
    use geometry_processing::bevel::sections::section_builder_walked_side::try_lay_one_side;
    use geometry_processing::bevel::sections::section_fan_walk::EdgeAtPair;
    use geometry_processing::bevel::sections::SectionPool;
    use geometry_processing::topology::{ChainEdge, FeatureEdge};

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

#[test]
fn rust_only_a_cube_edges_run_is_capped_on_the_faces_beyond_its_ends() {
    use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
    use geometry_processing::bevel::BevelChainEnds;
    use geometry_processing::topology::{ChainEdge, FeatureEdgeSet};

    let cube = create_cube(2.0, 2.0, 2.0);
    let features = FeatureEdgeSet::create(&cube);
    let edge = features.edges()[0];
    let run = [ChainEdge::new(0, edge, false)];
    let mut warnings = Vec::new();

    let ends = BevelChainEnds::resolve(
        &run,
        false,
        0.5,
        Some(&features),
        Some(&[edge]),
        Some(&mut warnings),
        ResolveOptions::default(),
    );

    // Each end of a lone cube edge meets the cube face square to it: capped on that face's plane,
    // facing the way the run leaves.
    let direction = run[0].direction();
    assert!(ends.start.is_capped() && ends.end.is_capped());
    assert!(
        (ends.start.plane_normal + direction).length() < 1e-9,
        "{:?}",
        ends.start.plane_normal
    );
    assert!(
        (ends.end.plane_normal - direction).length() < 1e-9,
        "{:?}",
        ends.end.plane_normal
    );
    assert!(ends.end.plane_distance >= direction.dot(run[0].to()) - 1e-9);
    assert_eq!((ends.start_pair_corner, ends.end_pair_corner), (None, None));
    assert_eq!(
        (ends.start_overshoot_corner, ends.end_overshoot_corner),
        (None, None)
    );
    assert!(warnings.is_empty(), "{warnings:?}");

    // A closed run, or no features, overshoots freely.
    assert_eq!(
        BevelChainEnds::resolve(
            &run,
            true,
            0.5,
            Some(&features),
            None,
            None,
            ResolveOptions::default()
        ),
        BevelChainEnds::overshoot()
    );
    assert_eq!(
        BevelChainEnds::resolve(
            &run,
            false,
            0.5,
            None,
            None,
            None,
            ResolveOptions::default()
        ),
        BevelChainEnds::overshoot()
    );
}
