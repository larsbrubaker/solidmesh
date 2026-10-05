//! Rust-only tests for the bevel-core package R1.P11's junction and corner fold readings, split from
//! `rust_only_bevel_r1p11_tests.rs` by responsibility: one canal at a junction (`SectionJunction.Canal`),
//! the junction section's samples (`JunctionSection`), the junction's line and facet readings, a side's
//! boundary sliding down a crease (`SectionBuilder.TryBoundary`), the corner's smooth-fold reading
//! (`SectionCorner.IsASmoothFold`), and a box's top corner built as a mitre (`SectionCorner.TryBuild`,
//! its fold readings) and refused as a junction (`SectionBuilder.TryJunction`). Each pins the C# rule
//! on a hand-built case.

use std::rc::Rc;

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::MeshEdgeGraph;
use vector_math::Vector3;

use geometry_processing::bevel::sections::section_corner::CornerRefusal;
use geometry_processing::bevel::sections::section_junction::Canal;
use geometry_processing::bevel::sections::section_junction_build::{self, JunctionRefusal};
use geometry_processing::bevel::sections::section_junction_canal as junction;
use geometry_processing::bevel::sections::{
    section_builder, section_corner, BevelSection, JunctionPiece, JunctionSection, SectionKind,
    SectionPool, SectionSide,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{ChainEdge, FeatureEdge, FeatureEdgeSet};

/// A square edge along +x: its left face the top (+z), its right the front (-y).
fn square_edge() -> ChainEdge {
    ChainEdge::new(
        0,
        FeatureEdge {
            start: Vector3::ZERO,
            end: Vector3::new(10.0, 0.0, 0.0),
            direction: Vector3::new(1.0, 0.0, 0.0),
            normal0: Vector3::new(0.0, 0.0, 1.0),
            normal1: Vector3::new(0.0, -1.0, 0.0),
            ..Default::default()
        },
        false,
    )
}

/// A canal on a square edge: its axis a radius in from both faces, its arc from the flat face
/// (v = 0) round to the wall (v = 1), and a point of it read back to its parameter.
#[test]
fn rust_only_a_canal_reads_its_arc() {
    let edge = square_edge();
    let mut canal = Canal::new(&edge, Vector3::ZERO, 2.0, true, 1e-9);
    assert!((canal.axis - Vector3::new(0.0, 2.0, -2.0)).length() < 1e-12);
    assert!((canal.wedge - std::f64::consts::FRAC_PI_2).abs() < 1e-12);

    let x = Vector3::new(1.0, 0.0, 0.0);
    let flat_end = canal.point_at(0.0, Vector3::ZERO, x);
    let wall_end = canal.point_at(1.0, Vector3::ZERO, x);
    assert!((flat_end - Vector3::new(0.0, 2.0, 0.0)).length() < 1e-12);
    assert!((wall_end - Vector3::new(0.0, 0.0, -2.0)).length() < 1e-12);
    assert_eq!(canal.lean, 1.0);

    let half = canal.point_at(0.5, Vector3::ZERO, x);
    assert!((canal.distance_from_the_axis(half) - 2.0).abs() < 1e-12);
    assert!((canal.try_parameter_of(half).unwrap() - 0.5).abs() < 1e-12);
    assert!(canal.try_parameter_of(Vector3::ZERO).is_none());
}

/// The junction's small readings: a lattice for its points, two lines' meet, a flat side read off two
/// edges, and a strip's section at the junction.
#[test]
fn rust_only_the_junction_reads_lines_and_sides() {
    // Never under a billionth; otherwise four float steps at the largest coordinate (2^-24 at 0.5).
    assert_eq!(junction::lattice_of(&[Vector3::ZERO]), 1e-9);
    assert_eq!(
        junction::lattice_of(&[Vector3::new(0.5, -0.25, 0.0)]),
        4.0 * 2f64.powi(-24)
    );

    let (meet, skew) = junction::meet_of_lines(
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.0, 1.0, 0.0),
    );
    assert!(meet.length() < 1e-12 && (skew - 2.0).abs() < 1e-12);
    let x = Vector3::new(1.0, 0.0, 0.0);
    assert_eq!(
        junction::meet_of_lines(Vector3::ZERO, x, Vector3::ZERO, x).1,
        f64::MAX
    );

    let edge = square_edge();
    assert_eq!(junction::try_read_the_flat_side(&edge, &edge), None);

    let section = junction::the_strips_section(
        3,
        Vector3::ZERO,
        x,
        &[10, 11, 12, 13],
        2,
        false,
        Vector3::ZERO,
        1.5,
    )
    .expect("a profile of the samples");
    assert_eq!(section.profile, vec![12, 11, 10]);
    assert_eq!(section.kind, SectionKind::Junction);
    assert_eq!(
        (section.left_kind, section.right_kind),
        (SectionSide::Offset, SectionSide::Offset)
    );
}

/// A junction section's own samples: wall ends, the patch fanned from the fold point between them,
/// and the directed edges its pieces lay - the strips' shared stretch cancelling.
#[test]
fn rust_only_a_junction_section_lays_its_patch() {
    let plain = Rc::new(
        BevelSection::new(
            0,
            Vector3::ZERO,
            Vector3::ZERO,
            0,
            1,
            vec![0, 1],
            Vector3::ZERO,
            Vector3::ZERO,
            SectionSide::Offset,
            SectionSide::Offset,
            -1,
            -1,
            1.0,
        )
        .unwrap(),
    );
    let junction = JunctionSection::new(
        5,
        Vector3::ZERO,
        Vector3::ZERO,
        Vector3::ZERO,
        vec![20, 21, 22, 23],
        1,
        3,
        99,
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        true,
        1.0,
        1.0,
        plain.clone(),
        plain,
        vec![-1; 4],
    );
    assert_eq!((junction.near_wall_end(), junction.far_wall_end()), (1, 3));
    assert!(junction.long_strip_runs_up_the_samples());
    assert!(junction.lies_in_the_arriving_wall(Vector3::new(0.0, 1.0, 0.0)));
    assert_eq!(junction.patch_triangles(), vec![[99, 22, 21], [99, 23, 22]]);

    let edges = junction.directed_edges();
    assert_eq!(edges.len(), 3 + 1 + 6 + 2);
    assert_eq!(edges[0], (20, 21, JunctionPiece::LeavingStrip));
    assert_eq!(edges[3], (21, 20, JunctionPiece::ArrivingStrip));
    assert_eq!(edges[edges.len() - 2], (99, 21, JunctionPiece::NearWallRim));
    assert_eq!(edges[edges.len() - 1], (23, 99, JunctionPiece::FarWallRim));
}

/// At a cube corner, one side of the fan between two of its edges crosses the third edge: a boundary
/// reaching out along that edge SLIDES down it by the setback; the cube's edges are no smooth folds.
#[test]
fn rust_only_a_boundary_slides_down_the_crease_on_its_side() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let features = FeatureEdgeSet::create(&cube);

    let corner = features.edges()[0].start_vertex;
    let at = |v: i32| Vector3::from_vector3_float(cube.vertices[v as usize]);
    let station = at(corner);
    let others: Vec<i32> = features
        .edges_at_vertex(corner)
        .iter()
        .map(|&e| {
            let edge = features.edges()[e as usize];
            if edge.start_vertex == corner {
                edge.end_vertex
            } else {
                edge.start_vertex
            }
        })
        .collect();
    assert_eq!(others.len(), 3, "three cube edges meet at a corner");

    let arriving = section_builder::try_feature_edge_between(&features, others[0], corner).unwrap();
    let leaving = section_builder::try_feature_edge_between(&features, corner, others[1]).unwrap();
    let third =
        pairs[&geometry_processing::bevel::sections::section_room::pair_key(corner, others[2])];
    let away = (at(others[2]) - station).get_normal();
    assert!(!section_corner::is_a_smooth_fold(&cube, &graph, third));

    let mut slid = None;
    for on_the_left in [true, false] {
        if let Ok(side) = section_builder::try_boundary(
            corner,
            station,
            &arriving,
            &leaving,
            on_the_left,
            Vector3::ZERO,
            away,
            1.5,
            1.0,
            station,
            Vector3::ZERO,
            &cube,
            &graph,
            &pairs,
            None,
        ) {
            if side.kind == SectionSide::Slide {
                slid = Some(side);
            }
        }
    }
    let slid = slid.expect("the side whose fan crosses the third edge slides down it");
    assert_eq!(slid.slide_edge, third);
    assert!((slid.at - (station + away * 1.5)).length() < 1e-9);
}

/// A box's top corner, between two top-rim edges, as the corner rule draws it: the top is the
/// shared flat side, so the wall-side fan holds the ONE vertical fold (`try_the_fold`) and the top's
/// fan holds none (`try_the_shared_side`, an offset); the section is the arriving edge's profile slid
/// onto the bisector plane. The same corner is no junction - its two wedges are equal - so the
/// junction rule refuses it by name.
#[test]
fn rust_only_a_box_top_corner_is_built_as_a_mitre() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let features = FeatureEdgeSet::create(&cube);
    let at = |v: i32| Vector3::from_vector3_float(cube.vertices[v as usize]);
    let top = |v: i32| at(v).z > 4.999;

    let corner = (0..cube.vertices.len() as i32)
        .find(|&v| top(v))
        .expect("the box has a top");
    let station = at(corner);
    let mut rim = Vec::new();
    let mut below = -1;
    for &e in features.edges_at_vertex(corner) {
        let edge = features.edges()[e as usize];
        let other = if edge.start_vertex == corner {
            edge.end_vertex
        } else {
            edge.start_vertex
        };
        if top(other) {
            rim.push(other);
        } else {
            below = other;
        }
    }
    assert_eq!(
        rim.len(),
        2,
        "two top-rim edges and one wall fold meet at a top corner"
    );

    let arriving = section_builder::try_feature_edge_between(&features, rim[0], corner).unwrap();
    let leaving = section_builder::try_feature_edge_between(&features, corner, rim[1]).unwrap();
    let (flat_on_the_left, cosine, _) =
        section_corner::try_read_the_sides(&arriving, &leaving).expect("the top is shared");
    let wall_fold =
        pairs[&geometry_processing::bevel::sections::section_room::pair_key(corner, below)];
    let (r, segments) = (1.5, 4);

    // The two fold readings on the real corner.
    assert_eq!(
        section_corner::try_the_fold(
            corner,
            &arriving,
            &leaving,
            !flat_on_the_left,
            cosine,
            r,
            &cube,
            &graph,
            &pairs,
            station,
        ),
        Ok(wall_fold)
    );
    assert_eq!(
        section_corner::try_the_shared_side(
            corner,
            &arriving,
            &leaving,
            flat_on_the_left,
            &cube,
            &graph,
            &pairs,
            station,
        ),
        Ok(-1)
    );
    // Folding the wall the wrong way finds the top's fan, which holds no crease to ride.
    assert!(section_corner::try_the_fold(
        corner,
        &arriving,
        &leaving,
        flat_on_the_left,
        cosine,
        r,
        &cube,
        &graph,
        &pairs,
        station,
    )
    .is_err());

    assert!(!section_junction_build::is_a_junction_station(
        &cube,
        &graph,
        &arriving,
        &leaving,
        corner,
        None,
        Some(&pairs),
    ));

    let mut pool = SectionPool::new(&cube.vertices);
    let section = section_corner::try_build(
        &arriving, &leaving, &cube, &graph, &pairs, &mut pool, r, segments, None,
    )
    .expect("a box corner is a mitre");
    assert_eq!(section.kind, SectionKind::Corner);
    assert_eq!(section.chain_vertex, corner);
    assert_eq!(section.profile.len(), segments as usize + 1);
    assert_eq!(section.left_boundary, section.profile[0]);
    assert_eq!(section.right_boundary, section.profile[segments as usize]);
    assert!(
        (section.setback - r).abs() < 1e-12,
        "a square wedge sets back one radius"
    );
    let bisector = (arriving.direction() + leaving.direction()).get_normal();
    assert!((section.tangent - bisector).length() < 1e-12);
    for &p in &section.profile {
        assert!((pool.position_of(p) - station).dot(bisector).abs() < 1e-9);
    }
    let (shared, wall) = if flat_on_the_left {
        (
            (section.left_kind, section.left_slide_edge),
            (section.right_kind, section.right_slide_edge),
        )
    } else {
        (
            (section.right_kind, section.right_slide_edge),
            (section.left_kind, section.left_slide_edge),
        )
    };
    assert_eq!(shared, (SectionSide::Offset, -1));
    assert_eq!(wall, (SectionSide::Slide, wall_fold));

    // The junction rule at the same corner: one crease on the wall side, but the leaving canal's wall
    // end is the arriving canal's own (a corner's case), so it is refused by name and mints nothing.
    let minted = pool.count();
    match section_builder::try_junction(
        &arriving, &leaving, corner, station, &cube, &graph, &pairs, &mut pool, r, segments, None,
    ) {
        Err(JunctionRefusal::Refused(advisory)) => {
            assert_eq!(
                advisory.code(),
                BevelAdvisoryCode::SurfaceCornerNotSupported
            )
        }
        other => panic!("a box corner is no junction - instead {other:?}"),
    }
    assert_eq!(pool.count(), minted);

    // A chain doubling back has no corner to bisect, and shares no side the rule can read.
    match section_corner::try_build(
        &arriving, &arriving, &cube, &graph, &pairs, &mut pool, r, segments, None,
    ) {
        Err(CornerRefusal::Refused(advisory)) => {
            assert_eq!(
                advisory.code(),
                BevelAdvisoryCode::SurfaceCornerNotSupported
            )
        }
        other => panic!("no corner to draw - instead {other:?}"),
    }
}
