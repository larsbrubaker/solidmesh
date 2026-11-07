//! One part of the pierced face tests: THE CORNER'S SLID END STANDS JUST OUTSIDE ITS OWN WALL - the
//! reading of the one join of the pierced face's smooth side that was refused with no junction in
//! it, and of the walk's setting-out that lays it. The class's helpers are
//! [`crate::section_pierced_face_readings`].

use std::collections::HashSet;

use geometry_processing::bevel::sections::{
    section_fan_walk, section_smooth_side, section_surface_walk_join, SectionKind, SectionSide,
};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use crate::inner_face_test_mesh;
use crate::section_pierced_face_junction_run_tests::the_run;
use crate::section_pierced_face_readings::RADIUS;

/// MEASURED. The pick's chain is CLOSED and runs the cube's top outline through
/// the bore's mouth; stations 42 and 43 are its CORNER stations at the two top corners of the cube's
/// -Y wall, each slid one radius down the vertical crease there. Their two boundary points are
/// 20 mm apart ACROSS that wall, which is an ordinary join to lay.
///
/// What refused it is that station 42's slid end stands 2.8e-6 OUTSIDE both walls it was built
/// from: a point drawn from float vertices at the corner, within the mesh's own lattice of the
/// crease it is placed on but not ON it. The walk read that crease as a crossing and refused
/// CreaseReached having travelled 1.99e-6 of its 20 mm. The walk now sets out from the FOOT on the
/// edge it stands on.
#[test]
fn the_corners_slid_end_sets_out_from_the_crease_it_stands_on() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let feature_edges: HashSet<i32> = features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();
    let is_a_feature_edge = |edge: i32| feature_edges.contains(&edge);
    let (run, mut pool) = the_run(&part, 2.0 * RADIUS, inner_face_test_mesh::SEGMENTS);

    let smooth_side_is_left = !run
        .junctions
        .values()
        .next()
        .expect("two junctions")
        .flat_on_the_left;

    // The two stations are the cube's own top corners, not ends: this chain is CLOSED.
    assert!(run.is_closed);
    assert_eq!(run.sections[42].kind, SectionKind::Corner);
    assert_eq!(run.sections[43].kind, SectionKind::Corner);
    assert_eq!(run.sections[42].left_kind, SectionSide::Slide);
    assert_eq!(run.sections[43].left_kind, SectionSide::Slide);
    assert_eq!(run.sections[42].chain_vertex, 70);
    assert_eq!(run.sections[43].chain_vertex, 163);

    let (cut, cut_the_smooth_side) = section_smooth_side::try_cut_the_smooth_side(
        &part,
        &graph,
        &edge_at_pair,
        &mut pool,
        &run,
        smooth_side_is_left,
        Some(&is_a_feature_edge),
    );

    assert_eq!(cut.station_points[42].len(), 1);
    assert_eq!(cut.station_points[43].len(), 1);
    let from = cut.station_points[42][0];
    let to = cut.station_points[43][0];
    let end_a = pool.position_of(from);
    let end_b = pool.position_of(to);

    // Each is placed on the vertical CREASE at its corner of the -Y wall, one radius below the top.
    assert_eq!(cut.places[&from].edge, 186);
    assert_eq!(cut.places[&to].edge, 479);
    assert!(feature_edges.contains(&186));
    assert!(feature_edges.contains(&479));

    // THE CAUSE. The far end lands exactly on its crease; the near one stands 2.8e-6 off its own,
    // and outside the part in BOTH walls that meet there - the corner's own float rounding.
    assert!(off_the_line(&part, &graph, 479, end_b).abs() <= 1e-12);
    assert!((off_the_line(&part, &graph, 186, end_a) - 2.82e-6).abs() <= 1e-8);
    assert!(end_a.x < -19.0, "outside the -X wall");
    assert!(end_a.y > -29.0, "and outside the -Y wall");

    // The -Y wall's two triangles are the one pair the join can be laid between, and it lays: the
    // walk sets out from the foot on crease 186 rather than reading that crease as a crossing.
    let crossings = section_surface_walk_join::try_join(
        &part,
        &graph,
        &edge_at_pair,
        end_a,
        57,
        end_b,
        162,
        Some(&is_a_feature_edge),
    )
    .unwrap_or_else(|why| panic!("{why:?}"));

    assert_eq!(
        crossings.len(),
        1,
        "the wall's own diagonal is all the line crosses"
    );

    assert!(
        cut_the_smooth_side.is_ok(),
        "{:?}",
        cut_the_smooth_side.err()
    );
}

/// How far a point stands off the line of a mesh edge.
fn off_the_line(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32, at: Vector3) -> f64 {
    let from = Vector3::from_vector3_float(mesh.vertices[graph.get_vertex0(edge) as usize]);
    let along = Vector3::from_vector3_float(mesh.vertices[graph.get_vertex1(edge) as usize]) - from;
    let s = (at - from).dot(along) / along.length_squared();

    (from + (along * s) - at).length()
}
