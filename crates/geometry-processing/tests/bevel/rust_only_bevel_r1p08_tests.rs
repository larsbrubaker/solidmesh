//! Rust-only tests for the bevel-core package R1.P08's methods that no ported C# test class covers
//! yet: a run's absorbed split vertices (`SectionAbsorbedSplits`), the section builder's starting
//! readings (`SectionBuilder`) and the room's region and crease readings (`SectionRoom`), on a cube.
//! Each pins the C# rule on a hand-built case.

use indexmap::IndexMap;
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::MeshEdgeGraph;
use vector_math::Vector3;

use geometry_processing::bevel::sections::section_split_blend::SplitPlacement;
use geometry_processing::bevel::sections::{
    section_builder, section_fan_walk, section_room, BevelSection, SectionAbsorbedSplits,
};
use geometry_processing::topology::FeatureEdgeSet;

/// Six stations, two of them split: the unsplit four are the stations, each gap carries the split
/// points between its ends, and a chain vertex reads back its absorbed point.
#[test]
fn rust_only_split_stations_are_absorbed_into_their_gaps() {
    let sections: Vec<BevelSection> = (0..6)
        .map(|i| section_at(100 + i, 200 + i, 300 + i))
        .collect();
    assert_eq!(sections[0].segments(), 2);

    let unsplit = SplitPlacement {
        before: -1,
        after: -1,
        parameter: 0.0,
    };
    let split = |parameter: f64| SplitPlacement {
        before: 0,
        after: 2,
        parameter,
    };
    let placed = vec![unsplit, split(0.5), unsplit, unsplit, split(0.25), unsplit];
    assert!(placed[1].is_split() && !placed[0].is_split());

    let absorbed = SectionAbsorbedSplits::from(&sections, &placed, true, None, None)
        .expect("two stations are split");
    assert_eq!(absorbed.stations, vec![0, 2, 3, 5]);
    assert_eq!((absorbed.count(), absorbed.gap_count()), (4, 4));
    assert_eq!(absorbed.left_of(0)[0].pool_index, 201);
    assert_eq!(absorbed.right_of(2)[0].parameter, 0.25);
    assert!(absorbed.left_of(1).is_empty());
    assert_eq!(absorbed.point_at(104, false), 304);
    assert_eq!(absorbed.point_at(103, true), -1);

    // Open: one gap fewer. Swallowed on the left: the corner's own point stands in.
    let swallowed = [-1, 77, -1, -1, -1, -1];
    let open = SectionAbsorbedSplits::from(&sections, &placed, false, None, Some(&swallowed))
        .expect("still split");
    assert_eq!(open.gap_count(), 3);
    assert_eq!(open.right_of(0)[0].pool_index, 77);
    assert!(SectionAbsorbedSplits::from(&sections, &[unsplit; 6], true, None, None).is_none());
}

/// On a cube every edge is a feature: the feature test reads the set, the feature edge between two
/// corners is found walked either way, and each side's flat region is its two triangles inside a rim
/// of four creases.
#[test]
fn rust_only_the_cube_reads_as_regions_and_creases() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let features = FeatureEdgeSet::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    assert_eq!(pairs.len() as i32, graph.edge_count());

    let is_a_feature = section_builder::the_feature_test(&features);
    let first = features.edges()[0];
    assert!(is_a_feature(first.mesh_edge_index));

    let forwards =
        section_builder::try_feature_edge_between(&features, first.start_vertex, first.end_vertex)
            .expect("the edge joins its own ends");
    let backwards =
        section_builder::try_feature_edge_between(&features, first.end_vertex, first.start_vertex)
            .expect("walked the other way");
    assert!(!forwards.is_reversed && backwards.is_reversed);

    assert_eq!(
        section_room::pair_key(3, 9),
        section_fan_walk::pair_key(9, 3)
    );

    let mut rim = Vec::new();
    let faces = section_room::faces_of_the_flat_region(
        &cube,
        &graph,
        &pairs,
        0,
        None,
        Some(&mut rim),
        None,
    );
    assert_eq!(faces.len(), 2);
    assert_eq!(rim.len(), 4);
    assert!(section_room::every_face_is_in_the_seeds_plane(
        &cube, &faces, 0
    ));

    let mut by_face = IndexMap::new();
    let region = section_room::region_of_the_face(&cube, &graph, &pairs, &mut by_face, 0, None);
    assert!(region.is_one_plane && region.rim.len() == 4);
    assert_eq!(by_face.len(), 2);
    let mut answers = IndexMap::new();
    assert!(section_room::the_region_is_one_plane(
        &cube,
        &graph,
        &pairs,
        0,
        Some(is_a_feature.as_ref()),
        Some(&mut answers)
    ));
    assert_eq!(answers.len(), 2);

    let crease = rim[0];
    let sides = section_room::the_two_sides(&cube, &graph, crease);
    assert!(sides.2 && section_room::same_two_sides(sides, sides));
    assert!(section_room::is_a_crease(&cube, &graph, crease, None));

    // From the middle of the region's face, a ray reaches its rim at half the cube's side.
    let face = &cube.faces[0];
    let normal = Vector3::from_vector3_float(face.normal);
    let middle_of_the_side = {
        let all: Vec<Vector3> = faces
            .iter()
            .flat_map(|&f| {
                let drawn = &cube.faces[f as usize];
                [drawn.v0, drawn.v1, drawn.v2]
            })
            .map(|v| Vector3::from_vector3_float(cube.vertices[v as usize]))
            .collect();
        all.iter().fold(Vector3::ZERO, |sum, &at| sum + at) / all.len() as f64
    };
    let away = normal.cross(Vector3::new(1.0, 0.0, 0.0));
    let away = if away.length_squared() > 0.0 {
        away.get_normal()
    } else {
        normal.cross(Vector3::new(0.0, 1.0, 0.0)).get_normal()
    };
    let reach = section_room::reach_across_the_face(
        &cube,
        &graph,
        &region,
        0,
        -1,
        middle_of_the_side,
        away,
    );
    assert!((reach - 5.0).abs() < 1e-5, "reach {reach}");
}

/// A section at one chain vertex with its two boundary points and a two-segment profile; nothing
/// else of it is read here.
fn section_at(chain_vertex: i32, left_boundary: i32, right_boundary: i32) -> BevelSection {
    BevelSection {
        kind: Default::default(),
        chain_vertex,
        station: Vector3::ZERO,
        tangent: Vector3::ZERO,
        left_boundary,
        right_boundary,
        profile: vec![0, 1, 2],
        centre_from: Vector3::ZERO,
        centre_to: Vector3::ZERO,
        left_kind: Default::default(),
        right_kind: Default::default(),
        left_slide_edge: -1,
        right_slide_edge: -1,
        left_walk_face: -1,
        right_walk_face: -1,
        left_walk_edge: -1,
        right_walk_edge: -1,
        left_walk_parameter: 0.0,
        right_walk_parameter: 0.0,
        left_walk_vertex: -1,
        right_walk_vertex: -1,
        left_walk_crossed_a_fold: false,
        right_walk_crossed_a_fold: false,
        left_boundary_is_join_placed: false,
        right_boundary_is_join_placed: false,
        left_walk_direction: Vector3::ZERO,
        right_walk_direction: Vector3::ZERO,
        setback: 0.0,
    }
}
