//! Rust-only tests for the bevel-core package R1.P08's methods that no other test covers on its
//! own: a run's absorbed split vertices (`SectionAbsorbedSplits`), the section builder's starting
//! readings (`SectionBuilder`), the room's region and crease readings (`SectionRoom`), the smooth
//! side's crease walk (`SectionSmoothSide`) and the builder's walked-side readings, on a cube. The
//! surface walk's own tests are in `rust_only_bevel_r1p08_walk_tests.rs`. Each pins the rule on a
//! hand-built case.

use indexmap::IndexMap;
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::MeshEdgeGraph;
use vector_math::Vector3;

use geometry_processing::bevel::sections::section_split_blend::SplitPlacement;
use geometry_processing::bevel::sections::{
    section_builder, section_builder_walked_side, section_fan_walk, section_room,
    section_smooth_side_boundary, BevelSection, JunctionSection, SectionAbsorbedSplits,
    SectionPool,
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

/// The cube's edges at each vertex, as the room keeps them.
fn edges_at_vertex(graph: &MeshEdgeGraph) -> IndexMap<i32, Vec<i32>> {
    let mut at: IndexMap<i32, Vec<i32>> = IndexMap::new();
    for edge in 0..graph.edge_count() {
        for vertex in [graph.get_vertex0(edge), graph.get_vertex1(edge)] {
            at.entry(vertex).or_default().push(edge);
        }
    }
    at
}

/// A crease runs as far as the edge it starts on where the cube's corner turns it two ways at once,
/// and the slide's chord walk finds the chord a landing lies on, or another crease where it runs
/// past the end.
#[test]
fn rust_only_a_crease_runs_to_the_corner_and_its_chord_is_walked_for() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let features = FeatureEdgeSet::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let is_a_feature = section_builder::the_feature_test(&features);

    let fold = features.edges()[0].mesh_edge_index;
    let (start, end) = (graph.get_vertex0(fold), graph.get_vertex1(fold));
    let station = Vector3::from_vector3_float(cube.vertices[start as usize]);
    let far = Vector3::from_vector3_float(cube.vertices[end as usize]);
    let along = (far - station).get_normal();

    let at = edges_at_vertex(&graph);
    for feature_test in [None, Some(is_a_feature.as_ref())] {
        let reached = section_room::length_of_the_crease(
            &cube,
            &graph,
            &at,
            fold,
            end,
            station,
            along,
            1e-6,
            feature_test,
        );
        assert!((reached - 10.0).abs() < 1e-9, "reached {reached}");
    }

    let creases = section_smooth_side_boundary::the_creases_at_each_vertex(
        &graph,
        &pairs,
        Some(is_a_feature.as_ref()),
    );
    assert_eq!(creases[&start].len(), 3);
    assert!(
        section_smooth_side_boundary::the_creases_at_each_vertex(&graph, &pairs, None).is_empty()
    );

    let half_way = station + (far - station) * 0.5;
    assert_eq!(
        section_smooth_side_boundary::try_the_crease_chord_under(
            &cube,
            &graph,
            &pairs,
            Some(is_a_feature.as_ref()),
            fold,
            start,
            half_way
        ),
        Ok(fold)
    );
    let (chord, off) = section_smooth_side_boundary::try_the_crease_chord_down(
        &cube, &graph, &creases, fold, start, half_way,
    )
    .expect("the first chord holds it");
    assert_eq!(chord, fold);
    assert!(off < 1e-9);

    // Past the corner nothing carries on straight, so the landing is read off the other creases.
    let past = station + (far - station) * 1.5;
    let others: Vec<i32> = creases[&start]
        .iter()
        .copied()
        .filter(|&c| c != fold)
        .collect();
    assert_eq!(
        section_smooth_side_boundary::try_the_crease_chord_under(
            &cube,
            &graph,
            &pairs,
            Some(is_a_feature.as_ref()),
            fold,
            start,
            past
        ),
        Ok(*others.iter().min().unwrap())
    );
}

/// The walked side's small readings on a cube: the face across an edge, an edge's corners, the
/// face's edge at a corner, the corner under a point, and a point placed along a join.
#[test]
fn rust_only_the_walked_side_names_its_places() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);

    let face = &cube.faces[0];
    let (a, b) = (face.v0, face.v1);
    let edge = pairs[&section_room::pair_key(a, b)];
    let across = section_builder_walked_side::the_other_face(&graph, edge, 0);
    assert_ne!(across, 0);
    assert!(section_builder_walked_side::is_a_corner_of(&graph, edge, a));
    assert!(!section_builder_walked_side::is_a_corner_of(
        &graph, edge, face.v2
    ));
    assert_eq!(
        section_builder_walked_side::the_face_edge_at(&cube, &pairs, 0, a),
        edge
    );

    let at_a = Vector3::from_vector3_float(cube.vertices[a as usize]);
    let at_b = Vector3::from_vector3_float(cube.vertices[b as usize]);
    assert_eq!(
        section_builder_walked_side::try_the_corner_under(
            &cube,
            0,
            at_a + Vector3::new(1e-9, 0.0, 0.0),
            1e-6
        ),
        Some(a)
    );
    assert_eq!(
        section_builder_walked_side::try_the_corner_under(&cube, 0, (at_a + at_b) / 2.0, 1e-6),
        None
    );

    let middle = section_builder_walked_side::try_along_the_join(
        &cube,
        &graph,
        &pairs,
        at_a,
        0,
        at_b,
        &[],
        0.5,
        1e-6,
    )
    .expect("a join with length");
    assert!((middle.point - (at_a + at_b) / 2.0).length() < 1e-9);
    assert_eq!((middle.face, middle.fold_edge, middle.corner), (0, -1, -1));

    let start = section_builder_walked_side::try_along_the_join(
        &cube,
        &graph,
        &pairs,
        at_a,
        0,
        at_b,
        &[],
        0.0,
        1e-6,
    )
    .expect("a join with length");
    assert_eq!((start.corner, start.fold_edge), (a, edge));
    assert_eq!(
        start.along,
        if graph.get_vertex0(edge) == a {
            0.0
        } else {
            1.0
        }
    );
    assert!(section_builder_walked_side::try_along_the_join(
        &cube,
        &graph,
        &pairs,
        at_a,
        0,
        at_a,
        &[],
        0.5,
        1e-6
    )
    .is_none());
}

/// A neighbour's end: a walked side's own point and walk face; a junction's wall side read off the
/// wall end on the join's own edge where that end was carried onto a facet, and the section's own
/// point where it was not; nothing to set out from where the side was neither walked nor slid.
#[test]
fn rust_only_the_neighbours_end_reads_the_junction_wall() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let mut pool = SectionPool::new(&cube.vertices);
    let own = pool.add(Vector3::new(1.0, 2.0, 3.0)).unwrap();
    let carried = pool.add(Vector3::new(4.0, 5.0, 6.0)).unwrap();

    let mut walked = section_at(10, own, own);
    walked.left_walk_face = 3;
    let plain = section_at(11, own, own);
    let sections = vec![walked, plain.clone()];

    let (point, faces) = section_builder_walked_side::the_neighbours_end(
        &cube, &graph, &pairs, &pool, &sections, None, 0, true, true, None,
    );
    assert_eq!((point, faces), (Vector3::new(1.0, 2.0, 3.0), vec![3]));

    let junction = JunctionSection {
        chain_vertex: 11,
        station: Vector3::ZERO,
        through: Vector3::ZERO,
        normal: Vector3::ZERO,
        samples: vec![carried, own],
        carried_onto: vec![7, -1],
        arriving_wall_end: 1,
        leaving_wall_end: 0,
        fold_point: -1,
        arriving_wall: Vector3::ZERO,
        leaving_wall: Vector3::ZERO,
        flat_on_the_left: false,
        setback_arriving: 0.0,
        setback_leaving: 0.0,
        arriving_section: std::rc::Rc::new(plain.clone()),
        leaving_section: std::rc::Rc::new(plain),
    };
    let mut junctions = IndexMap::new();
    junctions.insert(1, junction);

    // The join leaves the junction: its LEAVING wall end, carried onto facet 7.
    let (point, faces) = section_builder_walked_side::the_neighbours_end(
        &cube,
        &graph,
        &pairs,
        &pool,
        &sections,
        Some(&junctions),
        1,
        true,
        true,
        None,
    );
    assert_eq!((point, faces), (Vector3::new(4.0, 5.0, 6.0), vec![7]));

    // Arriving: that wall end was carried nowhere, so the section's own point - neither walked nor
    // slid, so no face to set out from.
    let (point, faces) = section_builder_walked_side::the_neighbours_end(
        &cube,
        &graph,
        &pairs,
        &pool,
        &sections,
        Some(&junctions),
        1,
        false,
        true,
        None,
    );
    assert_eq!(
        (point, faces),
        (Vector3::new(1.0, 2.0, 3.0), Vec::<i32>::new())
    );

    // The junction's FLAT side is read the ordinary way.
    let (_, faces) = section_builder_walked_side::the_neighbours_end(
        &cube,
        &graph,
        &pairs,
        &pool,
        &sections,
        Some(&junctions),
        1,
        true,
        false,
        None,
    );
    assert!(faces.is_empty());
}
