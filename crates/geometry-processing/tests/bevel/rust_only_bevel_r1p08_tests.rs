//! Rust-only tests for the bevel-core package R1.P08's methods that no ported C# test class covers
//! yet: a run's absorbed split vertices (`SectionAbsorbedSplits`), the section builder's starting
//! readings (`SectionBuilder`), the room's region and crease readings (`SectionRoom`), the smooth
//! side's crease walk (`SectionSmoothSide`) and the builder's walked-side readings, on a cube.
//! Each pins the C# rule on a hand-built case.

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

/// The surface walk's small readings on a cube: its grid, one point read twice, feet along an edge,
/// folds (a cube edge is one, a face's diagonal is not), a parameter turned to the graph's way, and a
/// refusal carrying where it stopped.
#[test]
fn rust_only_the_surface_walk_reads_folds_and_feet() {
    use geometry_processing::bevel::sections::section_surface_walk as walk;
    use geometry_processing::bevel::sections::{
        SurfaceWalkCrossing, SurfaceWalkRefusal, SurfaceWalkResult,
    };
    use geometry_processing::bevel::surface::SurfaceVertexPool;

    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);

    assert!(walk::grid_of(&cube) >= SurfaceVertexPool::weld_distance_for(&cube.vertices));
    assert!(walk::is_the_same_point(
        Vector3::ZERO,
        Vector3::new(5e-10, 0.0, 0.0),
        0.5
    ));
    assert!(!walk::is_the_same_point(
        Vector3::ZERO,
        Vector3::new(1e-6, 0.0, 0.0),
        2.0
    ));

    let face = &cube.faces[0];
    let side = pairs[&walk::pair_key(face.v0, face.v1)];
    let diagonal = [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)]
        .iter()
        .map(|&(a, b)| pairs[&walk::pair_key(a, b)])
        .find(|&edge| !walk::is_a_fold(&cube, &graph, edge))
        .expect("a triangle of a cube's side has its diagonal");
    assert!(walk::is_a_fold(&cube, &graph, side));

    let crossing = |edge: i32| SurfaceWalkCrossing::new(edge, 0.5, Vector3::ZERO);
    assert!(walk::crossed_a_fold(
        &cube,
        &graph,
        Some(&[crossing(diagonal), crossing(side)])
    ));
    assert!(!walk::crossed_a_fold(
        &cube,
        &graph,
        Some(&[crossing(diagonal)])
    ));
    assert!(!walk::crossed_a_fold(&cube, &graph, None));

    let v0 = graph.get_vertex0(side);
    let at_v0 = Vector3::from_vector3_float(cube.vertices[v0 as usize]);
    let at_v1 = Vector3::from_vector3_float(cube.vertices[graph.get_vertex1(side) as usize]);
    let quarter = walk::foot_on(&cube, &graph, side, 0.25);
    assert!((quarter - (at_v0 + (at_v1 - at_v0) * 0.25)).length() < 1e-12);
    assert!((walk::along_from(&cube, &graph, side, quarter) - 0.25).abs() < 1e-12);
    assert_eq!(
        walk::along_from(&cube, &graph, side, at_v1 + (at_v1 - at_v0)),
        1.0
    );
    assert_eq!(walk::parameter_on(&graph, side, v0, 0.25), 0.25);
    assert_eq!(
        walk::parameter_on(&graph, side, graph.get_vertex1(side), 0.25),
        0.75
    );

    let fresh = SurfaceWalkResult::new(Vector3::ZERO, 2, Vector3::ZERO, vec![]);
    assert_eq!(
        (fresh.end_edge, fresh.end_vertex, fresh.refusal),
        (-1, -1, SurfaceWalkRefusal::None)
    );
    let stopped = walk::refused(
        Vector3::ZERO,
        2,
        Vector3::ZERO,
        vec![crossing(side)],
        3.5,
        SurfaceWalkRefusal::CreaseReached,
        side,
        0.4,
    );
    assert_eq!(
        (
            stopped.travelled,
            stopped.refusal,
            stopped.end_edge,
            stopped.end_parameter,
            stopped.end_face
        ),
        (3.5, SurfaceWalkRefusal::CreaseReached, side, 0.4, 2)
    );
}

/// The walk's face readings on a cube's top: a line leaves a face across its far edge, a direction is
/// held by the face it points into and carried over the cube's edge onto the side, a vertex's fan
/// holds a direction in one wedge, and turning through a corner finds the face that carries on.
#[test]
fn rust_only_the_surface_walk_reads_faces_and_turns_through_vertices() {
    use geometry_processing::bevel::sections::section_surface_walk::normal_of;
    use geometry_processing::bevel::sections::section_surface_walk_faces as faces;
    use geometry_processing::bevel::sections::section_walk_vertex_turn as turn;

    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let up = Vector3::new(0.0, 0.0, 1.0);
    let top: Vec<i32> = (0..cube.faces.len() as i32)
        .filter(|&f| normal_of(&cube, f).dot(up) > 0.99)
        .collect();
    assert_eq!(top.len(), 2);
    let face = top[0];
    let drawn = &cube.faces[face as usize];
    let corners = [drawn.v0, drawn.v1, drawn.v2];
    let at = |v: i32| Vector3::from_vector3_float(cube.vertices[v as usize]);
    let centroid = (at(corners[0]) + at(corners[1]) + at(corners[2])) / 3.0;

    // A line from the face's middle towards a corner leaves through that corner.
    let exit = faces::try_exit(
        &cube,
        &pairs,
        face,
        centroid,
        (at(corners[0]) - centroid).get_normal(),
    )
    .expect("a line from inside leaves the face");
    assert_eq!(exit.vertex, corners[0]);
    assert!((exit.at - at(corners[0])).length() < 1e-9);

    // Edges under a point: a corner's own edges, and only folds when asked.
    let mid_side = (at(corners[0]) + at(corners[1])) / 2.0;
    let side_edge = pairs[&section_room::pair_key(corners[0], corners[1])];
    assert_eq!(
        faces::try_the_edge_under(&cube, &graph, &pairs, face, mid_side, 1e-6, false),
        Some(side_edge)
    );
    assert_eq!(
        faces::try_the_edge_under(&cube, &graph, &pairs, face, centroid, 1e-6, false),
        None
    );

    // Across a cube edge of the top, a direction pointing off the top is held by the side, carried
    // over the fold to point down it.
    let cube_edge = (0..graph.edge_count())
        .find(|&e| {
            graph.get_face_count(e) == 2
                && top.contains(&graph.get_face(e, 0)) != top.contains(&graph.get_face(e, 1))
        })
        .expect("the top meets a side");
    let top_face = if top.contains(&graph.get_face(cube_edge, 0)) {
        graph.get_face(cube_edge, 0)
    } else {
        graph.get_face(cube_edge, 1)
    };
    let side = graph.get_face(cube_edge, 0) + graph.get_face(cube_edge, 1) - top_face;
    let side_normal = normal_of(&cube, side);
    let off_the_top = Vector3::new(side_normal.x, side_normal.y, 0.0).get_normal();
    let carried = faces::unfold(&cube, off_the_top, cube_edge, &graph, top_face);
    assert!((carried - Vector3::new(0.0, 0.0, -1.0)).length() < 1e-9);
    let (held_by, along) =
        faces::try_face_holding_at_edge(&cube, &graph, cube_edge, top_face, carried)
            .expect("the side holds a direction running down it");
    assert_eq!(held_by, side);
    assert!((along - carried).length() < 1e-9);

    // A vertex's fan: the face whose wedge holds a direction pointing into it.
    let corner = corners[0];
    let fan: Vec<i32> = (0..cube.faces.len() as i32)
        .filter(|&f| {
            let d = &cube.faces[f as usize];
            [d.v0, d.v1, d.v2].contains(&corner)
        })
        .collect();
    let into_the_top = (centroid - at(corner)).get_normal();
    let (holding, _) = faces::try_face_holding(&cube, corner, into_the_top, &fan)
        .expect("the top's wedge holds it");
    assert_eq!(holding, face);
    let (n, first, second) = faces::try_wedge(&cube, face, corner).expect("a corner of the face");
    assert!(first.cross(second).dot(n) >= 0.0);

    // Turning through a corner the top's two triangles share, starting in the other triangle, finds
    // the face whose wedge holds a direction pointing into this one.
    let other_top = &cube.faces[top[1] as usize];
    let shared: Vec<i32> = corners
        .iter()
        .copied()
        .filter(|c| [other_top.v0, other_top.v1, other_top.v2].contains(c))
        .collect();
    assert_eq!(
        shared.len(),
        2,
        "the top's two triangles share their diagonal"
    );
    let pivot = shared[0];
    let mut turned_face = top[1];
    let mut direction = (centroid - at(pivot)).get_normal();
    assert!(turn::try_turn_out_of_the_corner(
        &cube,
        &graph,
        &pairs,
        pivot,
        None,
        &mut turned_face,
        &mut direction
    ));
    assert_eq!(turned_face, face);
    assert_eq!(
        turn::try_the_corner_under(&cube, face, at(corner), 1e-6),
        Some(corner)
    );
    let walked = pairs[&section_room::pair_key(corners[0], corners[1])];
    let other_edge = turn::try_the_other_edge_at(&cube, face, corners[0], walked, &pairs)
        .expect("the face's other edge at the corner");
    assert_eq!(
        other_edge,
        pairs[&section_room::pair_key(corners[0], corners[2])]
    );
}

/// The join's own readings on a cube: the fold between the top's two triangles, the shortest strip
/// from the top onto a side, the faces a walk ran through, the strip flattened with its lengths kept,
/// a point read into it, a strip's key, and whether a walk arrived.
#[test]
fn rust_only_the_join_reads_strips_and_unfolds_them() {
    use geometry_processing::bevel::sections::section_surface_walk::normal_of;
    use geometry_processing::bevel::sections::section_surface_walk_join as join;
    use geometry_processing::bevel::sections::{SurfaceWalkCrossing, SurfaceWalkResult};
    use vector_math::Vector2;

    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let up = Vector3::new(0.0, 0.0, 1.0);
    let top: Vec<i32> = (0..cube.faces.len() as i32)
        .filter(|&f| normal_of(&cube, f).dot(up) > 0.99)
        .collect();
    let side = (0..cube.faces.len() as i32)
        .find(|&f| normal_of(&cube, f).dot(Vector3::new(1.0, 0.0, 0.0)) > 0.99)
        .expect("a side");

    let diagonal = join::try_the_fold_between(&cube, &graph, &pairs, top[0], top[1], None)
        .expect("the top's triangles share their diagonal");
    let no_crossing: &dyn Fn(i32) -> bool = &|edge| edge == diagonal;
    assert!(
        join::try_the_fold_between(&cube, &graph, &pairs, top[0], top[1], Some(no_crossing))
            .is_none()
    );

    let (strip, folds) = join::try_strip(&cube, &graph, &pairs, top[0], side, None)
        .expect("the side is a few faces away");
    assert_eq!((strip[0], *strip.last().unwrap()), (top[0], side));
    assert_eq!(folds.len(), strip.len() - 1);
    for (pair, &fold) in strip.windows(2).zip(&folds) {
        assert!([graph.get_face(fold, 0), graph.get_face(fold, 1)].contains(&pair[0]));
        assert!([graph.get_face(fold, 0), graph.get_face(fold, 1)].contains(&pair[1]));
    }

    let walk = SurfaceWalkResult::new(
        Vector3::ZERO,
        top[1],
        Vector3::ZERO,
        vec![SurfaceWalkCrossing::new(diagonal, 0.5, Vector3::ZERO)],
    );
    let (faces, walked_folds) =
        join::try_the_faces_walked(&cube, &graph, &pairs, top[0], top[1], &walk, None)
            .expect("the walk crossed the diagonal");
    assert_eq!(
        (faces, walked_folds),
        (vec![top[0], top[1]], vec![diagonal])
    );

    let flat = join::flatten(&cube, &strip).expect("a connected strip");
    for (&one, &at_one) in &flat {
        for (&other, &at_other) in &flat {
            let real = (Vector3::from_vector3_float(cube.vertices[one as usize])
                - Vector3::from_vector3_float(cube.vertices[other as usize]))
            .length();
            let shared_face = strip.iter().any(|&f| {
                let d = &cube.faces[f as usize];
                [d.v0, d.v1, d.v2].contains(&one) && [d.v0, d.v1, d.v2].contains(&other)
            });
            if shared_face {
                assert!(((at_one - at_other).length() - real).abs() < 1e-5);
            }
        }
    }
    let corner = cube.faces[top[0] as usize].v1;
    let corner_at = Vector3::from_vector3_float(cube.vertices[corner as usize]);
    let read = join::in_flat(&cube, &flat, top[0], corner_at).expect("the face is in the strip");
    assert!((read - flat[&corner]).length() < 1e-9);

    let placed = join::place(
        Vector2::ZERO,
        Vector2::new(4.0, 0.0),
        5.0,
        3.0,
        Vector2::new(0.0, 1.0),
    );
    assert!((placed - Vector2::new(4.0, -3.0)).length() < 1e-12);
    assert_eq!(join::key_of(&[3, 1, 2]), "3,1,2");

    let far = Vector3::new(1.0, 0.0, 0.0);
    assert!(join::arrived_at(&cube, &walk, Vector3::ZERO, 1.0, false));
    assert!(!join::arrived_at(&cube, &walk, far, 1.0, true));
}
