//! Rust-only tests for the bevel-core package R1.P08's surface walk, split from
//! `rust_only_bevel_r1p08_tests.rs` by responsibility: the walk's small readings and face readings
//! (`SectionSurfaceWalk`), the turn through a vertex (`SectionWalkVertexTurn`), the join's strip
//! readings (`SectionSurfaceWalk.Join`) and the walk itself (`TryWalk`), all on a cube. Each pins
//! the rule on a hand-built case.

use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::MeshEdgeGraph;

use solidmesh::bevel::sections::{section_builder, section_room};
use solidmesh::topology::FeatureEdgeSet;

/// The surface walk's small readings on a cube: its grid, one point read twice, feet along an edge,
/// folds (a cube edge is one, a face's diagonal is not), a parameter turned to the graph's way, and a
/// refusal carrying where it stopped.
#[test]
fn rust_only_the_surface_walk_reads_folds_and_feet() {
    use solidmesh::bevel::sections::section_surface_walk as walk;
    use solidmesh::bevel::sections::{SurfaceWalkCrossing, SurfaceWalkRefusal, SurfaceWalkResult};
    use solidmesh::bevel::surface::SurfaceVertexPool;

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
    use solidmesh::bevel::sections::section_surface_walk::normal_of;
    use solidmesh::bevel::sections::section_surface_walk_faces as faces;
    use solidmesh::bevel::sections::section_walk_vertex_turn as turn;

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
    use solidmesh::bevel::sections::section_surface_walk::normal_of;
    use solidmesh::bevel::sections::section_surface_walk_join as join;
    use solidmesh::bevel::sections::{SurfaceWalkCrossing, SurfaceWalkResult};
    use solidmesh_math::Vector2;

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

/// The walk itself on a 10 mm cube centred on the origin: from the middle of the top, a short reach
/// stays on the top; a long one runs over the top's edge and down the side, its length kept; stopped
/// at creases it is refused at the edge having gone 5; and a reach that runs out on the edge lands
/// ON it.
#[test]
fn rust_only_the_walk_runs_over_the_edge_or_stops_at_the_crease() {
    use solidmesh::bevel::sections::section_surface_walk::normal_of;
    use solidmesh::bevel::sections::section_surface_walk_try::try_walk;
    use solidmesh::bevel::sections::SurfaceWalkRefusal;

    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let features = FeatureEdgeSet::create(&cube);
    let is_a_feature = section_builder::the_feature_test(&features);

    let up = Vector3::new(0.0, 0.0, 1.0);
    let top: Vec<i32> = (0..cube.faces.len() as i32)
        .filter(|&f| normal_of(&cube, f).dot(up) > 0.99)
        .collect();
    let max = cube.vertices.iter().fold(f32::MIN, |hi, v| hi.max(v.z));
    let middle_of_the_top = Vector3::new(0.0, 0.0, max as f64);
    let start_face = *top
        .iter()
        .find(|&&f| {
            let d = &cube.faces[f as usize];
            let a = Vector3::from_vector3_float(cube.vertices[d.v0 as usize]);
            let b = Vector3::from_vector3_float(cube.vertices[d.v1 as usize]);
            let c = Vector3::from_vector3_float(cube.vertices[d.v2 as usize]);
            let n = (b - a).cross(c - a);
            let inside = |p: Vector3, q: Vector3| {
                (q - p)
                    .cross(middle_of_the_top + Vector3::new(1.0, 0.1, 0.0) - p)
                    .dot(n)
                    >= 0.0
            };
            inside(a, b) && inside(b, c) && inside(c, a)
        })
        .expect("a top triangle holds a point just off the middle");
    let from = middle_of_the_top + Vector3::new(1.0, 0.1, 0.0);
    let x = Vector3::new(1.0, 0.0, 0.0);
    let to_the_edge = 5.0 - 1.0;

    let short =
        try_walk(&cube, &graph, &pairs, from, start_face, x, 1.0, None).expect("a short reach");
    assert!((short.end - (from + x)).length() < 1e-6);
    assert_eq!(short.refusal, SurfaceWalkRefusal::None);

    let long = try_walk(
        &cube,
        &graph,
        &pairs,
        from,
        start_face,
        x,
        to_the_edge + 2.0,
        None,
    )
    .expect("over the edge and down the side");
    assert!(
        (long.end - Vector3::new(5.0, 0.1, max as f64 - 2.0)).length() < 1e-5,
        "{:?}",
        long.end
    );
    assert!(normal_of(&cube, long.end_face).dot(x) > 0.99);
    assert!(long.crossings.iter().any(|c| (c.at.x - 5.0).abs() < 1e-6));

    let stopped = try_walk(
        &cube,
        &graph,
        &pairs,
        from,
        start_face,
        x,
        to_the_edge + 2.0,
        Some(is_a_feature.as_ref()),
    )
    .expect_err("the cube's edge is a crease");
    assert_eq!(stopped.refusal, SurfaceWalkRefusal::CreaseReached);
    assert!((stopped.travelled - to_the_edge).abs() < 1e-6);
    assert!(stopped.end_edge >= 0);

    let onto = try_walk(
        &cube,
        &graph,
        &pairs,
        from,
        start_face,
        x,
        to_the_edge,
        None,
    )
    .expect("a reach that runs out on the edge");
    assert!(onto.end_edge >= 0 && (onto.end.x - 5.0).abs() < 1e-9);

    let nothing = try_walk(
        &cube,
        &graph,
        &pairs,
        from,
        start_face,
        Vector3::ZERO,
        1.0,
        None,
    )
    .expect_err("no direction");
    assert_eq!(nothing.refusal, SurfaceWalkRefusal::NothingToWalk);
}
