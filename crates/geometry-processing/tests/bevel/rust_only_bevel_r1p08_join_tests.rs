//! Rust-only tests for the bevel-core package R1.P08's join and the readings built on it: the line
//! along the surface between two points (`SectionSurfaceWalk.TryJoin`, `TryLayTheLine`), a split
//! station's point placed on it (`SectionBuilder.TheJoinsOwnPoint`), an offset side's reach
//! (`SectionRoom.ReachOfAnOffsetSide`), and the walk setting out from a corner (`TryWalk`), all on
//! a cube. Each pins the rule on a hand-built case.

use indexmap::IndexMap;
use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use solidmesh::bevel::sections::section_split_blend::SplitPlacement;
use solidmesh::bevel::sections::section_surface_walk::normal_of;
use solidmesh::bevel::sections::{
    section_builder, section_builder_walked_side, section_room_smooth_side,
    section_surface_walk_join as join, section_surface_walk_try, BevelSection, SectionPool,
    SectionSide, SurfaceWalkRefusal,
};
use solidmesh::topology::FeatureEdgeSet;

/// The faces of a cube whose normal points along `along`.
fn faces_facing(cube: &Mesh, along: Vector3) -> Vec<i32> {
    (0..cube.faces.len() as i32)
        .filter(|&f| normal_of(cube, f).dot(along) > 0.99)
        .collect()
}

/// A point inside a face: its centroid.
fn centroid(cube: &Mesh, face: i32) -> Vector3 {
    let d = &cube.faces[face as usize];
    [d.v0, d.v1, d.v2]
        .iter()
        .map(|&v| Vector3::from_vector3_float(cube.vertices[v as usize]))
        .fold(Vector3::ZERO, |sum, at| sum + at)
        / 3.0
}

/// A join across the top's diagonal and one over the cube's edge onto a side are laid, crossing the
/// edges between; stopped at creases the second cannot be; ends in one face need no crossing.
#[test]
fn rust_only_a_join_is_laid_along_the_surface() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let features = FeatureEdgeSet::create(&cube);
    let is_a_feature = section_builder::the_feature_test(&features);
    let top = faces_facing(&cube, Vector3::new(0.0, 0.0, 1.0));
    let side = faces_facing(&cube, Vector3::new(1.0, 0.0, 0.0))[0];

    let (a, b) = (centroid(&cube, top[0]), centroid(&cube, top[1]));
    let across = join::try_join(&cube, &graph, &pairs, a, top[0], b, top[1], None)
        .expect("the top's two triangles join");
    assert_eq!(across.len(), 1);
    assert_eq!(
        join::try_join(&cube, &graph, &pairs, a, top[0], a, top[0], None),
        Ok(vec![])
    );

    let s = centroid(&cube, side);
    let (over, unfoldings) =
        join::try_join_counting(&cube, &graph, &pairs, a, top[0], s, side, None);
    let over = over.expect("a join runs over the cube's edge");
    assert!(unfoldings >= 1);
    assert!(over.iter().any(|c| (c.at.x - 5.0).abs() < 1e-6));
    assert!(join::try_join(
        &cube,
        &graph,
        &pairs,
        a,
        top[0],
        s,
        side,
        Some(is_a_feature.as_ref())
    )
    .is_err());

    let laid = join::try_lay_the_line(&cube, &graph, &pairs, a, b, &top, None, false)
        .expect("the strip of the top's two triangles");
    assert_eq!(laid.len(), 1);
}

/// A section with no walk of its own on either side, standing on the cube's top.
fn station(chain_vertex: i32, boundary: i32, walk_face: i32) -> BevelSection {
    BevelSection {
        kind: Default::default(),
        chain_vertex,
        station: Vector3::ZERO,
        tangent: Vector3::ZERO,
        left_boundary: boundary,
        right_boundary: boundary,
        profile: vec![0, 1, 2],
        centre_from: Vector3::ZERO,
        centre_to: Vector3::ZERO,
        left_kind: SectionSide::Offset,
        right_kind: SectionSide::Offset,
        left_slide_edge: -1,
        right_slide_edge: -1,
        left_walk_face: walk_face,
        right_walk_face: walk_face,
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

/// A split station between two neighbours on the top is placed half way along the join between
/// their points, on the top, join-placed and over no fold.
#[test]
fn rust_only_a_split_station_stands_on_its_neighbours_join() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let top = faces_facing(&cube, Vector3::new(0.0, 0.0, 1.0));
    let mut pool = SectionPool::new(&cube.vertices);
    let (a, b) = (centroid(&cube, top[0]), centroid(&cube, top[1]));
    let at_a = pool.add(a).unwrap();
    let at_b = pool.add(b).unwrap();

    let sections = vec![
        station(0, at_a, top[0]),
        station(1, -1, -1),
        station(2, at_b, top[1]),
    ];
    let placement = SplitPlacement {
        before: 0,
        after: 2,
        parameter: 0.5,
    };

    let placed = section_builder_walked_side::the_joins_own_point(
        &sections, None, placement, 1, true, &cube, &graph, &pairs, &mut pool, None, 1e-6,
    )
    .expect("a number");
    assert!(placed.boundary >= 0 && placed.is_join_placed && !placed.crossed_a_fold);
    assert!((pool.position_of(placed.boundary) - (a + b) / 2.0).length() < 1e-9);
    assert!(top.contains(&placed.face));

    let mut not_offset = sections.clone();
    not_offset[1].left_kind = SectionSide::Slide;
    let none = section_builder_walked_side::the_joins_own_point(
        &not_offset,
        None,
        placement,
        1,
        true,
        &cube,
        &graph,
        &pairs,
        &mut pool,
        None,
        1e-6,
    )
    .unwrap();
    assert_eq!(none.boundary, -1);
}

/// On the cube's flat top the reach is the ray across the region to its rim; asked across nothing
/// it is the same reading.
#[test]
fn rust_only_an_offset_side_reaches_the_rim_of_its_flat_region() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let top = faces_facing(&cube, Vector3::new(0.0, 0.0, 1.0));
    let max = cube.vertices.iter().fold(f32::MIN, |hi, v| hi.max(v.z)) as f64;
    let middle = Vector3::new(0.0, 0.0, max);
    let mut regions = IndexMap::new();

    let reach = section_room_smooth_side::reach_of_an_offset_side(
        &cube,
        &graph,
        &pairs,
        &mut regions,
        &top,
        -1,
        middle,
        Vector3::new(1.0, 0.0, 0.0),
        None,
        3.0,
    );
    assert!((reach - 5.0).abs() < 1e-6, "reach {reach}");
    assert_eq!(regions.len(), 2);
}

/// A walk setting out from a corner of the top that its named triangle cannot hold turns through
/// the corner into the triangle that can, and walks on from there.
#[test]
fn rust_only_a_walk_sets_out_from_a_corner() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let top = faces_facing(&cube, Vector3::new(0.0, 0.0, 1.0));
    let first = &cube.faces[top[0] as usize];
    let second = &cube.faces[top[1] as usize];
    let shared: Vec<i32> = [first.v0, first.v1, first.v2]
        .into_iter()
        .filter(|v| [second.v0, second.v1, second.v2].contains(v))
        .collect();
    assert_eq!(shared.len(), 2, "the top's triangles share their diagonal");
    let corner = Vector3::from_vector3_float(cube.vertices[shared[0] as usize]);
    let into_the_first = (centroid(&cube, top[0]) - corner).get_normal();

    let walk = section_surface_walk_try::try_walk(
        &cube,
        &graph,
        &pairs,
        corner,
        top[1],
        into_the_first,
        1.0,
        None,
    )
    .expect("the corner's fan holds the direction");
    assert_eq!(walk.end_face, top[0]);
    assert_eq!(walk.refusal, SurfaceWalkRefusal::None);
    assert!((walk.end - (corner + into_the_first)).length() < 1e-6);
}
