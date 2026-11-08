//! Rust-only tests for the bevel-core package R1.P11's methods that no other test covers on its
//! own: the section's constructor (`BevelSection`), a chain's sharp turns (`SectionCorner`), the
//! corner pass over a run's stations (`SectionAbsorbedSplits.BehindTheCorners`) and the split
//! blend's small readings (`SectionSplitBlend`). Each pins the rule on a hand-built case.

use solidmesh_math::Vector3;

use solidmesh::bevel::sections::section_split_blend::{self, SplitPlacement};
use solidmesh::bevel::sections::{
    section_corner, BevelSection, SectionAbsorbedSplits, SectionPool, SectionSide,
};
use solidmesh::topology::{ChainEdge, FeatureEdge};

/// A chain edge running from `start` to `end`.
fn edge(start: Vector3, end: Vector3) -> ChainEdge {
    ChainEdge::new(
        0,
        FeatureEdge {
            start,
            end,
            direction: (end - start).get_normal(),
            ..Default::default()
        },
        false,
    )
}

/// A section's profile has to run from its left boundary to its right by those very indices.
#[test]
fn rust_only_a_section_is_built_from_its_boundaries() {
    let build = |profile: Vec<i32>| {
        BevelSection::new(
            7,
            Vector3::ZERO,
            Vector3::new(1.0, 0.0, 0.0),
            3,
            9,
            profile,
            Vector3::ZERO,
            Vector3::ZERO,
            SectionSide::Offset,
            SectionSide::Slide,
            -1,
            4,
            0.5,
        )
    };

    let section = build(vec![3, 5, 9]).expect("its profile ends on its boundaries");
    assert_eq!(
        (
            section.chain_vertex,
            section.segments(),
            section.left_walk_face
        ),
        (7, 2, -1)
    );
    assert!(build(vec![3]).is_err());
    assert!(build(vec![3, 5, 8]).is_err());
}

/// A square turn is sharp; a turn of a few degrees is a point of a curve.
#[test]
fn rust_only_a_corner_is_a_sharp_turn() {
    let x = edge(Vector3::ZERO, Vector3::new(10.0, 0.0, 0.0));
    let y = edge(Vector3::new(10.0, 0.0, 0.0), Vector3::new(10.0, 10.0, 0.0));
    let slight = edge(Vector3::new(10.0, 0.0, 0.0), Vector3::new(20.0, 1.0, 0.0));

    assert!((section_corner::turn_degrees(&x, &y) - 90.0).abs() < 1e-9);
    assert!(section_corner::is_sharp_turn(&x, &y));
    assert!(!section_corner::is_sharp_turn(&x, &slight));
    assert!((section_corner::smooth_surface_dot() - 30f64.to_radians().cos()).abs() < 1e-15);
}

/// Round a square with an extra station just past one corner on each edge: the station whose offset
/// boundary stands behind the corner's meet is swallowed by it, on that side, and the pass reports
/// it; an open run is not read.
#[test]
fn rust_only_the_corner_swallows_the_station_behind_its_meet() {
    let corners = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ];
    let count = corners.len();
    let edges: Vec<ChainEdge> = (0..count)
        .map(|i| edge(corners[i], corners[(i + 1) % count]))
        .collect();

    let mut pool = SectionPool::new(&[]);
    let mut sections = Vec::new();
    for (i, at) in corners.iter().enumerate() {
        // Each station's offset boundary a unit inwards; the corner at 0 meets at (1, 1), and the
        // station at (1, 0) stands its boundary at (0.5, 1) - behind that meet along the leaving edge.
        let inward = if i == 0 {
            Vector3::new(1.0, 1.0, 0.0)
        } else if i == 1 {
            Vector3::new(0.5, 1.0, 0.0)
        } else {
            *at + Vector3::new(0.0, 0.0, 1.0)
        };
        let boundary = pool.add(inward).unwrap();
        let other = pool
            .add(*at + Vector3::new(0.0, 0.0, -1.0 - i as f64))
            .unwrap();
        sections.push(
            BevelSection::new(
                i as i32,
                *at,
                Vector3::ZERO,
                boundary,
                other,
                vec![boundary, other],
                *at,
                *at,
                SectionSide::Offset,
                SectionSide::Slide,
                -1,
                -1,
                1.0,
            )
            .unwrap(),
        );
    }

    let (left, right) = SectionAbsorbedSplits::behind_the_corners(&sections, &edges, true, &pool)
        .expect("the corner at the origin swallows the station after it");
    assert_eq!(left[1], sections[0].left_boundary);
    assert_eq!(right[1], -1);
    assert!(SectionAbsorbedSplits::behind_the_corners(&sections, &edges, false, &pool).is_none());
}

/// The split blend's small readings: the nearest unsplit neighbour either way round, points and
/// numbers part way between two, how far along the chain a point stands, and the absorb switch.
#[test]
fn rust_only_the_split_blend_reads_its_neighbours() {
    let split = [false, true, true, false, true];
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&split, 1, 1),
        3
    );
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&split, 1, -1),
        0
    );
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&split, 4, 1),
        0
    );
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&[false, true], 0, 1),
        -1
    );

    let half = section_split_blend::between(Vector3::ZERO, Vector3::new(2.0, 4.0, 0.0), 0.5);
    assert_eq!(half, Vector3::new(1.0, 2.0, 0.0));
    assert_eq!(section_split_blend::between_values(1.0, 3.0, 0.25), 1.5);
    assert_eq!(
        section_split_blend::how_far_along(
            Vector3::new(3.0, 1.0, 0.0),
            Vector3::ZERO,
            Vector3::new(1.0, 0.0, 0.0)
        ),
        3.0
    );
    assert_eq!(SplitPlacement::new(0, 2, 0.5).parameter, 0.5);
    assert!(section_split_blend::absorb_split_vertices());
}

/// A run of four stations, one of them standing on the straight chord between its neighbours with
/// both sides offsets: the blend redraws it from them, its two boundary points slid along the chain
/// onto the line between the neighbours' boundaries, and says where along the chord it stands.
#[test]
fn rust_only_a_split_station_is_blended_from_its_neighbours() {
    let mesh = solidmesh_mesh::solids::platonic_solids::create_cube(10.0, 10.0, 10.0);
    let mut pool = SectionPool::new(&[]);
    let x = Vector3::new(1.0, 0.0, 0.0);
    let mut section = |vertex: i32, station: Vector3, left: Vector3, right: Vector3| {
        let l = pool.add(left).unwrap();
        let r = pool.add(right).unwrap();
        BevelSection::new(
            vertex,
            station,
            x,
            l,
            r,
            vec![l, r],
            station,
            station,
            SectionSide::Offset,
            SectionSide::Offset,
            -1,
            -1,
            1.0,
        )
        .unwrap()
    };
    let mut sections = vec![
        section(
            0,
            Vector3::ZERO,
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        ),
        section(
            1,
            x,
            Vector3::new(1.3, 1.0, 0.0),
            Vector3::new(0.8, -1.0, 0.0),
        ),
        section(
            2,
            x * 2.0,
            Vector3::new(2.0, 1.0, 0.0),
            Vector3::new(2.0, -1.0, 0.0),
        ),
        section(
            3,
            Vector3::new(1.0, 5.0, 0.0),
            Vector3::new(1.0, 6.0, 0.0),
            Vector3::new(1.0, 4.0, 0.0),
        ),
    ];
    let lattice = solidmesh::bevel::bevel_split_chords::lattice_for(&mesh);
    assert!(section_split_blend::reads_as_a_split_vertex_at(
        &sections, 1, lattice
    ));
    assert!(!section_split_blend::reads_as_a_split_vertex_at(
        &sections, 0, lattice
    ));

    let placed = section_split_blend::apply(&mut sections, &mesh, &mut pool).expect("numbers");
    assert_eq!(placed[1], SplitPlacement::new(0, 2, 0.5));
    assert_eq!(placed[0].before, -1);
    let blended = &sections[1];
    assert!(
        (pool.position_of(blended.left_boundary) - Vector3::new(1.0, 1.0, 0.0)).length() < 1e-6
    );
    assert!(
        (pool.position_of(blended.right_boundary) - Vector3::new(1.0, -1.0, 0.0)).length() < 1e-6
    );
    assert!((blended.centre_from - Vector3::new(0.7, 0.0, 0.0)).length() < 1e-9);
}

/// The room's per-station readings: a boundary's motion per radius, the radius up to which a
/// boundary segment keeps the edge's direction, which stations carry the chord's room, and the
/// part's cache of edges and lattices.
#[test]
fn rust_only_the_room_reads_boundaries_and_caches_the_part() {
    use solidmesh::bevel::sections::section_room::{self, Boundary};
    use solidmesh::bevel::sections::SectionRoomCache;
    use solidmesh_mesh::solids::platonic_solids;
    use solidmesh_mesh::MeshEdgeGraph;

    let x = Vector3::new(1.0, 0.0, 0.0);
    let here = Boundary::new(Vector3::new(0.0, 1.0, 0.0), 2.0, 5.0, true);
    assert_eq!(here.per_radius(), Vector3::new(0.0, 2.0, 0.0));

    // The next boundary closes in along the edge by half a unit per radius: a segment 3 long stands
    // still at a radius of 6.
    let closing = Boundary::new(Vector3::new(-0.5, 2.0, 0.0), 1.0, 5.0, true);
    assert!((section_room::keeps_its_direction(x * 3.0, x, here, closing) - 6.0).abs() < 1e-12);
    assert_eq!(
        section_room::keeps_its_direction(x * 3.0, x, closing, here),
        f64::MAX
    );

    let stations = [Vector3::ZERO, x, x * 2.0, Vector3::new(1.0, 5.0, 0.0)];
    let sides = [here; 4];
    assert!(section_room::the_chord_carries_its_room(
        &stations, &sides, &sides, 1, 1e-6
    ));
    assert!(!section_room::the_chord_carries_its_room(
        &stations, &sides, &sides, 0, 1e-6
    ));
    assert!(!section_room::the_chord_carries_its_room(
        &stations, &sides, &sides, 1, 0.0
    ));

    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let cache = SectionRoomCache::new(&cube, &graph);
    assert!((cache.span - 300f64.sqrt()).abs() < 1e-9);
    assert!((cache.lattice - 8.0 * cache.span * 1.1920929e-7).abs() < 1e-18);
    assert_eq!(cache.edge_at_pair.len() as i32, graph.edge_count());
    assert_eq!(
        cache.edges_at_vertex.values().map(Vec::len).sum::<usize>() as i32,
        2 * graph.edge_count()
    );
    assert!(cache.feature_regions_by_face.is_empty() && cache.coplanar_regions_by_face.is_empty());
}

/// The builder's canal readings: a slerp a quarter turn round, an arm squashed across a lean, a square
/// edge's canal axis a radius in from both faces, and the creases one side of a cube corner's fan
/// crosses.
#[test]
fn rust_only_the_builder_reads_its_canals() {
    use solidmesh::bevel::sections::section_builder;
    use solidmesh_mesh::solids::platonic_solids;
    use solidmesh_mesh::MeshEdgeGraph;

    let x = Vector3::new(1.0, 0.0, 0.0);
    let y = Vector3::new(0.0, 1.0, 0.0);
    let half = section_builder::slerp(x, y, 0.5, std::f64::consts::FRAC_PI_2);
    assert!((half - Vector3::new(0.5f64.sqrt(), 0.5f64.sqrt(), 0.0)).length() < 1e-12);
    assert_eq!(section_builder::slerp(x, y, 0.5, 0.0), x);
    assert_eq!(
        section_builder::squashed(Vector3::new(2.0, 3.0, 0.0), x, 0.5),
        Vector3::new(1.0, 3.0, 0.0)
    );

    // A square edge along x, its faces' normals +z and -y: the axis stands a radius in from both.
    let square = ChainEdge::new(
        0,
        FeatureEdge {
            start: Vector3::ZERO,
            end: x * 10.0,
            direction: x,
            normal0: Vector3::new(0.0, 0.0, 1.0),
            normal1: Vector3::new(0.0, -1.0, 0.0),
            ..Default::default()
        },
        false,
    );
    let axis = section_builder::try_the_axis_at(&square, 2.0).expect("a square edge has an axis");
    assert!((axis - Vector3::new(0.0, 2.0, -2.0)).length() < 1e-12);
    assert!(section_builder::try_the_canals_corner(&square, &square, 2.0, x).is_none());

    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let graph = MeshEdgeGraph::create(&cube);
    let pairs = section_builder::edges_by_vertex_pair(&graph);
    let features = solidmesh::topology::FeatureEdgeSet::create(&cube);
    let first = features.edges()[0];
    let arriving =
        section_builder::try_feature_edge_between(&features, first.end_vertex, first.start_vertex)
            .expect("an edge into the corner");
    let corner = first.start_vertex;
    let leaving = features
        .edges_at_vertex(corner)
        .iter()
        .filter_map(|&e| {
            let edge = features.edges()[e as usize];
            let other = if edge.start_vertex == corner {
                edge.end_vertex
            } else {
                edge.start_vertex
            };
            (other != first.end_vertex)
                .then(|| section_builder::try_feature_edge_between(&features, corner, other))
                .flatten()
        })
        .next()
        .expect("an edge out of the corner");
    let mut sides_read = 0;
    for on_the_left in [true, false] {
        if let Ok((creases, fan)) = section_builder::try_creases_in_the_fan_reading(
            corner,
            &arriving,
            &leaving,
            on_the_left,
            &cube,
            &graph,
            &pairs,
            None,
        ) {
            assert!(!fan.is_empty());
            assert!(creases
                .iter()
                .all(|&e| section_room_is_fold(&cube, &graph, e)));
            sides_read += 1;
        }
    }
    assert!(
        sides_read >= 1,
        "at least one side of the corner's fan reads"
    );

    fn section_room_is_fold(mesh: &solidmesh_mesh::Mesh, graph: &MeshEdgeGraph, edge: i32) -> bool {
        solidmesh::bevel::sections::section_fan_walk::is_a_crease(mesh, graph, edge)
    }
}

/// A box's top corner, the chain turning from the front-top edge (+x) onto the right-top edge (+y):
/// the top is the shared flat side, the wedge square, the mitre plane square to the two directions'
/// sum through the point the two canal axes meet, and each edge's slid profile lies in that plane.
#[test]
fn rust_only_a_box_corner_reads_its_sides_and_mitre() {
    let x = Vector3::new(1.0, 0.0, 0.0);
    let y = Vector3::new(0.0, 1.0, 0.0);
    let z = Vector3::new(0.0, 0.0, 1.0);
    let chain = |start: Vector3, end: Vector3, left: Vector3, right: Vector3| {
        ChainEdge::new(
            0,
            FeatureEdge {
                start,
                end,
                direction: (end - start).get_normal(),
                normal0: left,
                normal1: right,
                ..Default::default()
            },
            false,
        )
    };
    let arriving = chain(x * -10.0, Vector3::ZERO, z, -y);
    let leaving = chain(Vector3::ZERO, y * 10.0, z, x);
    let r = 2.0;

    let (flat_on_the_left, cosine, wedge) =
        section_corner::try_read_the_sides(&arriving, &leaving).expect("the top is shared");
    assert!(flat_on_the_left && cosine.abs() < 1e-12);
    assert!((wedge - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!(section_corner::looks_like_a_corner(&arriving, &leaving));
    assert!(!section_corner::looks_like_a_corner(&arriving, &arriving));

    assert!(
        (section_corner::the_axis_at(&arriving, Vector3::ZERO, r) - Vector3::new(0.0, r, -r))
            .length()
            < 1e-12
    );
    let (through, normal) =
        section_corner::try_the_mitre_plane(&arriving, &leaving, r).expect("the axes meet");
    assert!((through - Vector3::new(-r, r, -r)).length() < 1e-12);
    assert!((normal - (x + y).get_normal()).length() < 1e-12);

    for edge in [&arriving, &leaving] {
        let profile = section_corner::slid_profile(edge, Vector3::ZERO, normal, r, 5);
        assert_eq!(profile.len(), 5);
        assert!(profile.iter().all(|p| p.dot(normal).abs() < 1e-9));
    }

    // Down the front wall (the arriving edge's right face) away from the edge.
    let away = section_corner::away_in_the_wall(&arriving, false, cosine);
    assert!((away - (-z)).length() < 1e-12);
    assert_eq!(section_corner::shared_kind(3), SectionSide::Slide);
    assert_eq!(section_corner::shared_kind(-1), SectionSide::Offset);
}

/// A section with one side walked takes that walk's end as its profile's end and records the walk;
/// one with neither side walked comes back as it was.
#[test]
fn rust_only_a_section_is_rewritten_with_its_walked_ends() {
    use solidmesh::bevel::sections::section_builder_walked_side::{rewritten, WalkedEnd};

    let section = BevelSection::new(
        4,
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
        10,
        12,
        vec![10, 11, 12],
        Vector3::ZERO,
        Vector3::ZERO,
        SectionSide::Offset,
        SectionSide::Offset,
        -1,
        -1,
        1.0,
    )
    .unwrap();
    let none = WalkedEnd::new(-1, -1, Vector3::ZERO, false, false, -1, 0.0, -1);
    assert_eq!(rewritten(&section, &none, &none), section);

    let walked = WalkedEnd::new(20, 7, Vector3::new(0.0, 1.0, 0.0), true, true, 9, 0.25, 30);
    let redrawn = rewritten(&section, &walked, &none);
    assert_eq!(redrawn.profile, vec![20, 11, 12]);
    assert_eq!((redrawn.left_boundary, redrawn.right_boundary), (20, 12));
    assert_eq!((redrawn.left_walk_face, redrawn.right_walk_face), (7, -1));
    assert!(redrawn.left_boundary_is_join_placed && redrawn.left_walk_crossed_a_fold);
    assert_eq!(
        (
            redrawn.left_walk_edge,
            redrawn.left_walk_parameter,
            redrawn.left_walk_vertex
        ),
        (9, 0.25, 30)
    );
}
