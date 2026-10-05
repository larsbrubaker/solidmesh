//! Rust-only tests for the rest of bevel slice 8: where a crease's ball enters a round
//! (`bevel_round_entry`), the rounds a part arrives with (`bevel_source_rounds::create`) and the
//! walk of a side's fan at a chain vertex (`sections::section_fan_walk`). MatterCAD exercises
//! these only through the sweep and the section construction (`BevelCanalOverTheRoundTests`,
//! `SectionSurfaceWalkTests`, `SectionCornerTests`, ...), which land with the cyclic core; these
//! hold the pieces to their own contracts until then. `BevelSourceRounds.TryFit` has its ported
//! class in `bevel_source_round_tests.rs`.

use geometry_processing::bevel::bevel_round_entry;
use geometry_processing::bevel::bevel_source_rounds;
use geometry_processing::bevel::sections::section_fan_walk::{self, pair_key};
use geometry_processing::bevel::{
    BevelAdvisoryCode, BevelRoundDescriptor, BevelRoundFaces, BevelSurfaceLabels,
};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, ChainEdge, EdgeConvexity, FaceGroup,
    FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

use super::source_round_test_meshes;

const SIZE: f64 = 2.0;

/// The round a size 2 cutter drawn with 64 chords leaves on a box edge along y at x = 0, z = 0: axis at (-2, *, -2),
/// spanning from the top face's radial (+z) to the side's (+x). Its top tangent plane is z = 0
/// for x < -2.
fn box_edge_round() -> BevelRoundDescriptor {
    BevelRoundDescriptor::new(
        Vector3::new(-SIZE, 0.0, -SIZE),
        Vector3::new(0.0, 1.0, 0.0),
        SIZE,
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        -20.0,
        20.0,
        64,
    )
}

/// A crease lying on that round's top tangent plane, running a little obliquely from well out on
/// the flat to the tangent line (nearly square to the round's axis: a wall much more oblique than
/// that puts the ball's centre outside the spine's tightest turn, where the station solve rightly
/// refuses), with a vertical wall standing on its far side. Face 0 is the flat (a
/// tangent plane of the round), face 1 the wall. The wall's outward normal points across the
/// flat toward the round, so the ball rolling in the crease climbs onto the round before the
/// crease reaches it.
fn crease_onto_the_round(read_from_the_source_mesh: bool) -> (BevelRoundFaces, ChainEdge) {
    let start = Vector3::new(-8.0, -1.5, 0.0);
    let end = Vector3::new(-2.0, 0.0, 0.0);
    let wall_normal = Vector3::new(1.5, -6.0, 0.0).get_normal();

    let mut part = Mesh::new();
    part.create_face(&[start, Vector3::new(-2.0, -10.0, 0.0), end]);
    part.create_face(&[start, end, Vector3::new(-5.0, -0.75, 5.0)]);

    let rounds =
        BevelRoundFaces::label(&part, &[box_edge_round()], SIZE, read_from_the_source_mesh);

    let edge = FeatureEdge {
        mesh_edge_index: 7,
        start_vertex: 0,
        end_vertex: 2,
        start,
        end,
        direction: (end - start).get_normal(),
        face0: 0,
        face1: 1,
        normal0: Vector3::new(0.0, 0.0, 1.0),
        normal1: wall_normal,
        turn_angle_degrees: -90.0,
        convexity: EdgeConvexity::Concave,
    };

    (rounds, ChainEdge::new(3, edge, false))
}

fn on_the_round(rounds: &BevelRoundFaces, edge: &ChainEdge, point: Vector3) -> bool {
    let (round, _) = rounds.try_read_face(edge.left_face()).unwrap();

    round
        .try_find_station(point, edge.from(), edge.right_normal(), SIZE)
        .is_some()
}

#[test]
fn rust_only_the_crease_is_cut_where_the_ball_enters_the_round() {
    let (rounds, edge) = crease_onto_the_round(false);

    assert!(rounds.try_read_face(0).is_some_and(|(_, tangent)| tangent));
    assert!(rounds.try_read_face(1).is_none());

    let cut = bevel_round_entry::subdivide(&[edge], Some(&rounds), SIZE, true)
        .expect("the crease straddles the round's entry");

    assert!(cut.len() >= 2, "{} pieces", cut.len());

    // The pieces run end to end along the very edge, keeping everything but where they stand.
    assert_eq!(cut[0].from(), edge.from());
    assert_eq!(cut[cut.len() - 1].to(), edge.to());

    for pair in cut.windows(2) {
        assert_eq!(pair[0].to(), pair[1].from());
    }

    for piece in &cut {
        assert_eq!(piece.edge_index, edge.edge_index);
        assert_eq!(piece.edge.mesh_edge_index, edge.edge.mesh_edge_index);
        assert_eq!(piece.left_face(), edge.left_face());
        assert_eq!(piece.edge.convexity, EdgeConvexity::Concave);
        assert!(piece.direction().dot(edge.direction()) > 1.0 - 1e-12);
    }

    // The flat stretch is ONE piece, and the first cut is where the ball enters the round: just
    // short of it the ball is still on the flat, just past it on the round.
    let entry = cut[0].to();
    let along = edge.direction();
    assert!(!on_the_round(&rounds, &edge, entry - (along * 1e-6)));
    assert!(on_the_round(&rounds, &edge, entry + (along * 1e-6)));

    // The on-round stretch is chopped no coarser than the round's own facet chord.
    let round = box_edge_round();
    let chord = 2.0 * round.radius * (std::f64::consts::PI / round.segments as f64).sin();
    for piece in &cut[1..] {
        assert!((piece.to() - piece.from()).length() <= chord + 1e-9);
    }
}

#[test]
fn rust_only_a_reversed_crease_is_cut_the_same_with_its_pieces_reversed() {
    let (rounds, edge) = crease_onto_the_round(false);
    let forward = bevel_round_entry::subdivide(&[edge], Some(&rounds), SIZE, true).unwrap();

    // The same physical crease walked the other way: its left face is now the wall, and the round
    // is on the right.
    let reversed = edge.reversed();
    let backward = bevel_round_entry::subdivide(&[reversed], Some(&rounds), SIZE, true).unwrap();

    assert_eq!(forward.len(), backward.len());
    assert!(backward.iter().all(|piece| piece.is_reversed));
    assert_eq!(backward[0].from(), reversed.from());
    assert_eq!(backward[backward.len() - 1].to(), reversed.to());

    for pair in backward.windows(2) {
        assert_eq!(pair[0].to(), pair[1].from());
    }

    // The round's entry is the LAST cut walking backwards, and stands where the forward walk's
    // first did.
    let forward_entry = forward[0].to();
    let backward_entry = backward[backward.len() - 1].from();
    assert!((forward_entry - backward_entry).length() < 1e-9);
}

#[test]
fn rust_only_a_chain_that_never_meets_a_round_is_handed_back_as_it_came() {
    let (rounds, edge) = crease_onto_the_round(false);

    // No rounds at all, a chamfer, an empty chain.
    assert!(bevel_round_entry::subdivide(&[edge], None, SIZE, true).is_none());
    assert!(bevel_round_entry::subdivide(&[edge], Some(&rounds), SIZE, false).is_none());
    assert!(bevel_round_entry::subdivide(&[], Some(&rounds), SIZE, true).is_none());

    // A convex edge is not a bead.
    let mut convex = edge;
    convex.edge.convexity = EdgeConvexity::Convex;
    assert!(bevel_round_entry::subdivide(&[convex], Some(&rounds), SIZE, true).is_none());

    // An edge whose ball never reaches the round: the first half of the crease, well out on the
    // flat.
    let mut far = edge;
    far.edge.end = (edge.edge.start + edge.edge.end) * 0.5;
    assert!(bevel_round_entry::subdivide(&[far], Some(&rounds), SIZE, true).is_none());

    // ...but a chain that has one is copied whole around the cut edge, in order.
    let cut = bevel_round_entry::subdivide(&[far, edge, far], Some(&rounds), SIZE, true).unwrap();
    assert_eq!(cut[0], far);
    assert_eq!(cut[cut.len() - 1], far);
    assert_eq!(cut[1].from(), edge.from());
}

#[test]
fn rust_only_a_source_round_is_cut_only_where_the_contact_leaves_the_creases_facet() {
    let (cut_rounds, edge) = crease_onto_the_round(false);
    let (source_rounds, _) = crease_onto_the_round(true);

    let first_pass = bevel_round_entry::subdivide(&[edge], Some(&cut_rounds), SIZE, true).unwrap();
    let source = bevel_round_entry::subdivide(&[edge], Some(&source_rounds), SIZE, true).unwrap();

    // The source round's test is the stricter of the two - the contact has to have left the
    // crease's own facet by more than twice the sagitta as well - so its entry is later along the
    // crease than the cut round's.
    let entry = |pieces: &[ChainEdge]| (pieces[0].to() - edge.from()).length();
    assert!(entry(&source) > entry(&first_pass) + 1e-6);
}

#[test]
fn rust_only_source_rounds_are_read_off_the_post_a_selected_crease_runs_along() {
    let part =
        source_round_test_meshes::create_post_in_a_wall(40, 10.0, Vector2::new(5.0, -4.0), -5.0);
    let features = FeatureEdgeSet::create(&part);

    let seed = features
        .edges()
        .iter()
        .position(|edge| {
            edge.convexity == EdgeConvexity::Concave
                && (edge.direction.z.abs() - 1.0).abs() < 1e-6
                && (edge.end - edge.start).length() > 10.0
        })
        .expect("the post meets the wall in a long vertical crease") as i32;

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        seed,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap();
    let resolution = bevel_edge_group_resolver::resolve(&group, &part);
    assert!(resolution.is_resolved());

    let mut labels = BevelSurfaceLabels::new(&part, None);
    let rounds = bevel_source_rounds::create(
        &part,
        std::slice::from_ref(&resolution),
        &[true],
        &mut labels,
        2.0,
        true,
    )
    .expect("the post is a round");

    assert!(rounds.read_from_the_source_mesh());
    assert!(rounds.count() > 0);

    let crease = features.edges()[seed as usize];
    let on_post = rounds
        .try_read_face(crease.face0)
        .or(rounds.try_read_face(crease.face1));
    assert!(on_post.is_some_and(|(round, _)| (round.radius - 10.0).abs() < 1e-3));

    // Nothing for a chamfer, nor for a group no tool is built for.
    let mut labels = BevelSurfaceLabels::new(&part, None);
    assert!(bevel_source_rounds::create(
        &part,
        std::slice::from_ref(&resolution),
        &[true],
        &mut labels,
        2.0,
        false
    )
    .is_none());
    assert!(bevel_source_rounds::create(
        &part,
        std::slice::from_ref(&resolution),
        &[false],
        &mut labels,
        2.0,
        true
    )
    .is_none());
}

/// A flat hexagonal fan round vertex 0, its rim 1..=6, every face wound up (+z).
fn hexagon(open: bool) -> Mesh {
    let mut mesh = Mesh::new();
    let rim: Vec<Vector3> = (0..6)
        .map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 6.0;
            Vector3::new(angle.cos(), angle.sin(), 0.0)
        })
        .collect();

    let faces = if open { 5 } else { 6 };
    for i in 0..faces {
        mesh.create_face(&[Vector3::ZERO, rim[i], rim[(i + 1) % 6]]);
    }

    mesh.clean_and_merge();

    mesh
}

/// The vertex nearest a point, and where it stands.
fn vertex_near(mesh: &Mesh, point: Vector3) -> (i32, Vector3) {
    let (index, position) = mesh
        .vertices
        .iter()
        .enumerate()
        .map(|(i, v)| (i as i32, Vector3::from_vector3_float(*v)))
        .min_by(|a, b| (a.1 - point).length().total_cmp(&(b.1 - point).length()))
        .unwrap();

    (index, position)
}

fn edge_between(mesh: &Mesh, one: Vector3, other: Vector3) -> FeatureEdge {
    let graph = mesh.get_mesh_edge_graph();

    (0..graph.edge_count())
        .filter_map(|edge| FeatureEdge::try_classify(mesh, &graph, edge, 0.5))
        .find(|edge| {
            (edge.start == one && edge.end == other) || (edge.start == other && edge.end == one)
        })
        .expect("the two points share an interior edge")
}

/// The chain edge running from `from` to `to` over that edge.
fn chain_edge(mesh: &Mesh, from: Vector3, to: Vector3) -> ChainEdge {
    let edge = edge_between(mesh, from, to);

    ChainEdge::new(0, edge, edge.start != from)
}

#[test]
fn rust_only_the_fan_walk_turns_each_side_from_the_arriving_edge_to_the_leaving_one() {
    let mesh = hexagon(false);
    let graph = mesh.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (centre, middle) = vertex_near(&mesh, Vector3::ZERO);
    let (_, east) = vertex_near(&mesh, Vector3::new(1.0, 0.0, 0.0));
    let (_, west) = vertex_near(&mesh, Vector3::new(-1.0, 0.0, 0.0));
    let arriving = chain_edge(&mesh, west, middle);
    let leaving = chain_edge(&mesh, middle, east);
    assert_eq!(arriving.to_vertex(), centre);
    assert_eq!(leaving.from_vertex(), centre);

    let mut seen = Vec::new();
    for on_the_left in [true, false] {
        let fan = section_fan_walk::try_walk(
            &mesh,
            &graph,
            &edge_at_pair,
            centre,
            &arriving,
            &leaving,
            on_the_left,
            Vector3::ZERO,
        )
        .expect("a closed fan walks");

        assert_eq!(fan.faces.len(), 3);
        assert_eq!(fan.crossed.len(), fan.faces.len() - 1);
        assert_eq!(
            fan.faces[0],
            if on_the_left {
                arriving.left_face()
            } else {
                arriving.right_face()
            }
        );

        // Every crossed edge leaves the centre and joins the two faces it was crossed between.
        for (j, &crossed) in fan.crossed.iter().enumerate() {
            assert!(graph.get_vertex0(crossed) == centre || graph.get_vertex1(crossed) == centre);
            let faces = [graph.get_face(crossed, 0), graph.get_face(crossed, 1)];
            assert!(faces.contains(&fan.faces[j]) && faces.contains(&fan.faces[j + 1]));
            assert!(!section_fan_walk::is_a_crease(&mesh, &graph, crossed));
        }

        seen.extend(fan.faces);
    }

    // The two sides between them are the whole fan, each face once.
    seen.sort();
    assert_eq!(seen, vec![0, 1, 2, 3, 4, 5]);
}

#[test]
fn rust_only_the_fan_walk_refuses_a_fan_that_runs_off_the_mesh() {
    let mesh = hexagon(true);
    let graph = mesh.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (centre, _) = vertex_near(&mesh, Vector3::ZERO);

    // From the face north of the centre, turning away from one of its edges at the centre, toward
    // an edge the walk can never reach: whichever way it turns it runs out at the open rim.
    let station = Vector3::new(0.25, 0.5, 0.0);
    let north = mesh
        .faces
        .iter()
        .position(|face| {
            let corners = [face.v0, face.v1, face.v2];
            corners.contains(&centre)
                && corners
                    .iter()
                    .all(|&v| v == centre || mesh.vertices[v as usize].y > 0.5)
        })
        .expect("the fan has a face due north") as i32;
    let drawn = &mesh.faces[north as usize];
    let spoke = [drawn.v0, drawn.v1, drawn.v2]
        .into_iter()
        .find(|&v| v != centre)
        .unwrap();
    let start_edge = edge_at_pair[&pair_key(centre, spoke)];
    let refusal = section_fan_walk::try_walk_to(
        &mesh,
        &graph,
        &edge_at_pair,
        centre,
        north,
        start_edge,
        -5,
        station,
    )
    .unwrap_err();

    assert_eq!(refusal.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
    assert_eq!(refusal.args(), &[station.into()]);

    // The open rim edge is a "crease" too: nothing may cross it unnoticed.
    let (east, _) = vertex_near(&mesh, Vector3::new(1.0, 0.0, 0.0));
    let rim_edge = edge_at_pair[&pair_key(centre, east)];
    assert_eq!(graph.get_face_count(rim_edge), 1);
    assert!(section_fan_walk::is_a_crease(&mesh, &graph, rim_edge));
}

#[test]
fn rust_only_a_vertex_pair_key_is_the_same_either_way_round() {
    assert_eq!(pair_key(3, 9), pair_key(9, 3));
    assert_ne!(pair_key(3, 9), pair_key(3, 10));
    assert_eq!(pair_key(1, 2), (1i64 << 32) | 2);
}
