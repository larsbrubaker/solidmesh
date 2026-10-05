//! Rust-only checks of R2.P05's small working-mesh readings: the corner cap's mitre depth, the
//! self-room ceiling over chain edges, the knife-end overrun's plane neighbour and the foreign
//! edge room's knife test by edge, open gap loops meeting, and the opposite-sense crease on the
//! working mesh, valley planes, would-thin splits and the longest-edge path's end.

use geometry_processing::bevel::level_line::{
    chain_on_the_mesh, corner_cap, end_face_sense, foreign_edge_room, knife_end_overrun,
    open_gap_fill, refinement_splits, self_room, strip_clip_valley, BandProfile, GapLoop,
    LevelLineMesh,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square as two triangles sharing the diagonal 0-2, tagged `tags`.
fn square(tags: [i32; 2]) -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&tags),
    )
    .expect("a square builds")
}

#[test]
fn rust_only_a_mitre_depth_averages_the_strip_planes() {
    let one = LevelLineMesh::from_triangles(
        &[v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)],
        &[(0, 1, 2)],
        None,
    )
    .unwrap();
    let up = v(0.0, 0.0, 1.0);

    assert_eq!(
        corner_cap::mitre_depth(&one, &[0, 1, 2], v(0.3, 0.3, 5.0), up, &[0]),
        Some(-5.0)
    );
    // A cycle edge on no strip triangle, or a plane edge-on to the axis: no depth.
    assert_eq!(
        corner_cap::mitre_depth(&one, &[0, 1, 2], v(0.0, 0.0, 5.0), up, &[]),
        None
    );
    assert_eq!(
        corner_cap::mitre_depth(&one, &[0, 1, 2], v(0.0, 0.0, 5.0), v(1.0, 0.0, 0.0), &[0]),
        None
    );
    // The square's diagonal stands on two triangles.
    assert_eq!(
        corner_cap::mitre_depth(&square([0, 0]), &[0, 1, 2], v(0.0, 0.0, 1.0), up, &[0, 1]),
        None
    );
}

#[test]
fn rust_only_the_self_room_ceiling_reads_each_chain_vertex_once() {
    let mesh = square([0, 0]);
    let ceiling = self_room::self_room_ceiling(&mesh, &[(0, 1), (1, 2), (2, 1)]);
    assert_eq!(
        ceiling,
        self_room::chain_ceiling(&[v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(1.0, 1.0, 0.0)])
    );
    assert!((ceiling - 2f64.sqrt()).abs() < 1e-12);
}

#[test]
fn rust_only_the_overrun_finds_the_open_neighbour_on_the_end_plane() {
    let mesh = square([0, 0]);
    let end = |vertex: Vector3, normal: Vector3| knife_end_overrun::KnifeEndOverrunEnd {
        vertex,
        normal,
        ..Default::default()
    };

    // Vertex 0's open edges reach 1 and 3; its diagonal to 2 stands on two triangles.
    assert_eq!(
        knife_end_overrun::meets_the_plane(&mesh, 0, -1, &end(v(1.0, 0.0, 0.0), v(1.0, 0.0, 0.0))),
        1
    );
    assert_eq!(
        knife_end_overrun::meets_the_plane(&mesh, 0, -1, &end(v(0.0, 1.0, 0.0), v(0.0, 1.0, 0.0))),
        3
    );
    assert_eq!(
        knife_end_overrun::meets_the_plane(&mesh, 0, 1, &end(v(1.0, 0.0, 0.0), v(1.0, 0.0, 0.0))),
        -1,
        "the inward vertex is never the answer"
    );
    assert_eq!(
        knife_end_overrun::meets_the_plane(
            &mesh,
            0,
            -1,
            &end(v(1.0, 1.0, 0.0), v(1.0, 1.0, 0.0).get_normal())
        ),
        -1,
        "the diagonal's far end is not on an open edge"
    );
}

#[test]
fn rust_only_a_knife_edge_needs_one_filled_side_and_a_fold() {
    // Both filled, or neither: not a fill's crease at all.
    assert!(!foreign_edge_room::is_a_knife_edge(&square([1, 2]), (0, 2)));
    assert!(!foreign_edge_room::is_a_knife_edge(&square([0, 0]), (0, 2)));
    // One filled, but flat: no knife.
    assert!(!foreign_edge_room::is_a_knife_edge(&square([1, 0]), (0, 2)));
    // An open edge stands on one triangle.
    assert!(!foreign_edge_room::is_a_knife_edge(&square([1, 0]), (0, 1)));
}

fn open_loop(side: i32, vertices: Vec<i32>) -> GapLoop {
    let parameters = vec![0.0; vertices.len()];
    GapLoop::new(side, false, vertices, parameters, 0, false, None)
}

#[test]
fn rust_only_open_loops_meet_end_to_end_on_the_ridge_between_their_sides() {
    let mesh = square([1, 2]);

    assert!(open_gap_fill::on_the_ridge_between(&mesh, 0, 1, 2).unwrap());
    assert!(open_gap_fill::on_the_ridge_between(&mesh, 2, 2, 1).unwrap());
    assert!(!open_gap_fill::on_the_ridge_between(&mesh, 0, 1, 3).unwrap());
    assert!(
        !open_gap_fill::on_the_ridge_between(&mesh, 1, 1, 2).unwrap(),
        "only open edges at 1"
    );

    // Left 0 .. 2 and right 2 .. 0: both ends shared, both on the 1|2 diagonal.
    assert!(open_gap_fill::ends_meet(
        &mesh,
        &open_loop(1, vec![0, 5, 2]),
        &open_loop(2, vec![2, 6, 0])
    )
    .unwrap());
    // Ends that are not the same vertex, or a loop too short to have two ends.
    assert!(
        !open_gap_fill::ends_meet(&mesh, &open_loop(1, vec![0, 2]), &open_loop(2, vec![0, 2]))
            .unwrap()
    );
    assert!(
        !open_gap_fill::ends_meet(&mesh, &open_loop(1, vec![0]), &open_loop(2, vec![2, 0]))
            .unwrap()
    );
    // Shared ends, but the sides are not the ones the ridge divides.
    assert!(
        !open_gap_fill::ends_meet(&mesh, &open_loop(1, vec![0, 2]), &open_loop(3, vec![2, 0]))
            .unwrap()
    );
}

#[test]
fn rust_only_an_opposite_sense_crease_needs_two_triangles_on_each_edge_and_one_filled() {
    // The chain edge 0-1 is open: one triangle.
    assert!(!end_face_sense::is_opposite_sense_crease_on(
        &square([1, 0]),
        (0, 1),
        (0, 2)
    ));
    // The crease's triangles both filled.
    assert!(!end_face_sense::is_opposite_sense_crease_on(
        &square([1, 2]),
        (0, 2),
        (0, 2)
    ));
}

#[test]
fn rust_only_unit_setbacks_read_each_edges_dihedral_and_average_at_the_vertices() {
    // Two triangles folded square along 0-1: the floor z = 0 and the wall y = 0.
    let positions = [
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 1.0),
    ];
    let indices = [(0, 1, 2), (1, 0, 3)];
    let edges = chain_on_the_mesh::chain_edges(&[0, 1], false);

    let chamfer = chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &edges,
        BandProfile::Chamfer,
    )
    .unwrap();
    assert_eq!(chamfer, vec![1.0]);

    // A fillet of radius 1 in a square wedge sets back tan(45 degrees) = 1.
    let fillet = chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &edges,
        BandProfile::Fillet,
    )
    .unwrap();
    assert!((fillet[0] - 1.0).abs() < 1e-12, "{fillet:?}");

    let at_vertices = chain_on_the_mesh::unit_setbacks(
        &positions,
        &indices,
        &[0, 1],
        false,
        &edges,
        BandProfile::Chamfer,
    )
    .unwrap();
    assert_eq!(at_vertices, vec![1.0, 1.0]);

    // An edge on one triangle only is refused, as C# throws.
    let open = chain_on_the_mesh::chain_edges(&[1, 2], false);
    assert!(chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &open,
        BandProfile::Chamfer
    )
    .is_err());
}

#[test]
fn rust_only_valley_planes_are_two_walls_facing_each_other() {
    let positions = [
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, -1.0, 1.0),
        v(0.0, 1.0, 1.0),
    ];
    // A V along x: both walls face up into it.
    let valley = LevelLineMesh::from_triangles(&positions, &[(0, 2, 1), (0, 1, 3)], None).unwrap();
    let walls = strip_clip_valley::valley_planes(&valley, &[(0, 2), (0, 3)]).expect("a V");
    assert_eq!(walls.len(), 2);
    assert!((walls[0].1 - v(0.0, 1.0, 1.0).get_normal()).length() < 1e-12);
    assert!((walls[1].1 - v(0.0, -1.0, 1.0).get_normal()).length() < 1e-12);
    assert!(walls[0].2.contains(&0) && walls[1].2.contains(&0));

    // The same two faces wound the other way are a ridge: neither stands in front of the other.
    let ridge = LevelLineMesh::from_triangles(&positions, &[(0, 1, 2), (0, 3, 1)], None).unwrap();
    assert!(strip_clip_valley::valley_planes(&ridge, &[(0, 2), (0, 3)]).is_none());

    // One plane only.
    assert!(strip_clip_valley::valley_planes(&valley, &[(0, 2), (1, 2)]).is_none());
}

#[test]
fn rust_only_a_split_that_would_leave_a_new_sliver_is_seen() {
    // A low triangle, aspect 100 / 10 = 10.
    let low = LevelLineMesh::from_triangles(
        &[v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0), v(5.0, 1.0, 0.0)],
        &[(0, 1, 2)],
        None,
    )
    .unwrap();

    // Splitting the long side at a fifth leaves a child of aspect 26 / 2 = 13.
    let on_the_edge = v(2.0, 0.0, 0.0);
    assert!(refinement_splits::would_thin_a_child_on_edge(
        &low,
        0,
        1,
        on_the_edge,
        12.0,
        0.0
    ));
    assert!(!refinement_splits::would_thin_a_child_on_edge(
        &low,
        0,
        1,
        on_the_edge,
        32.0,
        0.0
    ));
    assert_eq!(
        refinement_splits::triangle_thinned_by_on_edge(&low, 0, 1, on_the_edge, 12.0, 0.0, Some(5)),
        -1,
        "no triangle of that tag"
    );
    // Guarded: a triangle no longer than the guard is never thinned.
    assert!(!refinement_splits::would_thin_a_child_on_edge(
        &low,
        0,
        1,
        on_the_edge,
        12.0,
        10.0
    ));
    // The same point read through the triangle lands on its edge.
    assert_eq!(
        refinement_splits::triangle_thinned_by(&low, 0, on_the_edge, 12.0, 0.0, false),
        0
    );

    // An interior point 0.4 up the middle: the child on the long side has aspect 100 / 4 = 25,
    // and it stands 0.6 from the apex - farther than 10 / 20, nearer than 10 / 12.
    let inside = v(5.0, 0.4, 0.0);
    assert!(refinement_splits::would_thin_a_child(
        &low, 0, inside, 20.0, 0.0
    ));
    assert!(
        !refinement_splits::would_thin_a_child(&low, 0, inside, 12.0, 0.0),
        "too near the apex to split"
    );
    assert!(!refinement_splits::would_thin_a_child(
        &low, 0, inside, 32.0, 0.0
    ));
    // At a corner nothing is split.
    assert!(!refinement_splits::would_thin_a_child(
        &low,
        0,
        v(10.0, 0.0, 0.0),
        12.0,
        0.0
    ));
}

#[test]
fn rust_only_the_longest_edge_path_ends_where_two_triangles_share_it() {
    // The square's two triangles share their longest edge, the diagonal.
    assert_eq!(
        refinement_splits::terminal_edge(&square([0, 0]), 0),
        Some((0, 2))
    );
    assert_eq!(
        refinement_splits::terminal_edge(&square([0, 0]), 1),
        Some((0, 2))
    );
    // Across a fold between two tags the path stops on the fold.
    assert_eq!(
        refinement_splits::terminal_edge(&square([1, 2]), 0),
        Some((0, 2))
    );
}

#[test]
fn rust_only_every_chain_side_leaves_one_owned_loop() {
    use geometry_processing::bevel::level_line::{gap_loop_walk, BandGroup};

    let group = BandGroup {
        tag: 5,
        members: vec![0],
        side_of_chain: [(0, 1)].into_iter().collect(),
        ..Default::default()
    };
    let owned = |is_closed: bool, side: i32| {
        let mut l = GapLoop::new(side, is_closed, vec![1, 2, 3], vec![0.0; 3], 0, false, None);
        l.group_tag = 5;
        l
    };
    let groups = [group];

    assert!(gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &[true],
        &[owned(true, 1)],
        None
    ));
    // An open loop for a closed chain, none, or two: not one owned loop.
    assert!(!gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &[true],
        &[owned(false, 1)],
        None
    ));
    assert!(gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &[false],
        &[owned(false, 1)],
        None
    ));
    assert!(!gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &[true],
        &[],
        None
    ));
    assert!(!gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &[true],
        &[owned(true, 1), owned(true, 1)],
        None
    ));
    // The other side's loop is not this side's.
    assert!(!gap_loop_walk::every_chain_side_leaves_one_owned_loop(
        &groups,
        &[true],
        &[owned(true, 2)],
        None
    ));
    // A mixed-owner loop outside an end group fails the whole read.
    let mut mixed = owned(true, 1);
    mixed.has_mixed_owners = true;
    assert_eq!(
        gap_loop_walk::every_chain_side_leaves_one_owned_loop_at(
            &groups,
            &[true],
            &[mixed],
            Some(&[vec![1, 2, 3]]),
            true
        ),
        (false, false)
    );
}

#[test]
fn rust_only_spokes_merge_two_rails_by_their_edges_middles() {
    use geometry_processing::bevel::level_line::{open_gap_fill_stations, OpenGapFillOpenRail};

    let rail = |parameters: Vec<f64>| OpenGapFillOpenRail {
        vertices: (0..parameters.len() as i32).collect(),
        parameters,
    };

    // A station both rails carry is one spoke when read together.
    let even = open_gap_fill_stations::spokes(
        &rail(vec![0.0, 1.0, 2.0]),
        &rail(vec![0.0, 1.0, 2.0]),
        2.0,
        true,
    )
    .unwrap();
    assert_eq!(even, vec![(0, 0), (1, 1), (2, 2)]);

    // By middles: the right rail's first edge (middle 0.5) comes before the left's (middle 1).
    let merged = open_gap_fill_stations::spokes(
        &rail(vec![0.0, 2.0]),
        &rail(vec![0.0, 1.0, 2.0]),
        2.0,
        false,
    )
    .unwrap();
    assert_eq!(merged, vec![(0, 0), (0, 1), (1, 1), (1, 2)]);

    // Not together, the shared station is two spokes.
    let apart = open_gap_fill_stations::spokes(
        &rail(vec![0.0, 1.0, 2.0]),
        &rail(vec![0.0, 1.0, 2.0]),
        2.0,
        false,
    )
    .unwrap();
    assert_eq!(apart.len(), 5);
    assert_eq!(apart[apart.len() - 1], (2, 2));

    // Past the chain's end the far ends rule: the left's next station at 2.5 is before the
    // right's at 3.
    let past =
        open_gap_fill_stations::spokes(&rail(vec![0.0, 2.5]), &rail(vec![0.0, 3.0]), 2.0, false)
            .unwrap();
    assert_eq!(past, vec![(0, 0), (1, 0), (1, 1)]);
}

#[test]
fn rust_only_run_outs_are_filed_under_their_creases_groups_when_on() {
    use std::rc::Rc;

    use geometry_processing::bevel::level_line::{
        band_group_run_outs, turn_run_outs_on, LevelLineWork, WorkBudget,
    };

    use crate::level_line_fixture::LevelLineFixture;

    let fixture = LevelLineFixture::load("top-center-face-nose-r1");
    let tags: Vec<i32> = (0..fixture.indices.len() as i32)
        .map(|t| 1 + (t % 2))
        .collect();
    let read = || {
        band_group_run_outs(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            &|a, b| fixture.is_feature_edge(a, b),
            fixture.size,
            &tags,
            None,
        )
    };

    // Off - no budget in force, so C#'s flow default.
    assert!(read().is_none());

    let budget = Rc::new(WorkBudget::new(1_000_000_000).unwrap());
    let _work = LevelLineWork::begin(Some(budget));
    let _on = turn_run_outs_on();
    let by_tag = read().expect("two run-outs on the nose");
    let filed: usize = by_tag.values().map(Vec::len).sum();
    // Each crease's two triangles are filed once per distinct tag they carry.
    assert!((2..=4).contains(&filed), "{filed}");
    assert!(by_tag.keys().all(|tag| *tag == 1 || *tag == 2));
}

#[test]
fn rust_only_a_square_corner_has_no_pair_corner_end_face_and_gathers_nothing() {
    use geometry_processing::bevel::level_line::band_gap_extra_fields;
    use geometry_processing::bevel::level_line::PairCornerEndFaceField;
    use polygon_mesh::solids::platonic_solids::create_cube;

    let mut cube = create_cube(2.0, 2.0, 2.0);
    cube.clean_and_merge();
    let positions: Vec<Vector3> = cube
        .vertices
        .iter()
        .map(|&p| Vector3::from_vector3_float(p))
        .collect();
    let indices: Vec<(i32, i32, i32)> = cube.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let find = |p: Vector3| {
        positions
            .iter()
            .position(|&q| (q - p).length() < 1e-6)
            .unwrap() as i32
    };
    // Two top edges meeting at a corner, sharing the top face: their own faces are square to it,
    // so neither round runs out through the other's face.
    let corner = find(v(1.0, 1.0, 1.0));
    let chain = vec![find(v(-1.0, 1.0, 1.0)), corner, find(v(1.0, -1.0, 1.0))];
    let tags = vec![1; indices.len()];
    let all = |_: i32, _: i32| true;

    assert!(PairCornerEndFaceField::find(
        &positions,
        &indices,
        std::slice::from_ref(&chain),
        &[false],
        &all,
        0.5,
        &tags,
        None
    )
    .is_none());
    assert!(band_gap_extra_fields::gather(
        &positions,
        &indices,
        &[chain],
        &[false],
        &all,
        0.5,
        &tags,
        None
    )
    .is_none());
}

#[test]
fn rust_only_a_knife_prisms_foot_corner_has_a_pair_corner_end_face() {
    use geometry_processing::bevel::level_line::band_gap_extra_fields;
    use geometry_processing::bevel::level_line::PairCornerEndFaceField;
    use polygon_mesh::Mesh;

    // A prism along x whose section is a tall thin triangle: its sides lean at 14 degrees off
    // vertical, so the floor and a side meet sharper than square.
    let (a, b, c) = (v(0.0, -1.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 4.0));
    let along = v(6.0, 0.0, 0.0);
    let mut mesh = Mesh::new();
    mesh.create_face(&[a, c, b]);
    mesh.create_face(&[a + along, b + along, c + along]);
    mesh.create_face(&[a, b, b + along, a + along]);
    mesh.create_face(&[b, c, c + along, b + along]);
    mesh.create_face(&[c, a, a + along, c + along]);
    mesh.clean_and_merge();

    let positions: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|&p| Vector3::from_vector3_float(p))
        .collect();
    let indices: Vec<(i32, i32, i32)> = mesh.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let find = |p: Vector3| {
        positions
            .iter()
            .position(|&q| (q - p).length() < 1e-6)
            .unwrap() as i32
    };

    // The floor's x edge and the end cap's floor edge, meeting at the foot corner a.
    let chain = vec![find(a + along), find(a), find(b)];
    let tags = vec![1; indices.len()];
    let all = |_: i32, _: i32| true;

    let found = PairCornerEndFaceField::find(
        &positions,
        &indices,
        std::slice::from_ref(&chain),
        &[false],
        &all,
        0.5,
        &tags,
        None,
    )
    .expect("the x edge's round runs out through the end cap");
    assert_eq!(found.len(), 1);
    assert_eq!(
        found[&1].len(),
        1,
        "only the edge whose own face leans past the floor"
    );

    let gathered = band_gap_extra_fields::gather(
        &positions,
        &indices,
        std::slice::from_ref(&chain),
        &[false],
        &all,
        0.5,
        &tags,
        None,
    )
    .expect("gathered");
    assert_eq!(gathered[&1].len(), 1);

    // Off for the build in force: nothing.
    let budget = std::rc::Rc::new(
        geometry_processing::bevel::level_line::WorkBudget::new(1_000_000).unwrap(),
    );
    let _work = geometry_processing::bevel::level_line::LevelLineWork::begin(Some(budget));
    let _off = PairCornerEndFaceField::turn_off();
    assert!(PairCornerEndFaceField::find(
        &positions,
        &indices,
        &[chain],
        &[false],
        &all,
        0.5,
        &tags,
        None
    )
    .is_none());
}

#[test]
fn rust_only_a_chain_is_split_where_its_setback_steps_at_a_sharp_turn() {
    // Chain 0 -> 1 -> 4 turns square at 1. Edge 0-1 is a square fold (fillet setback 1); edge
    // 1-4 a 60 degree one (setback tan 30): a step of far more than 5%.
    let positions = [
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 0.0, -1.0),
        v(1.0, 1.0, 0.0),
        v(2.0, 0.0, -(3f64.sqrt())),
    ];
    let indices = [(0, 1, 4), (1, 0, 3), (4, 1, 5)];
    let chains = vec![vec![0, 1, 4]];

    let (split, closed) = chain_on_the_mesh::split_at_setback_steps(
        &positions,
        &indices,
        &chains,
        &[false],
        BandProfile::Fillet,
        45.0,
        None,
    )
    .unwrap();
    assert_eq!(split, vec![vec![0, 1], vec![1, 4]]);
    assert_eq!(closed, vec![false, false]);

    // A chamfer never steps; nor does a chain read with no turn threshold, or a turn gentler
    // than the threshold.
    for (profile, degrees) in [
        (BandProfile::Chamfer, 45.0),
        (BandProfile::Fillet, f64::INFINITY),
        (BandProfile::Fillet, 120.0),
    ] {
        let (same, _) = chain_on_the_mesh::split_at_setback_steps(
            &positions,
            &indices,
            &chains,
            &[false],
            profile,
            degrees,
            None,
        )
        .unwrap();
        assert_eq!(same, chains);
    }

    // No unpicked crease runs into vertex 1: no step.
    let none = |_: i32, _: i32| false;
    let (kept, _) = chain_on_the_mesh::split_at_setback_steps(
        &positions,
        &indices,
        &chains,
        &[false],
        BandProfile::Fillet,
        45.0,
        Some(&none),
    )
    .unwrap();
    assert_eq!(kept, chains);
}
