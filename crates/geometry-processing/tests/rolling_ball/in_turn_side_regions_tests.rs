//! The caller's face regions handed to a pick built a chain at a time
//! (`band_gap_in_turn::build_open_chains_in_turn_with_side_regions`): the first chain reads them
//! on the input mesh, the later ones read them carried onto the mesh the earlier chains left
//! (`side_regions_after_a_build`). Held to a cube whose left face blends smoothly into its
//! neighbours, so the kernel's own flood joins each picked edge's two sides round it while the
//! regions keep them apart.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn_with_side_regions, feature_edges_after_a_build,
};
use geometry_processing::bevel::level_line::band_gap_rounds_first::kept;
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::side_regions_after_a_build::carried;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandProfile, GapFillRefusal, GapFillResult,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use super::band_route_tests::{a_box, creases, Triangles};

/// The fillet's radius.
const RADIUS: f64 = 0.25;

/// The cube from (-1, -1, -1) to (1, 1, 1), its faces' two triangles each one region (bottom 0,
/// top 1, front 2, right 3, back 4, left 5, the order `a_box` makes them in).
fn the_cube_and_its_faces() -> ((Vec<Vector3>, Triangles), Vec<u32>) {
    let part = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let faces = (0..part.1.len() as u32).map(|t| t / 2).collect();
    (part, faces)
}

/// The cube's creases but the four round its left face (x = -1, corners 0, 3, 7 and 4), as a part
/// whose left end blends smoothly into the faces round it reads them: a flood from any face
/// beside the left face runs over it into the others.
fn the_creases_but_the_left_faces(
    part: &(Vec<Vector3>, Triangles),
) -> Rc<dyn Fn(i32, i32) -> bool> {
    let sharp = creases(&part.0, &part.1);
    Rc::new(move |a, b| {
        let (low, high) = if a < b { (a, b) } else { (b, a) };
        sharp(a, b) && !matches!((low, high), (0, 3) | (3, 7) | (4, 7) | (0, 4))
    })
}

/// The top front edge (4 to 5, y = -1, z = 1) and the bottom back edge (3 to 2, y = 1, z = -1):
/// opposite edges, so neither band reaches the other.
fn the_two_edges() -> Vec<Vec<i32>> {
    vec![vec![4, 5], vec![3, 2]]
}

/// `chains` filleted at [`RADIUS`] a chain at a time with the switch on, `side_regions` as given;
/// the result and the route the last chain's build wrote.
fn built_in_turn(
    chains: &[Vec<i32>],
    side_regions: Option<&[u32]>,
) -> (GapFillResult, Option<String>) {
    let (part, _) = the_cube_and_its_faces();
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_open_chains_in_turn_with_side_regions(
        &part.0,
        &part.1,
        chains,
        Some(the_creases_but_the_left_faces(&part)),
        RADIUS,
        BandProfile::Fillet,
        DEFAULT_FILLET_SEGMENTS,
        None,
        1e-9,
        0.0,
        f64::INFINITY,
        None,
        None,
        side_regions,
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// True when every directed edge of the mesh's live triangles is run once and once back.
fn is_closed(fill: &GapFillResult) -> bool {
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for (a, b, c) in kept(&mesh) {
        for edge in [(a, b), (b, c), (c, a)] {
            *directed.entry(edge).or_default() += 1;
        }
    }
    directed
        .iter()
        .all(|(&(a, b), &count)| count == 1 && directed.get(&(b, a)) == Some(&1))
}

/// By hand: between two flat faces at a right angle the ball of radius r touches each face r
/// from the edge (its centre r off both faces), so each rail stands r from its edge and the round
/// between them is the quarter circle of radius r about the line r in from both faces. Every vertex
/// the build added clear of the end planes (|x| < 1) lies on one of the two rounds: the top front
/// edge's about (y, z) = (-1 + r, 1 - r), the bottom back edge's about (1 - r, -1 + r), within the
/// band's root precision. Both rails of each round are found exactly r from their edge.
fn the_rounds_are_the_balls(fill: &GapFillResult) {
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let near = 1e-6;
    let axes = [(-1.0 + RADIUS, 1.0 - RADIUS), (1.0 - RADIUS, -1.0 + RADIUS)];
    let mut rails = [[0; 2]; 2];
    for v in 8..mesh.vertex_count() {
        let p = mesh.position(v);
        if p.x.abs() > 1.0 - 1e-9 {
            continue;
        }
        let round = if p.y < 0.0 { 0 } else { 1 };
        let (cy, cz) = axes[round];
        let off = ((p.y - cy) * (p.y - cy) + (p.z - cz) * (p.z - cz)).sqrt();
        assert!(
            (off - RADIUS).abs() < near,
            "{p:?} stands {off} off round {round}'s axis, not {RADIUS}"
        );
        // On the face across the edge's z (top or bottom) and on the face across its y.
        if (p.z.abs() - 1.0).abs() < 1e-9 && (p.y.abs() - (1.0 - RADIUS)).abs() < near {
            rails[round][0] += 1;
        }
        if (p.y.abs() - 1.0).abs() < 1e-9 && (p.z.abs() - (1.0 - RADIUS)).abs() < near {
            rails[round][1] += 1;
        }
    }
    for (round, sides) in rails.iter().enumerate() {
        assert!(
            sides[0] > 0 && sides[1] > 0,
            "round {round}'s rails: {sides:?} vertices r from the edge on each face"
        );
    }
}

/// The first chain, built on the input mesh, takes the ball from the regions where the kernel's
/// own flood joins its sides round the left face; the second chain, built on the mesh the first
/// left, takes it too from the regions carried onto that mesh, where its own flood joins its sides
/// the same way. Each round is the ball's by hand, and the result is closed.
#[test]
fn a_pick_built_a_chain_at_a_time_hands_every_chain_its_side_regions() {
    let (_, faces) = the_cube_and_its_faces();

    let (alone, route) = built_in_turn(&the_two_edges()[..1], Some(&faces));
    assert_eq!(
        route.as_deref(),
        Some(ROLLING_BALL_ROUTE),
        "the first chain"
    );
    assert_eq!(alone.refusal, GapFillRefusal::None);

    let (both, route) = built_in_turn(&the_two_edges(), Some(&faces));
    assert_eq!(
        route.as_deref(),
        Some(ROLLING_BALL_ROUTE),
        "the second chain"
    );
    assert_eq!(both.refusal, GapFillRefusal::None);
    assert!(is_closed(&both), "the mesh is closed");
    the_rounds_are_the_balls(&both);
}

/// Without the regions the first chain's sides flood into one round the blended left face, so
/// the ball has no two sides to roll between: with the switch on the pass is the ball's or
/// refused, so the pick is refused at that chain with the reason, which the result carries. This
/// is what the regions change.
#[test]
fn without_side_regions_the_first_chain_floods_its_sides_into_one() {
    let (both, route) = built_in_turn(&the_two_edges(), None);
    assert_eq!(
        route.as_deref(),
        Some("refused: chain 0: its two sides are one surface")
    );
    assert_eq!(both.refusal, GapFillRefusal::OpenChainEnds);
    assert_eq!(both.offending_chain, 0);
    assert_eq!(
        both.rolling_ball_refusal.as_deref(),
        Some("chain 0: its two sides are one surface")
    );
}

/// The regions carried onto the mesh the top front edge's fillet left: every triangle on a cube
/// face (its centroid in that face's plane) keeps that face's region, whether the build left it
/// whole or cut it, and every other triangle (the round, off every face) is region 6, one above
/// the cube's, which is no side's. On that mesh the bottom back edge's own flood joins its sides
/// round the left face (with the switch on its pass is refused, with that reason), and the carried
/// regions give it two, so the ball takes it.
#[test]
fn regions_carried_past_a_build_keep_each_face_and_give_the_round_its_own() {
    let ((positions, triangles), faces) = the_cube_and_its_faces();
    set_use_rolling_ball(true);
    let gap = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[vec![4, 5]],
        &[false],
        Some(the_creases_but_the_left_faces(&(
            positions.clone(),
            triangles.clone(),
        ))),
        RADIUS,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        Some(&faces),
    )
    .expect("the pick's arguments are good");
    reset_to_the_default();
    let fill = gap_fill::fill(&gap, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("fills");
    assert_eq!(fill.refusal, GapFillRefusal::None);
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let after = kept(&mesh);
    let labels = carried(
        (&positions, &triangles),
        &faces,
        &mesh,
        &fill.added_triangles,
        None,
    )
    .expect("nothing cancels the build")
    .expect("a region is free for the round");
    assert_eq!(labels.len(), after.len());

    // A face's plane by its region: (axis, side).
    let planes = [
        (2, -1.0),
        (2, 1.0),
        (1, -1.0),
        (0, 1.0),
        (1, 1.0),
        (0, -1.0),
    ];
    let (mut cut, mut round) = (0, 0);
    for (&(a, b, c), &label) in after.iter().zip(&labels) {
        let centroid = (mesh.position(a) + mesh.position(b) + mesh.position(c)) / 3.0;
        let on = planes.iter().position(|&(axis, side)| {
            let coordinate = [centroid.x, centroid.y, centroid.z][axis];
            (coordinate - side).abs() < 1e-12
        });
        match on {
            Some(face) => assert_eq!(label, face as u32, "a triangle on face {face}"),
            None => {
                assert_eq!(label, 6, "a triangle of the round at {centroid:?}");
                round += 1;
            }
        }
        if a >= 8 || b >= 8 || c >= 8 {
            cut += 1;
        }
    }
    assert!(
        round > 0 && cut > round,
        "{round} round triangles, {cut} with a new corner"
    );

    // The bottom back edge on that mesh, read as the build a chain at a time reads it: its own
    // flood joins its sides round the left face, the carried regions keep them apart.
    let input_edges: HashSet<(i32, i32)> = triangles
        .iter()
        .flat_map(|&(a, b, c)| [(a, b), (b, c), (c, a)])
        .map(|(a, b)| if a < b { (a, b) } else { (b, a) })
        .collect();
    let second = |side_regions: Option<&[u32]>| {
        set_use_rolling_ball(true);
        set_last_route(None);
        let gap = build_pick_with_side_regions(
            &mesh.vertices,
            &after,
            &[vec![3, 2]],
            &[false],
            Some(feature_edges_after_a_build(
                mesh.vertices.clone(),
                after.clone(),
                positions.len() as i32,
                input_edges.clone(),
                the_creases_but_the_left_faces(&(positions.clone(), triangles.clone())),
            )),
            RADIUS,
            BandProfile::Fillet,
            &BandGapBuildOptions::default(),
            side_regions,
        )
        .expect("the pick's arguments are good");
        let route = last_route();
        reset_to_the_default();
        (gap, route)
    };
    let (ball, route) = second(Some(&labels));
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    let (flooded, route) = second(None);
    assert_eq!(
        route.as_deref(),
        Some("refused: chain 0: its two sides are one surface")
    );
    assert_eq!(flooded.refusal, BandGapRefusal::RollingBall);
    assert_eq!(
        flooded.rolling_ball_refusal.as_deref(),
        Some("chain 0: its two sides are one surface")
    );
}
