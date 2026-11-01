//! An open chain that ends free (every face round its end vertex one side's or the other's, the
//! picked edge running on as an unpicked crease): on the rolling ball's route the round closes onto
//! the chain's end vertex there, each side's band thinning to nothing at it
//! (`level_line::rolling_ball_free_ends`). Here on stretches of the one-wall-bending groove's ring
//! (`band_setback_tests.rs`): a long stretch whose sides differ, and a short one whose round is a
//! lens. The helpers that build such a pick and check its rails and fill are shared with the other
//! free-end and side-region tests.

use std::cell::Cell;
use std::collections::HashMap;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::open_gap_fill::{self, WatchFadeHole};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, ChainField, GapFillRefusal,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
};
use vector_math::Vector3;

use crate::band_route_tests::{creases, Triangles};
use crate::band_setback_tests::{
    a_ring_with_one_groove_wall_bending_steeper, the_contact_distances_in_the_one_sided_groove,
    RING_RADIUS, RING_SEGMENTS,
};

/// How far `point` stands from the open polyline through `chain`, and the arc length along it of
/// its nearest point.
pub fn from_the_open_chain(positions: &[Vector3], chain: &[i32], point: Vector3) -> (f64, f64) {
    let (mut nearest, mut along, mut walked) = (f64::INFINITY, 0.0, 0.0);
    for pair in chain.windows(2) {
        let (from, to) = (positions[pair[0] as usize], positions[pair[1] as usize]);
        let on = ChainField::closest_on_segment(point, from, to);
        let distance = (point - on).length();
        if distance < nearest {
            nearest = distance;
            along = walked + (on - from).length();
        }
        walked += (to - from).length();
    }
    (nearest, along)
}

/// One side's rail: its side tag and its points in order.
pub struct Rail {
    pub side: i32,
    pub points: Vec<Vector3>,
}

/// Builds `part` round the open `chain` at `radius` with the switch on and checks what a chain
/// free at both ends must show: the ball's route with nothing refused; no end region, one rail per
/// side; both rails running from one of the chain's end vertices to the other, sharing those two
/// mesh vertices; the fill closing the round with nothing refused, as two runs meeting end to end
/// (the one branch that hands its hole to the fade watcher), and the result closed (every directed
/// edge of a live triangle run once, and once back). Returns the build and its two rails.
pub fn closes_onto_both_end_vertices(
    (positions, triangles): &(Vec<Vector3>, Triangles),
    chain: &[i32],
    radius: f64,
) -> (BandGapResult, Vec<Rail>) {
    let ends = [
        positions[chain[0] as usize],
        positions[chain[chain.len() - 1] as usize],
    ];
    set_use_rolling_ball(true);
    set_last_route(None);
    let ball = build_pick(
        positions,
        triangles,
        &[chain.to_vec()],
        &[false],
        Some(creases(positions, triangles)),
        radius,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    let rails = the_rails_close_onto_both_end_vertices(&ball, ends);
    (ball, rails)
}

/// What [`closes_onto_both_end_vertices`] checks of a build, `ball`, round an open chain free at
/// both ends whose end vertices stand at `ends`: nothing refused, one rail per side running from
/// one end vertex to the other, the fill closing the round as two runs meeting end to end, the
/// result closed. Returns the two rails.
pub fn the_rails_close_onto_both_end_vertices(
    ball: &BandGapResult,
    ends: [Vector3; 2],
) -> Vec<Rail> {
    assert_eq!(ball.refusal, BandGapRefusal::None);

    let mut rails = Vec::new();
    let mut rail_ends: Vec<[i32; 2]> = Vec::new();
    {
        let mesh = ball
            .mesh
            .as_ref()
            .expect("the ball's band came out")
            .borrow();
        for gap_loop in &ball.loops {
            assert!(
                gap_loop.side == 1 || gap_loop.side == 2,
                "a free end has no end region: {}",
                gap_loop.side
            );
            let points: Vec<Vector3> = gap_loop
                .vertices
                .iter()
                .map(|&v| mesh.position(v))
                .collect();
            let (first, last) = (gap_loop.vertices[0], gap_loop.vertices[points.len() - 1]);
            for (end, vertex) in [(points[0], first), (points[points.len() - 1], last)] {
                assert!(
                    ends.iter().any(|e| (*e - end).length() < 1e-9),
                    "side {}'s rail ends at {end:?} (vertex {vertex}), not at a chain end",
                    gap_loop.side
                );
            }
            rail_ends.push([first, last]);
            rails.push(Rail {
                side: gap_loop.side,
                points,
            });
        }
    }
    assert_eq!(rail_ends.len(), 2, "one rail per side");
    let (a, b) = (rail_ends[0], rail_ends[1]);
    assert!(
        (a[0] == b[1] && a[1] == b[0]) || (a[0] == b[0] && a[1] == b[1]),
        "the rails end at {a:?} and {b:?}, not at shared vertices"
    );

    let fades = Rc::new(Cell::new(0));
    let seen = Rc::clone(&fades);
    let watch: WatchFadeHole = Rc::new(move |_hole: &[(i32, i32)]| seen.set(seen.get() + 1));
    open_gap_fill::WATCH_FADE_HOLE.with(|w| *w.borrow_mut() = Some(watch));
    let fill = gap_fill::fill(ball, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS);
    open_gap_fill::WATCH_FADE_HOLE.with(|w| *w.borrow_mut() = None);
    let fill = fill.expect("the fill runs");
    assert_eq!(fades.get(), 1, "the runs meet end to end at both ends");
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);
    {
        let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
        let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
        for t in 0..mesh.triangle_count() {
            if mesh.is_removed(t) {
                continue;
            }
            let face = mesh.triangles()[t as usize];
            for (a, b) in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
                *directed.entry((a, b)).or_default() += 1;
            }
        }
        for (&(a, b), &count) in &directed {
            assert_eq!(count, 1, "edge {a}-{b} is run {count} times one way");
            assert_eq!(
                directed.get(&(b, a)),
                Some(&1),
                "edge {a}-{b} has no triangle running it back: the mesh is open"
            );
        }
    }
    rails
}

/// Twice the ring facets' sagitta at its outer radius, as the closed ring's test reasons: how far
/// the faceted ring may move a contact, and the rail at its distance, from the smooth ring's.
fn ring_tolerance() -> f64 {
    2.0 * (RING_RADIUS + 5.0) * (1.0 - (std::f64::consts::PI / f64::from(RING_SEGMENTS)).cos())
}

/// Radius 2.3 (no power of two, so a setback per unit of size read back at the size rounds) round
/// a 31-facet stretch of the ring's floor, free at both ends. The straight wall's contact stands
/// 2.69 from the floor and the steeper wall's 3.13 (by hand, as the ring's closed chain). Clear of
/// both ends by the ball's setback and a radius, each side's rail runs at its own hand distance;
/// no rail is ever wider than its side's; and the round closes onto both end vertices.
#[test]
fn a_free_end_whose_sides_differ_closes_the_round_onto_the_end_vertex() {
    let radius = 2.3;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);
    let tolerance = ring_tolerance();
    assert!(tolerance < 0.01 && steeper - straight > 0.3, "{tolerance}");

    let (positions, triangles, ring) = a_ring_with_one_groove_wall_bending_steeper();
    let chain = ring[..32].to_vec();
    let length: f64 = chain
        .windows(2)
        .map(|pair| (positions[pair[1] as usize] - positions[pair[0] as usize]).length())
        .sum();
    let part = (positions, triangles);
    let (_, rails) = closes_onto_both_end_vertices(&part, &chain, radius);
    for rail in &rails {
        let outside = |p: &Vector3| p.x.hypot(p.y) > RING_RADIUS;
        let on_the_steeper_wall = outside(&rail.points[rail.points.len() / 2]);
        let expected = if on_the_steeper_wall {
            steeper
        } else {
            straight
        };
        let mut in_the_middle = 0;
        for p in &rail.points {
            let (from_the_floor, along) = from_the_open_chain(&part.0, &chain, *p);
            if from_the_floor > tolerance {
                assert_eq!(outside(p), on_the_steeper_wall, "one side's rail: {p:?}");
            }
            assert!(
                from_the_floor < expected + tolerance,
                "side {} rail at {p:?} stands {from_the_floor} from the floor, past {expected}",
                rail.side
            );
            if along > expected + radius && along < length - expected - radius {
                in_the_middle += 1;
                assert!(
                    (from_the_floor - expected).abs() < tolerance,
                    "side {} rail at {p:?} stands {from_the_floor} from the floor, not {expected}",
                    rail.side
                );
            }
        }
        assert!(in_the_middle > 10, "side {}: {in_the_middle}", rail.side);
    }
}

/// Radius 2 round a 7-facet stretch of the ring's floor (3.44 long), free at both ends: shorter
/// than twice either side's setback (2.34 and 2.72), so both ends' tapers bind all along and the
/// round is a lens, thinning to nothing at each end vertex. By hand: each chain vertex's setback
/// is its distance from the nearer end vertex, read linearly between vertices, so the lens is
/// widest along the middle edge, whose two vertices both stand three facets' chord from their
/// nearer end; a rail there stands that far from the floor (within the facets' sagitta, by which
/// the faceted chain bends from the segment's line).
#[test]
fn a_free_chain_shorter_than_twice_its_setback_gets_a_lens_shaped_round() {
    let radius = 2.0;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);
    let (positions, triangles, ring) = a_ring_with_one_groove_wall_bending_steeper();
    let chain = ring[..8].to_vec();
    let middle = (positions[chain[3] as usize] - positions[chain[0] as usize]).length();
    let mirrored = (positions[chain[4] as usize] - positions[chain[7] as usize]).length();
    assert!(
        (middle - mirrored).abs() < 1e-9 && middle < straight,
        "{middle} {mirrored}"
    );
    let part = (positions, triangles);
    let (_, rails) = closes_onto_both_end_vertices(&part, &chain, radius);
    let tolerance = ring_tolerance();
    for rail in &rails {
        let mut widest: f64 = 0.0;
        for p in &rail.points {
            let (from_the_floor, _) = from_the_open_chain(&part.0, &chain, *p);
            assert!(
                from_the_floor < straight.min(steeper) + tolerance,
                "side {} rail at {p:?} stands {from_the_floor}, wider than the ball",
                rail.side
            );
            widest = widest.max(from_the_floor);
        }
        assert!(
            (widest - middle).abs() < tolerance,
            "side {}'s widest rail point stands {widest} from the floor, not {middle}",
            rail.side
        );
    }
}
