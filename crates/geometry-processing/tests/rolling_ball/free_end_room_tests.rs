//! The room a pass of one chain is checked against, on the rolling ball's route
//! (`level_line::one_chain_room`): read with the ball's setbacks, so a free end whose round closes
//! onto its end vertex is not refused by a crease the level line's round, run one setback past the
//! end, would reach. Held to a plain 90 degree V-groove on a thin floor, picked free at both ends 2
//! short of the groove's square ends, whose bottom crease stands 0.5 below the floor's line: the
//! level line's room is the distance from the end vertex to that crease, the ball's the distance
//! from the chain vertex nearest a full setback back from the end. And a room the caller hands in
//! (read, as the model reads it, with the angle's setbacks) is set aside wherever the ball may take
//! a pass of one chain, open or closed: held on the one-wall-bending groove's ring, whose steeper
//! wall's top crease the ball's band reaches before the angle's does. And each side's creases, and
//! how far a side runs before it is swallowed, are read against that side's own band: held on the
//! ring with its straight wall creasing between the two sides' contact distances.

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, GapFillRefusal};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
};
use std::collections::HashMap;

use vector_math::Vector3;

use crate::band_route_tests::{built, creases, Triangles};
use crate::band_setback_tests::{
    a_ring_with_one_groove_wall_bending_steeper, the_contact_distances_in_the_one_sided_groove,
    RING_RADIUS, RING_SEGMENTS, STEEP_DEGREES,
};
use crate::free_end_taper_tests::sliced;
use crate::free_end_tests::closes_onto_both_end_vertices;

/// The groove's floor depth below its line: the end faces' bottom crease runs 0.5 under the floor.
const FLOOR: f64 = 0.5;

/// How far each free end stands short of its square end: the picked stretch runs from slice 8 to
/// slice 56 of 64, y = 2 to 14 of 16.
const SHORT: f64 = 2.0;

/// The radius asked: above the level line's room, well under the ball's.
const RADIUS: f64 = 2.3;

/// The chain's edge length (16 / 64).
const EDGE: f64 = 0.25;

/// The plain 90 degree V-groove, its walls 4 high, on a floor [`FLOOR`] thick, sliced, picked from
/// y = 2 to y = 14, free at both ends.
fn the_groove_on_a_thin_floor() -> ((Vec<Vector3>, Triangles), Vec<i32>) {
    sliced(
        &[
            (0.0, 0.0),
            (4.0, 4.0),
            (6.0, 4.0),
            (6.0, -FLOOR),
            (-6.0, -FLOOR),
            (-6.0, 4.0),
            (-4.0, 4.0),
        ],
        (0.0, -FLOOR / 2.0),
        (8, 56),
    )
}

/// The level line's room by hand. Its round runs one setback (the size, at 90 degrees) past the
/// free end, so it reaches the square end's bottom crease, nearest at (0, 0, -0.5), at the size
/// equal to that crease's distance from the end vertex: sqrt(2^2 + 0.5^2).
fn the_level_lines_room() -> f64 {
    SHORT.hypot(FLOOR)
}

/// The ball's room by hand, read at [`RADIUS`]. Its setback at a chain vertex t from the free end is
/// t, capped at the radius (the round closes onto the end vertex as the sphere of radius S
/// through it), so per unit of size t / S under a full setback. Scaled to a size s it reaches the
/// bottom crease from that vertex at s min(t, S) / S = sqrt((2 + t)^2 + 0.5^2): the room is the
/// least such s over the chain's vertices (t = 0.25 k), which is t = 2.25, the last vertex under a
/// full setback: sqrt(4.25^2 + 0.25) 2.3 / 2.25 = 4.3744.
fn the_balls_room() -> f64 {
    (1..=56 - 8)
        .map(|k| {
            let t = EDGE * f64::from(k);
            let reach = t.min(RADIUS) / RADIUS;
            (SHORT + t).hypot(FLOOR) / reach
        })
        .fold(f64::INFINITY, f64::min)
}

/// With the switch off the level line's round reaches the crease past the free end and the size is
/// refused, naming the level line's room; with it on the room is the ball's, the size fits, the
/// pass takes the ball, both rails end on the end vertices and the filled mesh is closed.
#[test]
fn a_free_end_short_of_a_crease_is_refused_by_the_level_line_and_built_by_the_ball() {
    let (part, chain) = the_groove_on_a_thin_floor();

    let (level_line, _) = built(&part, std::slice::from_ref(&chain), RADIUS, false);
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert!(
        (level_line.room - the_level_lines_room()).abs() < 1e-6,
        "the level line's room {}, by hand {}",
        level_line.room,
        the_level_lines_room()
    );

    let (ball, route) = built(&part, std::slice::from_ref(&chain), RADIUS, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert!(
        (ball.room - the_balls_room()).abs() < 1e-6,
        "the ball's room {}, by hand {}",
        ball.room,
        the_balls_room()
    );
    // Rails on the end vertices, the fill closed and the mesh closed.
    closes_onto_both_end_vertices(&part, &chain, RADIUS);
}

/// One pick built with the room `handed` handed in, as the model hands the room it read before
/// the build, with the rolling ball's switch `on`: the result and the route.
fn built_on_a_handed_room(
    (positions, triangles): &(Vec<Vector3>, Triangles),
    chain: &[i32],
    closed: bool,
    size: f64,
    handed: f64,
    on: bool,
) -> (
    geometry_processing::bevel::level_line::BandGapResult,
    Option<String>,
) {
    set_use_rolling_ball(on);
    set_last_route(None);
    let result = build_pick(
        positions,
        triangles,
        &[chain.to_vec()],
        &[closed],
        Some(creases(positions, triangles)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            known_room: Some(handed),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// How far the ring's steeper wall's top crease stands from the groove's floor in the profile:
/// the wall runs from (0, 0) to (1, 1), then 4 at [`STEEP_DEGREES`].
fn the_steeper_walls_top() -> f64 {
    let steep = STEEP_DEGREES.to_radians();
    (1.0 + (4.0 * steep.cos())).hypot(1.0 + (4.0 * steep.sin()))
}

/// The groove handed the level line's room, which refuses the size: with the switch off the build
/// refuses it, naming that room; with it on the handed room is set aside, the build reads the
/// ball's own, which fits, and builds.
#[test]
fn a_handed_room_that_refuses_the_size_is_set_aside_for_the_balls_own() {
    let (part, chain) = the_groove_on_a_thin_floor();
    let handed = the_level_lines_room();

    let (level_line, _) = built_on_a_handed_room(&part, &chain, false, RADIUS, handed, false);
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert_eq!(level_line.room, handed, "the handed room is quoted");

    let (ball, route) = built_on_a_handed_room(&part, &chain, false, RADIUS, handed, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert!(
        (ball.room - the_balls_room()).abs() < 1e-6,
        "the ball's room {}, by hand {}",
        ball.room,
        the_balls_room()
    );
}

/// The ring at 3.9, handed the level line's room (the steeper wall's top crease 5.35 from the
/// floor, at a setback of the size: 5.35), which clears the size. By hand the ball's contact on the
/// steeper wall would stand 5.42 out on a wall that went on, past the wall's top at 5.35: the ball
/// rests on that crease, so its band reaches it at exactly the size and the room is the size less
/// the kept strip's margin. With the switch off the handed room is trusted and nothing is refused
/// by size; with it on the size is refused, naming the ball's room.
#[test]
fn a_handed_room_that_clears_the_size_is_not_trusted_where_the_balls_band_reaches_past_it() {
    let size = 3.9;
    let (_, steeper) = the_contact_distances_in_the_one_sided_groove(size);
    assert!(steeper > the_steeper_walls_top(), "{steeper}");
    let (positions, triangles, chain) = a_ring_with_one_groove_wall_bending_steeper();
    let part = (positions, triangles);
    let (unhanded, _) = built(&part, std::slice::from_ref(&chain), size, false);
    let handed = unhanded.room;
    assert!(
        (handed - the_steeper_walls_top()).abs() < 0.01 && handed > size,
        "the level line's room {handed}"
    );

    let (level_line, _) = built_on_a_handed_room(&part, &chain, true, size, handed, false);
    assert_ne!(level_line.refusal, BandGapRefusal::Size);

    let (ball, route) = built_on_a_handed_room(&part, &chain, true, size, handed, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::Size);
    assert!(
        ball.room < size && ball.room > size - 1e-6,
        "the ball's room {}",
        ball.room
    );
}

/// A stretch of the ring a quarter round (64 facets), free at both ends, at 3.9 and handed a room
/// of 2 that refuses the size, as the model hands one: the ball's band reaches the steeper wall's
/// top crease along the stretch's middle, so with the switch on the size is refused and the
/// refusal names the ball's room (the size less the margin, as on the whole ring), not the handed
/// one. (With the switch off the level line's band flows past the handed room on this open chain
/// and builds; that is the level line's own business.)
#[test]
fn a_ball_refusal_on_a_handed_room_names_the_balls_room() {
    let size = 3.9;
    let handed = 2.0;
    let (positions, triangles, ring) = a_ring_with_one_groove_wall_bending_steeper();
    let chain = ring[..64].to_vec();
    let part = (positions, triangles);

    let (ball, route) = built_on_a_handed_room(&part, &chain, false, size, handed, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::Size);
    assert!(
        ball.room < size && ball.room > size - 1e-6,
        "the ball's room {}",
        ball.room
    );
}

/// The whole ring, a closed chain, at radius 2 handed a room of 1: with the switch off it is
/// refused up front on that room; with it on the build reaches the ball, reads its room (the
/// steeper wall's top over the steeper side's setback per unit of size, 5.35 x 2 / 2.72, within the
/// facets' sagitta) and builds.
#[test]
fn a_closed_chain_handed_a_refusing_room_still_reaches_the_ball() {
    let size = 2.0;
    let handed = 1.0;
    let (_, steeper) = the_contact_distances_in_the_one_sided_groove(size);
    let by_hand = the_steeper_walls_top() * size / steeper;
    let (positions, triangles, chain) = a_ring_with_one_groove_wall_bending_steeper();
    let part = (positions, triangles);

    let (level_line, _) = built_on_a_handed_room(&part, &chain, true, size, handed, false);
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert_eq!(level_line.room, handed);

    let (ball, route) = built_on_a_handed_room(&part, &chain, true, size, handed, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert!(
        (ball.room - by_hand).abs() < 0.02,
        "the ball's room {}, by hand {by_hand}",
        ball.room
    );
}

/// How far along its 45 degree line the straight wall of [`a_ring_whose_straight_wall_creases`]
/// runs from the floor before it creases onto the ring's top.
const STRAIGHT_WALL: f64 = 2.7;

/// The radius asked round [`a_ring_whose_straight_wall_creases`].
const SIDE_ROOM_RADIUS: f64 = 2.0;

/// The one-wall-bending groove's ring (`a_ring_with_one_groove_wall_bending_steeper`, revolved
/// the same way) with its straight wall stopping [`STRAIGHT_WALL`] from the floor at a
/// convex crease onto a flat top; the steeper wall and its top 5.35 out as before. A closed ring,
/// its floor the closed chain.
fn a_ring_whose_straight_wall_creases() -> (Vec<Vector3>, Triangles, Vec<i32>) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let top = STRAIGHT_WALL * std::f64::consts::FRAC_1_SQRT_2;
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, top),
        (-top, top),
    ];
    let n = profile.len() as i32;
    let mut positions = Vec::new();
    for j in 0..RING_SEGMENTS {
        let angle = std::f64::consts::TAU * f64::from(j) / f64::from(RING_SEGMENTS);
        for &(x, z) in &profile {
            let rho = RING_RADIUS + x;
            positions.push(Vector3::new(rho * angle.cos(), rho * angle.sin(), z));
        }
    }
    let mut triangles: Triangles = Vec::new();
    for j in 0..RING_SEGMENTS {
        let next = (j + 1) % RING_SEGMENTS;
        for i in 0..n {
            let k = (i + 1) % n;
            let (a, b, c, d) = (j * n + i, j * n + k, next * n + k, next * n + i);
            triangles.push((a, b, c));
            triangles.push((a, c, d));
        }
    }
    let mut volume = 0.0;
    for &(a, b, c) in &triangles {
        let (pa, pb, pc) = (
            positions[a as usize],
            positions[b as usize],
            positions[c as usize],
        );
        volume += pa.dot(pb.cross(pc));
    }
    if volume < 0.0 {
        triangles = triangles.iter().map(|&(a, b, c)| (a, c, b)).collect();
    }
    let chain = (0..RING_SEGMENTS).map(|j| j * n).collect();
    (positions, triangles, chain)
}

/// A narrow side with a crease near the chain beside a wide side with none: radius 2 round the
/// ring whose straight wall creases [`STRAIGHT_WALL`] = 2.7 out. By hand the ball touches the
/// straight wall 2.34 from the floor (short of its crease) and the steeper wall 2.72 out. Read
/// against the larger side, the crease (2.7 from the chain all round) would be reached at
/// 2.7 x 2 / 2.72 = 1.98, under the size asked, and the straight wall, all of its side, would be
/// swallowed at that size too; read against its own side, the straight wall's, at
/// 2.7 x 2 / 2.34 = 2.31, above it. The steeper side's nearest crease, its top, is reached later
/// still. So the room is the straight side's, the size builds on the ball and the filled mesh is
/// closed. The facets move a contact by at most twice their sagitta (under 0.01), and the room
/// with it, as the closed ring's room test reasons.
#[test]
fn a_crease_on_the_narrow_side_is_read_against_that_sides_own_band() {
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(SIDE_ROOM_RADIUS);
    assert!(straight < STRAIGHT_WALL && STRAIGHT_WALL < steeper);
    let by_the_larger_side = STRAIGHT_WALL * SIDE_ROOM_RADIUS / steeper;
    let by_its_own_side = STRAIGHT_WALL * SIDE_ROOM_RADIUS / straight;
    assert!(by_the_larger_side < SIDE_ROOM_RADIUS && SIDE_ROOM_RADIUS < by_its_own_side);
    assert!(the_steeper_walls_top() * SIDE_ROOM_RADIUS / steeper > by_its_own_side);

    let (positions, triangles, chain) = a_ring_whose_straight_wall_creases();
    set_use_rolling_ball(true);
    set_last_route(None);
    let ball = build_pick(
        &positions,
        &triangles,
        std::slice::from_ref(&chain),
        &[true],
        Some(creases(&positions, &triangles)),
        SIDE_ROOM_RADIUS,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None, "room {}", ball.room);
    assert!(
        (ball.room - by_its_own_side).abs() < 0.02,
        "the ball's room {}, by hand {by_its_own_side} (read against the larger side \
         {by_the_larger_side})",
        ball.room
    );

    let fill =
        gap_fill::fill(&ball, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);
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
