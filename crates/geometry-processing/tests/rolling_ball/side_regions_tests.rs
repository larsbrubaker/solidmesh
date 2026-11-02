//! The rolling ball's two sides read from the caller's face regions
//! (`band_gap_entry_points::build_pick_with_side_regions`) rather than flooded by the kernel over
//! the build's feature edges: a cube whose back the feature edges let one side flood round into the
//! other, which the regions keep apart and the kernel's flood divides where the floods meet (the
//! band's own floods join the two sides there, so the band is tagged from the ball's sides), regions that do not give a chain two sides (refused by the
//! ball, strict), a chain whose band's own side floods join two sides that differ, whose band is
//! tagged from the ball's sides and cut per side (and which the switch off leaves as it was), whose
//! crease running on past each free end the tapered round never reaches, and a box whose two side
//! regions meet again away from the chain, where the room is read, and a step whose tread, in the
//! top's region, wraps round behind the riser the ball rests on, which the band never cuts and
//! whose riser|tread rim is the riser's rail, its round drawn as the ball's own arc.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, LevelLineMesh, RoomReadingKind,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_REFUSED,
    ROLLING_BALL_ROUTE,
};
use vector_math::{ieee_math, Vector3};

use super::band_route_tests::{a_box, creases, Triangles};
use super::band_setback_tests::{the_contact_distances_in_the_one_sided_groove, STEEP_DEGREES};
use super::free_end_taper_tests::{on_the_sphere_through_the_end, sliced};
use super::free_end_tests::the_rails_close_onto_both_end_vertices;

/// The cube from (-1, -1, -1) to (1, 1, 1), its faces' two triangles each numbered as one region
/// (bottom 0, top 1, front 2, right 3, back 4, left 5: `a_box` makes two triangles per face in
/// that order).
fn a_cube_and_its_faces() -> ((Vec<Vector3>, Triangles), Vec<u32>) {
    let part = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let faces = (0..part.1.len() as u32).map(|t| t / 2).collect();
    (part, faces)
}

/// The cube's top front edge (4 to 5) filleted at 0.25 with the rolling ball's switch on, its
/// feature edges every crease but the three round the back (top to back, back to bottom, bottom to
/// front), as a part whose top blends smoothly round its back into its front reads them: a flood
/// from the top reaches the front without crossing the chain. `side_regions` as given.
fn the_top_front_edge(side_regions: Option<&[u32]>) -> (BandGapResult, Option<String>) {
    let ((positions, triangles), _) = a_cube_and_its_faces();
    let sharp = creases(&positions, &triangles);
    let blended = |a: i32, b: i32| {
        let (low, high) = if a < b { (a, b) } else { (b, a) };
        matches!((low, high), (6, 7) | (2, 3) | (0, 1))
    };
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[vec![4, 5]],
        &[false],
        Some(Rc::new(move |a, b| sharp(a, b) && !blended(a, b))),
        0.25,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        side_regions,
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// A side that wraps round the part into the other far from the chain takes the ball with or
/// without the caller's regions: the kernel's flood runs round the back from the top into the
/// front, and without the regions that one surface is divided where the floods meet (round the
/// back); with them the sides are the top and the front. Either way the ball rolls in a square
/// corner with its contacts the radius from the edge. That the division keeps a finely cut side
/// whole is held in `fine_front_sides_tests`.
#[test]
fn a_side_wrapping_round_into_the_other_takes_the_ball_with_or_without_regions() {
    let (flooded, flooded_route) = the_top_front_edge(None);
    assert_eq!(flooded_route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(flooded.refusal, BandGapRefusal::None);
    for setback in &flooded.chains[0].setbacks {
        assert!((setback - 0.25).abs() < 1e-8, "{setback}");
    }

    let (_, faces) = a_cube_and_its_faces();
    let (ball, route) = the_top_front_edge(Some(&faces));
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    for setback in &ball.chains[0].setbacks {
        assert!((setback - 0.25).abs() < 1e-8, "{setback}");
    }
}

/// Regions that put the top and the front in one region give the chain no two sides: the ball
/// refuses the pass and says so, whatever the feature edges would have flooded.
#[test]
fn face_regions_that_join_a_chains_two_sides_are_refused_by_the_ball() {
    let (_, mut faces) = a_cube_and_its_faces();
    for region in &mut faces {
        if *region == 2 {
            *region = 1;
        }
    }
    let (result, route) = the_top_front_edge(Some(&faces));
    let why = "chain 0: its two sides are one face region (1)";
    assert_eq!(route, Some(format!("{ROLLING_BALL_REFUSED}{why}")));
    assert_eq!(result.refusal, BandGapRefusal::RollingBall);
    assert_eq!(result.rolling_ball_refusal.as_deref(), Some(why));
}

/// A labelling of another mesh (one triangle short) names no sides at all: the ball refuses the
/// pass with its reason.
#[test]
fn face_regions_that_miss_a_triangle_are_refused_by_the_ball() {
    let ((positions, triangles), faces) = a_cube_and_its_faces();
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[vec![4, 5]],
        &[false],
        Some(creases(&positions, &triangles)),
        0.25,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        Some(&faces[..faces.len() - 1]),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    assert_eq!(
        route.as_deref(),
        Some("refused: the side regions label 11 triangles where the mesh has 12")
    );
    assert_eq!(result.refusal, BandGapRefusal::RollingBall);
}

/// The one-wall-bending groove sliced along its length, picked from y = 2 to y = 14, free at both
/// ends, built at radius 2 with the switch on. Its feature edges are its creases but the floor's
/// line past the pick, so the band's side floods run round each free end and join the two sides
/// into one region; its side regions are its profile's faces, the steeper wall's narrow first face
/// and the wall above it one region, so the ball reads the straight wall and the steeper wall
/// (2.34 and 2.72 from the floor) apart.
fn the_groove_whose_sides_join_round_its_free_ends(on: bool) -> (BandGapResult, Option<String>) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, tz),
        (-tz, tz),
    ];
    let n = profile.len() as u32;
    let ((positions, triangles), chain) = sliced(&profile, (0.0, -2.0), (8, 56));
    // Two triangles per profile face per slice, then the two end fans.
    let walls = (triangles.len() as u32) - (2 * n);
    let regions: Vec<u32> = (0..triangles.len() as u32)
        .map(|t| match t {
            t if t >= walls => n,
            t => ((t / 2) % n).max(1),
        })
        .collect();
    let sharp = creases(&positions, &triangles);
    let floor = move |a: i32, b: i32| a % (n as i32) == 0 && b % (n as i32) == 0;
    set_use_rolling_ball(on);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[chain],
        &[false],
        Some(Rc::new(move |a, b| sharp(a, b) && !floor(a, b))),
        2.0,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        Some(&regions),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// One open chain whose band's own side floods join its two sides round its free ends, while the
/// ball reads them 2.34 and 2.72 from the floor: the band is tagged from the sides the ball rolled
/// on, so it is cut per side. Clear of the ends each rail stands at its own side's hand distance
/// (the straight wall's 2.3416, the steeper wall's 2.7235), near each end it follows the sphere of
/// that distance through the end vertex, both rails close onto both free end vertices, the fill
/// closes the round and the mesh is closed.
#[test]
fn a_chain_whose_band_floods_join_two_sides_that_differ_is_cut_per_side_on_the_balls_sides() {
    let (ball, route) = the_groove_whose_sides_join_round_its_free_ends(true);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(2.0);
    assert!((straight - 2.3416).abs() < 1e-3, "{straight}");
    assert!((steeper - 2.7235).abs() < 1e-3, "{steeper}");
    // The picked stretch of the floor runs from y = 2 to y = 14 at x = z = 0.
    let ends = [Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.0, 14.0, 0.0)];
    let rails = the_rails_close_onto_both_end_vertices(&ball, ends);
    let mut sides: Vec<i32> = rails.iter().map(|rail| rail.side).collect();
    sides.sort_unstable();
    assert_eq!(sides, [1, 2], "one rail per side of the ball");
    for rail in &rails {
        let middle = rail.points[rail.points.len() / 2];
        let setback = if middle.x > 0.0 { steeper } else { straight };
        let tapered = rail
            .points
            .iter()
            .filter(|p| on_the_sphere_through_the_end(**p, setback))
            .count();
        let clear = rail.points.len() - tapered;
        assert!(
            tapered > 10 && clear > 10,
            "side {}: {tapered} points in the tapers, {clear} clear of them",
            rail.side
        );
    }
}

/// A box 6 long (x), 2 deep (y) and 7 tall (z), its faces numbered as in [`a_cube_and_its_faces`]
/// but with the bottom in the top's region and the back in the front's: each of the top front
/// edge's two sides spans two faces that never touch it, as a pick's region can span the whole
/// part, so the top's region meets the front's again at the top back crease, 2 from the chain.
fn a_deep_box_whose_regions_meet_again_at_its_back() -> ((Vec<Vector3>, Triangles), Vec<u32>) {
    let part = a_box(Vector3::new(-3.0, -1.0, -6.0), Vector3::new(3.0, 1.0, 1.0));
    let regions = (0..part.1.len() as u32)
        .map(|t| match t / 2 {
            0 => 1,
            4 => 2,
            face => face,
        })
        .collect();
    (part, regions)
}

/// The deep box's top front edge (4 to 5) filleted at `size` with the switch on, its feature
/// edges every crease but the three round the back, so the band's own floods join its sides and
/// the band is tagged from the ball's.
fn the_deep_box_top_front_edge(size: f64) -> (BandGapResult, Option<String>) {
    let ((positions, triangles), regions) = a_deep_box_whose_regions_meet_again_at_its_back();
    let sharp = creases(&positions, &triangles);
    let blended = |a: i32, b: i32| {
        let (low, high) = if a < b { (a, b) } else { (b, a) };
        matches!((low, high), (6, 7) | (2, 3) | (0, 1))
    };
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[vec![4, 5]],
        &[false],
        Some(Rc::new(move |a, b| sharp(a, b) && !blended(a, b))),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        Some(&regions),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// The deep box's top front edge at 0.5 takes the ball, its band tagged from the ball's sides
/// (the top and bottom, the front and back), and the room is read at the top back crease, where
/// the top's region meets the front's again with no untagged triangle beside it. By hand: the
/// crease stands 2 from the chain across the flat top, and the ball between two flat faces at a
/// right angle sets back exactly its radius, so the band reaches the crease at a size of 2 (less
/// the kept strip's margin). Read as no crease, the room would be the sides' own reach, the far
/// corner of the bottom (sqrt(2^2 + 7^2) = 7.28) or of the back, and a band past 2 would cut
/// across the crease.
#[test]
fn a_side_region_meeting_the_other_side_away_from_the_chain_bounds_the_room_there() {
    let (ball, route) = the_deep_box_top_front_edge(0.5);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert!(ball.room < 2.0 && ball.room > 2.0 - 1e-6, "{}", ball.room);
    assert_eq!(ball.room_set_by.kind, RoomReadingKind::Foreign);
}

/// With the switch off the groove whose band floods join round its free ends builds as it always
/// has: no route is written, and the level line's room, 2 less the margin, is set where the
/// floor's crease runs on past the first free end, 2 from it at the origin, so a size of 2 is
/// refused.
#[test]
fn the_groove_whose_sides_join_round_its_free_ends_is_unchanged_with_the_switch_off() {
    let (level_line, route) = the_groove_whose_sides_join_round_its_free_ends(false);
    assert_eq!(route, None);
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert_eq!(level_line.room, 1.999999998);
    assert_eq!(level_line.room_set_by.kind, RoomReadingKind::Foreign);
    assert_eq!(level_line.room_set_by.crease_at, Some(Vector3::ZERO));
}

/// The floor's crease runs on past each free end of the groove whose band floods join round its
/// free ends, eight edges to the groove's square end, between the two sides' regions: on the
/// ball's tags each of those edges is a 1|2 edge away from the chain. The round closes onto the end
/// vertex, so by hand the band's fields (the chain's own and each side's) read every point of that
/// crease at exactly its distance from the end vertex - no ball reaches past it, which a straight
/// run's taper (each setback capped by its distance from the end vertex) makes exact - and the
/// crease never sets the room: radius 2 builds, both rails closing onto both end vertices, the mesh closed.
#[test]
fn the_crease_running_on_past_a_free_end_is_never_reached_by_the_tapered_round() {
    let (ball, route) = the_groove_whose_sides_join_round_its_free_ends(true);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert!(ball.room > 2.0, "{}", ball.room);
    let ends = [Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.0, 14.0, 0.0)];
    the_rails_close_onto_both_end_vertices(&ball, ends);
    let sides = ball.side_chains.as_ref().expect("the ball's two sides");
    let fields = [&ball.chains[0], &sides[0][0], &sides[0][1]];
    // The floor's line past each end, in the slices' quarter steps: y = 0 to 2 and 14 to 16.
    let past_the_ends = (0..8)
        .map(|j| {
            (
                f64::from(j) * 0.25,
                f64::from(j + 1) * 0.25,
                2.0 - f64::from(j + 1) * 0.25,
            )
        })
        .chain((0..8).map(|j| {
            let near = 14.0 + (f64::from(j) * 0.25);
            (near, near + 0.25, near - 14.0)
        }));
    for (from, to, nearest) in past_the_ends {
        let (a, b) = (Vector3::new(0.0, from, 0.0), Vector3::new(0.0, to, 0.0));
        for field in fields {
            let least = field.min_along_segment(a, b).expect("no budget").value;
            // The edge through the end vertex reads zero there, to rounding either way: a reading
            // the room cannot tell from a reach, which is why that one edge is exempt (an end that
            // runs out). Every edge past it stands clear by its distance.
            let clear = nearest == 0.0 || least > 0.0;
            assert!(
                clear && (least - nearest).abs() < 1e-9,
                "the crease from y = {from} to {to} reads {least}, not its distance {nearest}"
            );
        }
    }
}

/// A step: the top (z = 0, x <= 0) and the tread (z = -0.5, 0 <= x <= 4) one side region, the
/// riser (x = 0) between them another, on a block 6 deep, sliced along its length (y) and picked
/// along the top|riser edge from y = 2 to y = 14, free at both ends. Its feature edges are its
/// creases but the picked edge's line past the pick, so the band's side floods run round each free
/// end and join the two sides into one region, and the band is tagged from the ball's sides: side
/// A the top and the tread, side B the riser. Built at radius 2 with the switch on; returns the
/// result, the route and the part's vertex positions.
fn the_step_whose_tread_wraps_round_behind_its_riser(
) -> (BandGapResult, Option<String>, Vec<Vector3>) {
    let profile = [
        (0.0, 0.0),
        (0.0, -0.5),
        (4.0, -0.5),
        (4.0, -6.0),
        (-6.0, -6.0),
        (-6.0, 0.0),
    ];
    let n = profile.len() as u32;
    let ((positions, triangles), chain) = sliced(&profile, (-1.0, -3.0), (8, 56));
    let walls = (triangles.len() as u32) - (2 * n);
    // Face i runs from profile point i to i + 1: the riser is face 0, the tread face 1 and the top
    // face 5, which the tread joins.
    let regions: Vec<u32> = (0..triangles.len() as u32)
        .map(|t| match t {
            t if t >= walls => n,
            t if (t / 2) % n == 1 => n - 1,
            t => (t / 2) % n,
        })
        .collect();
    let sharp = creases(&positions, &triangles);
    let picked_line = move |a: i32, b: i32| a % (n as i32) == 0 && b % (n as i32) == 0;
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[chain],
        &[false],
        Some(Rc::new(move |a, b| sharp(a, b) && !picked_line(a, b))),
        2.0,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        Some(&regions),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route, positions)
}

/// The step whose tread wraps round behind its riser. By hand, the ball of radius 2 rests on the
/// top 1.3229 (sqrt(1.75)) from the chain and on the riser|tread rim 0.5 below it, so side A's field
/// reaches the tread up to x = 1.2247, behind the rim the ball rests on, which its contact on A never
/// sweeps. The tread is taken out of side A (`rolling_ball_sweep`): no tread triangle is in A's
/// band, cut or moved, and the room is no longer read across the tread at the rim (0.7559, a size
/// of 2 refused), so the band is cut at 2. The riser's band reaches exactly the rim its contact
/// rests on, and that rim is side B's rail (`rolling_ball_rested_rims`): B's rail is one run from
/// the free end at y = 2 down the taper, along the rim through every one of its 45 vertices
/// (0, 2.5 + 0.25 k, -0.5) in order, and up the taper to y = 14; A's rail clear of the tapers (from
/// the chain vertex at y = 3.5 to the one at 12.5, the first and last whose setback the free ends'
/// taper leaves whole) runs at the contact's distance on the top, x = -sqrt(1.75). The fill closes
/// and the mesh is closed. The round between the rails is the ball's arc
/// ([`the_steps_round_is_the_balls_arc`]).
#[test]
fn a_crease_a_sides_contacts_rest_on_is_that_sides_rail() {
    let (ball, route, positions) = the_step_whose_tread_wraps_round_behind_its_riser();
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None, "{}", ball.room);
    assert!(ball.room > 2.0, "{}", ball.room);
    let on_the_tread = |p: Vector3| p.x > -1e-12 && (p.z + 0.5).abs() < 1e-12;
    let the_tread_is_untouched = |mesh: &LevelLineMesh| {
        for (v, &p) in positions.iter().enumerate() {
            if on_the_tread(p) {
                assert_eq!(mesh.position(v as i32), p, "tread vertex {v} moved");
            }
        }
        let mut tread = 0;
        for t in 0..mesh.triangle_count() {
            let face = mesh.triangles()[t as usize];
            if !mesh.is_removed(t)
                && [face.a, face.b, face.c]
                    .iter()
                    .all(|&v| on_the_tread(mesh.position(v)))
            {
                assert_eq!(face.tag, 0, "tread triangle {t} is in side {}", face.tag);
                tread += 1;
            }
        }
        // 64 slices of two triangles each, none of them cut.
        assert_eq!(tread, 128);
    };
    the_tread_is_untouched(&ball.mesh.as_ref().expect("the band came out").borrow());

    let ends = [Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.0, 14.0, 0.0)];
    let rails = the_rails_close_onto_both_end_vertices(&ball, ends);
    let rail_b = rails.iter().find(|r| r.side == 2).expect("side B's rail");
    let on_the_rim: Vec<Vector3> = rail_b
        .points
        .iter()
        .copied()
        .filter(|p| p.x.abs() < 1e-12 && (p.z + 0.5).abs() < 1e-12)
        .collect();
    let mut along = on_the_rim.iter().map(|p| p.y).collect::<Vec<f64>>();
    if along.first() > along.last() {
        along.reverse();
    }
    let rim: Vec<f64> = (0..45).map(|k| 2.5 + (0.25 * f64::from(k))).collect();
    assert_eq!(along, rim, "side B's rail does not run along the rim");
    let rail_a = rails.iter().find(|r| r.side == 1).expect("side A's rail");
    let contact = -(1.75f64.sqrt());
    let clear = rail_a
        .points
        .iter()
        .filter(|p| p.y >= 3.5 && p.y <= 12.5)
        .collect::<Vec<_>>();
    assert!(clear.len() > 30, "{}", clear.len());
    for p in clear {
        assert!(
            (p.x - contact).abs() < 1e-9 && p.z.abs() < 1e-9,
            "side A's rail is at {p:?}, not on the top at x = {contact}"
        );
    }
    // The fill (run by `the_rails_close_onto_both_end_vertices`, closed) writes into the band's
    // own mesh, and leaves the tread as the cut did.
    the_tread_is_untouched(&ball.mesh.as_ref().expect("the filled mesh").borrow());
}

/// The step's round is the ball's own arc, not the angle's: clear of the free ends' tapers (y from
/// 3.5 to 12.5) every row from the top's contact P = (-a, y, 0) to the rim Q = (0, y, -0.5), with
/// a = sqrt(1.75), is drawn about the ball's centre O = (-a, y, -2) (`ball_row_control`), so every
/// vertex the fill mints on it stands the radius 2 from O. By hand the arc's middle is
/// O + 2 (u_P + u_Q) / |u_P + u_Q| with u_P = (0, 0, 1), u_Q = (a, 0, 1.5) / 2 and
/// |u_P + u_Q| = sqrt(3.5): (-a (1 - 1 / sqrt(3.5)), y, -2 + sqrt(3.5)) = (-0.615769, y,
/// -0.129171), and with the default four segments that is each row's middle vertex.
///
/// Only the rows whose two rail ends stand at one y are that arc: this fill is a fade (both ends
/// free), whose spokes pair each rail vertex with the other rail's nearest rather than a station at
/// the same parameter, and A's rail has a vertex where the cut crosses each slice's diagonal, so
/// every other row runs from one y to another. No arc between such P and Q lies on the ball's
/// cylinder; drawn about the ball's centre at their mean, the 216 vertices of those rows stand at
/// most 1.23e-3 off it, against 0.086 when every row was drawn about the chain point.
#[test]
fn the_steps_round_is_the_balls_arc() {
    let (ball, route, _) = the_step_whose_tread_wraps_round_behind_its_riser();
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    let ends = [Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.0, 14.0, 0.0)];
    the_rails_close_onto_both_end_vertices(&ball, ends);
    let a = 1.75f64.sqrt();
    let middle = (-a * (1.0 - (1.0 / 3.5f64.sqrt())), -2.0 + 3.5f64.sqrt());
    assert!((middle.0 + 0.615769).abs() < 1e-6 && (middle.1 + 0.129171).abs() < 1e-6);
    let mesh = ball.mesh.as_ref().expect("the filled mesh").borrow();
    // A vertex strictly between the two contacts is off every face of the part: the fill's own.
    let between = |p: &Vector3| {
        p.y >= 3.5
            && p.y <= 12.5
            && p.x > -a + 1e-9
            && p.x < -1e-9
            && p.z > -0.5 + 1e-9
            && p.z < -1e-9
    };
    let rows: Vec<Vector3> = mesh.vertices().iter().copied().filter(between).collect();
    let off = |p: &Vector3| (*p - Vector3::new(-a, p.y, -2.0)).length() - 2.0;
    let on_the_ball = rows.iter().filter(|p| off(p).abs() < 1e-9).count();
    // The rows square to the chain: 109 vertices measured, 37 of them rows' middles.
    assert!(
        on_the_ball > 90,
        "{on_the_ball} of {} on the ball",
        rows.len()
    );
    for p in &rows {
        assert!(
            off(p).abs() < 1.5e-3,
            "row vertex {p:?} stands {} off the ball",
            off(p)
        );
    }
    let middles = rows
        .iter()
        .filter(|p| (p.x - middle.0).abs() < 1e-9 && (p.z - middle.1).abs() < 1e-9)
        .count();
    assert!(middles > 30, "{middles} rows' middles on the ball's arc");
}

/// The step's round meets its free ends' tapers with no ridge. Each row of the fill is a rational
/// quadratic in four segments, minted as its three inner vertices in turn (its quarter point, its
/// middle and its three-quarter point, top to rim), so the fill's own vertices read in mesh order
/// are its rows in order along the chain. A round that changes smoothly from row to row carries a
/// row's middle no further than the larger of its quarter points' moves: the row moves and turns
/// with its rails, and its middle, the deepest point, moves most only where the row's own shape
/// steps. The ball's rows move their middle half as far as their rails (0.5 of the quarter points'
/// largest), the taper's at most 0.77 of it. Where a row the ball describes met the taper's row
/// drawn about the chain point, the round turned 41 degrees on one and 90 on the next, and the
/// middle moved 1.2 times its quarter points' largest move (0.24 against 0.20, at y = 3.47 and
/// 12.56); the rows between now fade from the ball's control point to the chain point
/// (`ball_row_control::ball_weight`).
#[test]
fn the_steps_round_meets_its_free_ends_tapers_with_no_ridge() {
    let (ball, route, positions) = the_step_whose_tread_wraps_round_behind_its_riser();
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    let ends = [Vector3::new(0.0, 2.0, 0.0), Vector3::new(0.0, 14.0, 0.0)];
    the_rails_close_onto_both_end_vertices(&ball, ends);
    let mesh = ball.mesh.as_ref().expect("the filled mesh").borrow();
    // The fill's vertices strictly between the top and the rim: off every face of the part.
    let inside = |p: &Vector3| p.x < -1e-9 && p.z < -1e-9 && p.z > -0.5 + 1e-9;
    let minted: Vec<Vector3> = mesh.vertices()[positions.len()..]
        .iter()
        .copied()
        .filter(inside)
        .collect();
    assert_eq!(
        minted.len() % 3,
        0,
        "{} vertices are not whole rows",
        minted.len()
    );
    let rows: Vec<&[Vector3]> = minted.chunks(3).collect();
    assert!(rows.len() > 100, "{} rows", rows.len());
    for row in &rows {
        // Each is one row, top to rim: it descends from its quarter point to its three-quarter.
        assert!(
            row[0].z > row[1].z && row[1].z > row[2].z,
            "{row:?} is not a row"
        );
    }
    for pair in rows.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let quarters = ieee_math::max_f64((to[0] - from[0]).length(), (to[2] - from[2]).length());
        let middle = (to[1] - from[1]).length();
        assert!(
            middle <= quarters,
            "the round steps between the rows {from:?} and {to:?}: its middle moves {middle}, \
             its quarter points at most {quarters}"
        );
    }
}
