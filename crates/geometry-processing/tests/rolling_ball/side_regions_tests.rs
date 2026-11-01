//! The rolling ball's two sides read from the caller's face regions
//! (`band_gap_entry_points::build_pick_with_side_regions`) rather than flooded by the kernel over
//! the build's feature edges: a cube whose back the feature edges let one side flood round into the
//! other, which the regions keep apart (the band's own floods join the two sides there, so the band
//! is tagged from the ball's sides), regions that do not give a chain two sides (refused by the
//! ball, strict), a chain whose band's own side floods join two sides that differ, whose band is
//! tagged from the ball's sides and cut per side (and which the switch off leaves as it was), whose
//! crease running on past each free end the tapered round never reaches, and a box whose two side
//! regions meet again away from the chain, where the room is read.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, RoomReadingKind,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_REFUSED,
    ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

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

/// The kernel's flood runs round the back from the top into the front, so without the regions
/// the chain has one side and the ball refuses it; with them its sides are the top and the front,
/// a square corner, and the ball takes the pass with its contacts the radius from the edge.
#[test]
fn the_callers_face_regions_split_sides_the_feature_edges_let_flood_into_one() {
    let (flooded, flooded_route) = the_top_front_edge(None);
    assert_eq!(
        flooded_route.as_deref(),
        Some("refused: chain 0: its two sides are one surface")
    );
    assert_eq!(flooded.refusal, BandGapRefusal::RollingBall);
    assert_eq!(
        flooded.rolling_ball_refusal.as_deref(),
        Some("chain 0: its two sides are one surface")
    );

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
