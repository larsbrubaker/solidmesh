//! The rolling ball's two sides read from the caller's face regions
//! (`band_gap_entry_points::build_pick_with_side_regions`) rather than flooded by the kernel over
//! the build's feature edges: a cube whose back the feature edges let one side flood round into
//! the other, which the regions keep apart (the band's own tagging still joins the two sides into
//! one group there, and the sides agree, so that group reads the chain's own field), regions that do
//! not give a chain two sides, and a chain whose band tagging joins two sides that differ, which
//! keeps the level line.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use super::band_route_tests::{a_box, creases, Triangles};
use super::band_setback_tests::STEEP_DEGREES;
use super::free_end_taper_tests::sliced;

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
/// the chain has one side and keeps the level line; with them its sides are the top and the front,
/// a square corner, and the ball takes the pass with its contacts the radius from the edge.
#[test]
fn the_callers_face_regions_split_sides_the_feature_edges_let_flood_into_one() {
    let (flooded, flooded_route) = the_top_front_edge(None);
    assert_eq!(
        flooded_route.as_deref(),
        Some("level-line: chain 0: its two sides are one surface")
    );
    assert_eq!(flooded.refusal, BandGapRefusal::None);

    let (_, faces) = a_cube_and_its_faces();
    let (ball, route) = the_top_front_edge(Some(&faces));
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    for setback in &ball.chains[0].setbacks {
        assert!((setback - 0.25).abs() < 1e-8, "{setback}");
    }
}

/// Regions that put the top and the front in one region give the chain no two sides: the pass
/// keeps the level line and says so, whatever the feature edges would have flooded.
#[test]
fn face_regions_that_join_a_chains_two_sides_keep_the_level_line() {
    let (_, mut faces) = a_cube_and_its_faces();
    for region in &mut faces {
        if *region == 2 {
            *region = 1;
        }
    }
    let (result, route) = the_top_front_edge(Some(&faces));
    assert_eq!(
        route.as_deref(),
        Some("level-line: chain 0: its two sides are one face region (1)")
    );
    assert_eq!(result.refusal, BandGapRefusal::None);
}

/// A labelling of another mesh (one triangle short) names no sides at all: the pass keeps the
/// level line with its reason.
#[test]
fn face_regions_that_miss_a_triangle_keep_the_level_line() {
    let ((positions, triangles), faces) = a_cube_and_its_faces();
    set_use_rolling_ball(true);
    set_last_route(None);
    build_pick_with_side_regions(
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
        Some("level-line: the side regions label 11 triangles where the mesh has 12")
    );
}

/// The one-wall-bending groove sliced along its length, picked from y = 2 to y = 14, free at both
/// ends, built at radius 2 with the switch `on`. Its feature edges are its creases but the floor's
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

/// One open chain whose band tagging joins its two sides into one group, while the ball reads them
/// 2.34 and 2.72 from the floor: one group is cut by one field, and neither side's own nor the
/// larger stands for both, so the pass keeps the level line, the route says why, and its setbacks
/// are bit for bit those of the build with the switch off.
#[test]
fn a_chain_whose_band_joins_two_sides_that_differ_keeps_the_level_line() {
    let (ball, route) = the_groove_whose_sides_join_round_its_free_ends(true);
    let (level_line, _) = the_groove_whose_sides_join_round_its_free_ends(false);
    let route = route.expect("the outer pass writes its route");
    assert!(
        route.starts_with("level-line: chain 0 point ")
            && route.ends_with(
                "and the band's tagging joins its two sides into one region, which is not cut \
                 per side yet"
            ),
        "{route}"
    );
    assert_eq!(ball.refusal, level_line.refusal);
    assert_eq!(ball.room.to_bits(), level_line.room.to_bits());
    assert_eq!(ball.chains.len(), level_line.chains.len());
    for (a, b) in ball.chains.iter().zip(&level_line.chains) {
        let bits = |s: &[f64]| s.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(&a.setbacks), bits(&b.setbacks));
    }
}
