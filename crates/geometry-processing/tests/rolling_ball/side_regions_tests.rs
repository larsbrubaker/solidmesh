//! The rolling ball's two sides read from the caller's face regions
//! (`band_gap_entry_points::build_pick_with_side_regions`) rather than flooded by the kernel over
//! the build's feature edges: a cube whose back the feature edges let one side flood round into
//! the other, which the regions keep apart, and regions that do not give a chain two sides.

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
