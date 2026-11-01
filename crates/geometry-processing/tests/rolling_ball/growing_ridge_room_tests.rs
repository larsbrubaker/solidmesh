//! A growing neighbour's ridge bounds a fillet's room on the rolling ball's route as it does on the
//! level line's (`level_line::ball_pierce_room`): the rib of the strip clip tests, its box made 20
//! deep so the ball of the sizes here stays inside the part, the box's top front edge picked past
//! a wedge rib whose flanks grow and whose ridge, from its tip T (10, 3, 10) to R (10, 8, 20),
//! leaves the round at 6 / (3 - sqrt 5) = 7.854 (the ball's centre at the square edge is
//! (x, s, 10 - s); `ball_pierce_room_tests` derives the root).

use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
};
use std::collections::HashMap;
use vector_math::Vector3;

use crate::band_route_tests::{built, Triangles};

/// The rib's ridge's room by hand (the reading lands within its margin of two snaps under it).
fn the_ridges_room() -> f64 {
    6.0 / (3.0 - 5.0_f64.sqrt())
}

fn the_ribbed_box() -> (Vec<Vector3>, Triangles) {
    let positions = vec![
        Vector3::new(0.0, 0.0, -10.0),
        Vector3::new(20.0, 0.0, -10.0),
        Vector3::new(20.0, 20.0, -10.0),
        Vector3::new(0.0, 20.0, -10.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
        Vector3::new(20.0, 20.0, 10.0),
        Vector3::new(0.0, 20.0, 10.0),
        Vector3::new(10.0, 3.0, 10.0),
        Vector3::new(9.0, 8.0, 10.0),
        Vector3::new(11.0, 8.0, 10.0),
        Vector3::new(10.0, 8.0, 20.0),
    ];
    let triangles = vec![
        // The box's bottom and walls.
        (0, 2, 1),
        (0, 3, 2),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
        // The top round the rib's base T P Q.
        (4, 5, 8),
        (5, 10, 8),
        (5, 6, 10),
        (6, 9, 10),
        (6, 7, 9),
        (7, 4, 9),
        (4, 8, 9),
        // The rib's flanks and back.
        (8, 11, 9),
        (8, 10, 11),
        (9, 11, 10),
    ];
    (positions, triangles)
}

/// The top front edge.
fn the_chain() -> Vec<i32> {
    vec![4, 5]
}

/// The pick filled, the fill refusing nothing and every edge run once each way.
fn fills_closed(result: &BandGapResult) {
    let fill = gap_fill::fill(result, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS)
        .expect("the fill runs");
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
            "edge {a}-{b}: the mesh is open"
        );
    }
}

/// Above the ridge's room both routes refuse the size, naming that room: the ball's centres stand
/// where the square edge's angle puts them, so the ridge's line leaves the ball's round where it
/// leaves the level line's. The room is read along the ridge's whole line because the rib's flanks
/// clip the round along it: a band cut past that room is one the fill's growing-neighbour clip
/// refuses.
#[test]
fn a_size_past_the_ridges_room_is_refused_on_both_routes() {
    let part = the_ribbed_box();
    for on in [false, true] {
        let (result, route) = built(&part, &[the_chain()], 7.9, on);
        assert_eq!(result.refusal, BandGapRefusal::Size, "on {on}: {route:?}");
        assert!(
            (result.room - the_ridges_room()).abs() < 1e-6,
            "on {on}: room {}, by hand {}",
            result.room,
            the_ridges_room()
        );
        if on {
            assert_eq!(route.as_deref(), Some("rolling-ball"));
        }
    }
}

/// Below it both routes build the rib's edge, the fill closed and the mesh closed.
#[test]
fn a_size_under_the_ridges_room_builds_closed_on_both_routes() {
    let part = the_ribbed_box();
    for on in [false, true] {
        let (result, route) = built(&part, &[the_chain()], 7.0, on);
        assert_eq!(result.refusal, BandGapRefusal::None, "on {on}: {route:?}");
        if on {
            assert_eq!(route.as_deref(), Some("rolling-ball"));
        }
        fills_closed(&result);
    }
}
