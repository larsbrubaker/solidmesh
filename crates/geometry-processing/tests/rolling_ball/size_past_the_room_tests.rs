//! A pass of one chain the rolling ball refuses at a size its part has no room for: it is refused
//! by its size, naming the ball's own room, read off a walk at a smaller size the ball takes
//! (`level_line::rolling_ball_size_room`). The size is the user's to change, and the room is the
//! number that tells them to what. Where no room explains the refusal, it stays the ball's.

use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    ROLLING_BALL_REFUSED, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use crate::band_route_tests::{a_box, built, Triangles};

/// A plate 10 by 10 and 1 thick, and its top edge along x at y = 5 (corners 7 to 6): the side
/// face below the edge is 1 tall, so a round larger than that has no face to touch on that side.
fn a_plate() -> (Vec<Vector3>, Triangles) {
    a_box(Vector3::new(-5.0, -5.0, 0.0), Vector3::new(5.0, 5.0, 1.0))
}

fn the_top_edge() -> Vec<Vec<i32>> {
    vec![vec![7, 6]]
}

/// The pick's room as the build names it with the switch off, where it refuses `size`.
fn level_line_room(size: f64) -> BandGapResult {
    let (off, _) = built(&a_plate(), &the_top_edge(), size, false);
    assert_eq!(off.refusal, BandGapRefusal::Size, "room {}", off.room);
    off
}

/// Far past the plate's thickness (R3 and R30 on a plate 1 thick) the ball's round reaches past
/// the part, and the pass is refused by its size, naming the room the ball's own walk at a smaller
/// size reads: on flat faces the same number whatever size it was read at, and the plate's own
/// limit, the level line's room there. No band is handed back as if built, and the route still
/// says the ball refused the size asked.
#[test]
fn a_size_past_the_room_is_refused_by_size_naming_the_balls_room() {
    let off = level_line_room(3.0);
    for size in [3.0, 30.0] {
        let (on, route) = built(&a_plate(), &the_top_edge(), size, true);
        assert_eq!(on.refusal, BandGapRefusal::Size, "at {size}: {route:?}");
        assert!(
            (on.room - off.room).abs() <= 1e-9,
            "at {size}: the room named {} against the plate's {}",
            on.room,
            off.room
        );
        assert!(on.room > 0.0 && on.room < size, "{}", on.room);
        assert_eq!(on.rolling_ball_refusal, None, "a size refusal is no ball's");
        assert!(on.mesh.is_none(), "nothing is cut");
        assert!(
            route
                .as_deref()
                .is_some_and(|r| r.starts_with(ROLLING_BALL_REFUSED)),
            "{route:?}"
        );
    }
}

/// Under the room it names, the ball takes the pass: the room is a size the user can ask for.
#[test]
fn under_the_room_named_the_ball_builds() {
    let (refused, _) = built(&a_plate(), &the_top_edge(), 3.0, true);
    let (under, route) = built(&a_plate(), &the_top_edge(), 0.9 * refused.room, true);
    assert_eq!(under.refusal, BandGapRefusal::None, "{route:?}");
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
}

/// The same edge at a size within its room that the ball still cannot take - a sealed cavity just
/// under the edge, which the round would cut into and no room reading sees (it is no face of
/// either side) - is the ball's refusal, with its reason, and no band: the room read at a smaller
/// size the ball takes is above the size asked, so it does not explain the refusal.
#[test]
fn where_no_room_explains_it_the_refusal_stays_the_balls() {
    let (mut positions, mut triangles) = a_plate();
    let (cavity, cavity_triangles) =
        a_box(Vector3::new(-4.9, 4.7, 0.6), Vector3::new(4.9, 4.85, 0.85));
    let base = positions.len() as i32;
    positions.extend(cavity);
    triangles.extend(
        cavity_triangles
            .iter()
            .map(|&(a, b, c)| (base + a, base + c, base + b)),
    );
    let part = (positions, triangles);
    let size = 0.6;
    let (off, _) = built(&part, &the_top_edge(), size, false);
    assert_eq!(
        off.refusal,
        BandGapRefusal::None,
        "within the level line's room"
    );
    let (on, route) = built(&part, &the_top_edge(), size, true);
    assert_eq!(on.refusal, BandGapRefusal::RollingBall, "{route:?}");
    let why = on
        .rolling_ball_refusal
        .clone()
        .expect("a rolling-ball refusal says why");
    assert!(why.contains("ReachesPastThePart"), "{why}");
    assert!(on.mesh.is_none(), "a refused pass hands back no band");
    assert!(on.chains.is_empty(), "nor any field it was not cut by");
}
