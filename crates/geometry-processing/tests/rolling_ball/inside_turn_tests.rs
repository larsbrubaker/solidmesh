//! The rolling ball at a sharp inside turn of a chain (`rolling_ball_legs`'s header): a square
//! pocket's floor rim picked as one closed chain, with a chain point half a radius from each
//! corner. At a corner the real ball touches the floor and both walls, centred a radius from each
//! (the three-face ball); between where its foot falls on a leg (a radius back from the corner)
//! and the corner, no ball square to the leg touches only the floor and that leg's wall, because
//! the other wall stands nearer than the radius. The level line mitres such a turn (each leg's
//! round carried on to the corner, its rails meeting at the three-face ball's contacts), so each
//! leg is walked as a chain ending on the other leg's wall, and on flat faces every setback is the
//! angle's: here 1 per unit of size on both sides, the floor's rails meeting at the three-face
//! ball's floor contact.

use geometry_processing::bevel::level_line::rolling_ball_setbacks::{for_the_pass, SideSetbacks};
use geometry_processing::bevel::level_line::BandProfile;
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use crate::band_route_tests::{creases, Triangles};
use crate::band_setback_tests::the_band;

/// How far a setback per unit of size may stand from the angle's on flat faces: every solve holds
/// its distances to 1e-11 of the radius, and a leg's end station is held in its end plane to 1e-9.
const AGREE: f64 = 1e-9;

/// The pocket's floor is 4 square; its walls stand this high, well over the ball's diameter.
const WALL: f64 = 3.0;

/// The floor's rim, counterclockwise from above: each corner, then a point half a unit along the
/// side from it and one half a unit short of the next corner.
fn rim() -> Vec<(f64, f64)> {
    let corners = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)];
    let mut rim = Vec::new();
    for (i, &(x, y)) in corners.iter().enumerate() {
        let (nx, ny) = corners[(i + 1) % 4];
        let (dx, dy) = ((nx - x) / 8.0, (ny - y) / 8.0);
        rim.push((x, y));
        rim.push((x + dx, y + dy));
        rim.push((nx - dx, ny - dy));
    }
    rim
}

/// A square pocket: its floor at z = 0 (facing up, a fan from its middle) and its four walls
/// (facing into the pocket), open at the top.
fn a_square_pocket() -> (Vec<Vector3>, Triangles) {
    a_pocket(&rim(), (2.0, 2.0))
}

/// A pocket whose floor's rim is `rim`, counterclockwise from above: its floor at z = 0 (facing
/// up, a fan from `middle`, which must see every rim point) and its walls (facing into the
/// pocket), open at the top. Rim point i is vertex i; the wall's top above it is vertex n + i; the
/// floor's middle is vertex 2n.
fn a_pocket(rim: &[(f64, f64)], middle: (f64, f64)) -> (Vec<Vector3>, Triangles) {
    let n = rim.len() as i32;
    let mut positions: Vec<Vector3> = rim.iter().map(|&(x, y)| Vector3::new(x, y, 0.0)).collect();
    positions.extend(rim.iter().map(|&(x, y)| Vector3::new(x, y, WALL)));
    positions.push(Vector3::new(middle.0, middle.1, 0.0));
    let mut triangles = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push((2 * n, i, j));
        triangles.push((i, n + j, j));
        triangles.push((i, n + i, n + j));
    }
    (positions, triangles)
}

/// The ball's setbacks for the pocket's floor rim, one closed chain, at size 1, after checking the
/// pass took the ball's route.
fn the_rim_ball() -> SideSetbacks {
    the_ball_round(a_square_pocket(), rim().len())
}

/// The ball's setbacks for `part`'s floor rim (its first `rim` vertices), one closed chain, at
/// size 1, after checking the pass took the ball's route.
fn the_ball_round((positions, triangles): (Vec<Vector3>, Triangles), rim: usize) -> SideSetbacks {
    let chains = vec![(0..rim as i32).collect::<Vec<i32>>()];
    let closed = vec![true];
    let crease = creases(&positions, &triangles);
    set_use_rolling_ball(true);
    set_last_route(None);
    let setbacks = for_the_pass(
        &positions,
        &triangles,
        &chains,
        &closed,
        Some(&*crease),
        None,
        1.0,
        BandProfile::Fillet,
        &the_band(&positions, &triangles, &chains, &closed),
        false,
    );
    let route = last_route();
    reset_to_the_default();
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    let mut setbacks = setbacks.expect("the ball takes the pocket's rim");
    assert_eq!(setbacks.len(), 1);
    setbacks.remove(0)
}

/// Every setback of the rim, the corners' and the points half a radius from them included, is the
/// angle's on both sides (a right-angled concave edge: 1 per unit of size), so the floor's rails
/// meet at each corner's three-face ball's floor contact, a radius in from both walls.
#[test]
fn a_pockets_sharp_inside_turns_take_the_angles_setbacks() {
    assert_every_setback_is_one(&the_rim_ball());
}

/// The same faces wound the other way round are a square post's bottom rim, a convex chain: the
/// ball inside the post meets each corner as a pocket's ball does, touching the bottom and both
/// walls, and every setback is again the angle's.
#[test]
fn a_posts_convex_rim_takes_the_angles_setbacks_at_its_corners() {
    let (positions, triangles) = a_square_pocket();
    let turned = triangles.iter().map(|&(a, b, c)| (a, c, b)).collect();
    assert_every_setback_is_one(&the_ball_round((positions, turned), rim().len()));
}

/// Both sides' setbacks are 1 per unit at every rim point, and no end carries a cap of its own.
fn assert_every_setback_is_one(ball: &SideSetbacks) {
    assert_eq!(ball.caps, [None, None]);
    for side in [&ball.left, &ball.right] {
        assert_eq!(side.len(), rim().len());
        for (i, got) in side.iter().enumerate() {
            assert!((got - 1.0).abs() <= AGREE, "point {i}: {got}");
        }
    }
}

fn near(got: Vector3, want: Vector3) -> bool {
    (got - want).length() <= AGREE
}

/// Half a radius short of the corner (4, 0), on the leg along +x, the station is the leg's own
/// ball carried on toward the turn - a radius off the floor and off the wall at y = 0, its centre
/// (3.5, 1, 1) - though the wall at x = 4 stands half a radius from it: that is the round the mitre
/// carries to the corner. At the corner each leg's ball, carried along its own leg to the mitre
/// plane (through the corner, square to the sum of the legs' directions), lands on the mitre's
/// point: the centre (3, 1, 1), the three-face ball's, with the floor contact (3, 1, 0) and the
/// wall contact on the walls' crease at (4, 0, 1), where the field's corner is.
#[test]
fn a_leg_carries_its_own_ball_on_to_the_turn() {
    let ball = the_rim_ball();
    // Rim point 2 is (3.5, 0), point 3 the corner (4, 0), point 4 (4, 0.5).
    assert!(
        near(ball.centres[2], Vector3::new(0.0, 1.0, 1.0)),
        "{:?}",
        ball.centres[2]
    );
    assert!(
        near(ball.centres[4], Vector3::new(-1.0, 0.0, 1.0)),
        "{:?}",
        ball.centres[4]
    );
    assert!(
        near(ball.centres[3], Vector3::new(-1.0, 1.0, 1.0)),
        "{:?}",
        ball.centres[3]
    );
    let (floor, wall) = (Vector3::new(-1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let (a, b) = (ball.contacts[0][3], ball.contacts[1][3]);
    assert!(
        (near(a, floor) && near(b, wall)) || (near(a, wall) && near(b, floor)),
        "{a:?} {b:?}"
    );
    // The corner's contacts are on end faces of both legs: no feature is named there.
    assert_eq!(ball.features[0][3], None);
    assert_eq!(ball.features[1][3], None);
}
