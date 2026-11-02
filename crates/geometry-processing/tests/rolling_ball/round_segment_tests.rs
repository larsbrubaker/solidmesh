//! What of the part a ball may not reach (`StationFault::ReachesPastThePart`): only the part within
//! the ball's circular segment beyond the chord through its two contacts, seen along the chain.
//! The round is the ball's arc beyond that chord; the part the ball overlaps on the centre's side
//! of the chord is never touched by it. Held to a concave corner under a ceiling nearer than the
//! ball's diameter, with a plate standing between the centre and the chord, and a convex edge of a
//! bar narrower than the ball's diameter, which both roll with every centre a radius off both
//! sides, as the round a fillet of that size leaves there.

use geometry_processing::bevel::rolling_ball::{ChainPoint, RollingBall, RollingBallSide};
use vector_math::Vector3;

use crate::surfaces::rectangle;

/// Exact answers, give or take the solve's own tolerance.
const EXACT: f64 = 1e-9;

/// Points along y from -4 to 4 on an edge through (`x`, y, `z`) between side A (normal
/// `normal_a`) and side B (normal `normal_b`).
fn along_y(x: f64, z: f64, normal_a: Vector3, normal_b: Vector3) -> Vec<ChainPoint> {
    (0..9)
        .map(|i| ChainPoint {
            point: Vector3::new(x, -4.0 + (i as f64), z),
            tangent: Vector3::UNIT_Y,
            normal_a,
            normal_b,
        })
        .collect()
}

#[test]
fn a_concave_round_rolls_past_a_ceiling_and_a_plate_its_ball_overlaps_before_the_chord() {
    // A channel 8 high: the floor z = 0 and the wall x = 0 meet at the picked edge, and a ceiling
    // faces down at z = 8. The R5 ball rests at (5, y, 5), touching the floor at (5, y, 0) and the
    // wall at (0, y, 5); it pokes 2 into the ceiling, but straight up from its centre, short of the
    // round. The round itself, the arc between the contacts facing the edge, rises no higher than
    // the wall contact at z = 5, 3 below the ceiling.
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        6,
        4,
    );
    let wall = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 0.0, 8.0),
        4,
        4,
    );
    let ceiling = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 8.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        4,
        6,
    );
    let (a, b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &wall),
    );
    // A plate across the line from the centre toward the corner, through (4, y, 4) square to it:
    // 1.41 from the centre, inside the ball between the centre and the chord x + z = 5 through the
    // contacts, so on the centre's side of the chord and clear of the round.
    let plate = rectangle(
        &mut positions,
        Vector3::new(3.7, -10.0, 4.3),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.6, 0.0, -0.6),
        4,
        1,
    );
    let part = RollingBallSide::new(&positions, &[floor, wall, ceiling, plate].concat());
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: Some(&part),
        radius: 5.0,
        convex: false,
    };
    let walk = ball
        .walk(&along_y(0.0, 0.0, Vector3::UNIT_Z, Vector3::UNIT_X))
        .unwrap();
    assert_eq!(walk.stations.len(), 9);
    for s in &walk.stations {
        let y = s.chain_point.y;
        assert!(
            (s.centre - Vector3::new(5.0, y, 5.0)).length() <= EXACT,
            "{s:?}"
        );
        assert!((s.contact_a.point - Vector3::new(5.0, y, 0.0)).length() <= EXACT);
        assert!((s.contact_b.point - Vector3::new(0.0, y, 5.0)).length() <= EXACT);
        // The ball does reach the plate and the ceiling: what lets it roll is where.
        assert!((part.closest(s.centre).unwrap().distance - 2.0_f64.sqrt()).abs() <= EXACT);
    }
}

#[test]
fn a_convex_round_rolls_along_a_bar_narrower_than_its_ball() {
    // A bar 2 by 2 in section (x and z from 0 to 2) along y; the picked edge is its top front edge
    // at x = 2, z = 2, between the top (A, facing +z) and the front (B, facing +x). The R1.5 ball
    // inside rests at (0.5, y, 0.5), touching the top at x = 0.5 and the front at z = 0.5, both
    // within their faces. It pokes out through the bottom and the back, 1 past each, but only
    // on the centre's side of the chord: the round's arc beyond it stays within the bar, the round a
    // fillet of 1.5 leaves on a bar 2 wide.
    let mut positions = Vec::new();
    let top = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 2.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        2,
        4,
    );
    let front = rectangle(
        &mut positions,
        Vector3::new(2.0, -10.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 0.0, 2.0),
        4,
        2,
    );
    let bottom = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        4,
        2,
    );
    let back = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(0.0, 20.0, 0.0),
        2,
        4,
    );
    let (a, b) = (
        RollingBallSide::new(&positions, &top),
        RollingBallSide::new(&positions, &front),
    );
    let part = RollingBallSide::new(&positions, &[top, front, bottom, back].concat());
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: Some(&part),
        radius: 1.5,
        convex: true,
    };
    let walk = ball
        .walk(&along_y(2.0, 2.0, Vector3::UNIT_Z, Vector3::UNIT_X))
        .unwrap();
    assert_eq!(walk.stations.len(), 9);
    for s in &walk.stations {
        let y = s.chain_point.y;
        assert!(
            (s.centre - Vector3::new(0.5, y, 0.5)).length() <= EXACT,
            "{s:?}"
        );
        assert!((s.contact_a.point - Vector3::new(0.5, y, 2.0)).length() <= EXACT);
        assert!((s.contact_b.point - Vector3::new(2.0, y, 0.5)).length() <= EXACT);
        assert!((part.closest(s.centre).unwrap().distance - 0.5).abs() <= EXACT);
    }
}
