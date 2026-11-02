//! A ball never passes through a side it rolls on (`RollingBall::walk`): where the rolling ball
//! comes upon another part of a side - a ridge on the wall ahead, which the contact followed over
//! the wall's surface cannot slide onto - the contact moves to that part, and the ball rides on it.
//! Held to a corner whose wall carries a ridge partway along the chain, where every centre is the
//! hand answer, the ball never stands nearer a side than the radius, and the same corner turned and
//! moved through space walks to the same centres.

use geometry_processing::bevel::rolling_ball::{ChainPoint, RollingBall, RollingBallSide};
use vector_math::{Matrix4X4, Vector3};

use crate::surfaces::rectangle;

const RADIUS: f64 = 5.0;

/// Exact answers, give or take the solve's own tolerance.
const EXACT: f64 = 1e-9;

/// The corner: the floor z = 0 (side A, facing +z, x from 0 to 30) and the wall x = 0 (side B,
/// facing +x, z from 0 to 20), both from y = -12 to 14, and on the wall a ridge from y = 3 to 12:
/// a box x from 0 to 1, z from 7 to 9, its faces wound to face out into the corner's free space.
/// The ridge shares no vertex with the wall, so a contact followed over the wall's triangles can
/// never reach it. Returns the positions and each side's triangles.
fn corner_with_a_ridge() -> (Vec<Vector3>, Vec<[i32; 3]>, Vec<[i32; 3]>) {
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -12.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 26.0, 0.0),
        6,
        13,
    );
    let mut wall = rectangle(
        &mut positions,
        Vector3::new(0.0, -12.0, 0.0),
        Vector3::new(0.0, 26.0, 0.0),
        Vector3::new(0.0, 0.0, 20.0),
        13,
        20,
    );
    let (y0, y1, z0, z1) = (3.0, 12.0, 7.0, 9.0);
    // Front (+x), underside (-z), top (+z), near end (-y), far end (+y).
    for (corner, u, v) in [
        (
            Vector3::new(1.0, y0, z0),
            Vector3::new(0.0, y1 - y0, 0.0),
            Vector3::new(0.0, 0.0, z1 - z0),
        ),
        (
            Vector3::new(0.0, y0, z0),
            Vector3::new(0.0, y1 - y0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        ),
        (
            Vector3::new(0.0, y0, z1),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, y1 - y0, 0.0),
        ),
        (
            Vector3::new(0.0, y0, z0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, z1 - z0),
        ),
        (
            Vector3::new(0.0, y1, z0),
            Vector3::new(0.0, 0.0, z1 - z0),
            Vector3::new(1.0, 0.0, 0.0),
        ),
    ] {
        wall.extend(rectangle(&mut positions, corner, u, v, 2, 2));
    }
    (positions, floor, wall)
}

/// The chain along the corner's edge, y from -8 to 8 in steps of 1, each point and direction
/// carried by `motion`.
fn chain(motion: Matrix4X4) -> Vec<ChainPoint> {
    (0..17)
        .map(|k| ChainPoint {
            point: Vector3::new(0.0, -8.0 + (k as f64), 0.0).transform_position(motion),
            tangent: Vector3::UNIT_Y.transform_vector(motion),
            normal_a: Vector3::UNIT_Z.transform_vector(motion),
            normal_b: Vector3::UNIT_X.transform_vector(motion),
        })
        .collect()
}

/// The walk's centres for the corner carried by `motion`, carried back by its inverse, each
/// checked to stand no nearer either side than the radius.
fn centres(motion: Matrix4X4) -> Vec<Vector3> {
    let (positions, floor, wall) = corner_with_a_ridge();
    let moved: Vec<Vector3> = positions
        .iter()
        .map(|p| p.transform_position(motion))
        .collect();
    let (a, b) = (
        RollingBallSide::new(&moved, &floor),
        RollingBallSide::new(&moved, &wall),
    );
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: None,
        radius: RADIUS,
        convex: false,
    };
    let walk = ball.walk(&chain(motion)).unwrap();
    assert_eq!(walk.stations.len(), 17);
    let back = motion.inverted();
    walk.stations
        .iter()
        .map(|s| {
            for side in [&a, &b] {
                let nearest = side.closest(s.centre).unwrap();
                assert!(
                    nearest.distance >= RADIUS - EXACT,
                    "the ball passes {} into a side it rolls on: {s:?}",
                    RADIUS - nearest.distance
                );
            }
            s.centre.transform_position(back)
        })
        .collect()
}

#[test]
fn a_ball_rolling_onto_a_ridge_of_its_wall_rides_on_the_ridge() {
    // Clear of the ridge (y <= 0: its near corner (1, 3, 7) stands sqrt(21 + (3 - y)^2) from
    // (5, y, 5)) the ball rests on the floor and the wall at (5, y, 5). Along the ridge (y >= 3)
    // it rests on the floor and the ridge's lower front edge (1, y, 7): (x - 1)^2 + 2^2 = 5^2, so
    // x = 1 + sqrt 21, standing clear of the wall.
    let on_ridge = 1.0 + 21.0_f64.sqrt();
    for (k, centre) in centres(Matrix4X4::IDENTITY).iter().enumerate() {
        let y = -8.0 + (k as f64);
        assert!((centre.y - y).abs() <= EXACT, "{centre:?}");
        assert!((centre.z - RADIUS).abs() <= EXACT, "{centre:?}");
        if y <= 0.0 {
            assert!((centre.x - RADIUS).abs() <= EXACT, "{centre:?} at {y}");
        } else if y >= 3.0 {
            assert!((centre.x - on_ridge).abs() <= EXACT, "{centre:?} at {y}");
        }
    }
}

#[test]
fn the_corner_with_a_ridge_walks_to_the_same_centres_turned_and_moved() {
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125));
    let still = centres(Matrix4X4::IDENTITY);
    let moved = centres(motion);
    // Relative to the part's size (about 30): the rounding a turn and a move leave, no more.
    for (s, m) in still.iter().zip(&moved) {
        assert!(
            (*s - *m).length() <= 30.0 * EXACT,
            "{s:?} still, {m:?} moved"
        );
    }
}
