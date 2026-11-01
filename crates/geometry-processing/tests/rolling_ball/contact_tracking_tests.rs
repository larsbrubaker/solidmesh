//! A side's contact followed over its surface as the ball rolls (`RollingBallSide::closest_tracked`,
//! used by `RollingBall::walk` after the first ball): a wide round whose contact slides down a
//! side, across the corner's bisector plane where reading the side within its half would hide it,
//! held to the hand answer; and a side that wraps round behind the other, whose wrapped part stands
//! as near as the surface the ball rolls on and is never taken for it.

use geometry_processing::bevel::rolling_ball::{
    ChainPoint, RollingBall, RollingBallSide, StationFailure,
};
use vector_math::Vector3;

use crate::surfaces::rectangle;

const RADIUS: f64 = 2.0;

/// Exact answers, give or take the solve's own tolerance.
const EXACT: f64 = 1e-9;

/// The ceiling's rise along x and its fall along y: it is the plane z = h(y) + SLOPE x.
const SLOPE: f64 = 0.2;
const FALL: f64 = 0.5;

/// The wall's height at `y`, where the ceiling starts.
fn height(y: f64) -> f64 {
    8.0 - (FALL * y)
}

/// Side A: the wall x = 0 (facing +x) from the floor up to h(y), then folding sharply into the
/// ceiling z = h(y) + SLOPE x (facing down) out to x = 30, y from -2 to 16, one mesh sharing the
/// fold's vertices. Its triangles are appended to `positions`.
fn wall_and_ceiling(positions: &mut Vec<Vector3>) -> Vec<[i32; 3]> {
    let (ny, nx) = (18usize, 30usize);
    let y_of = |j: usize| -2.0 + (j as f64);
    let base = positions.len() as i32;
    // Row j: the wall's foot, then the ceiling's columns from the fold (i = 0) outward.
    for j in 0..=ny {
        let y = y_of(j);
        positions.push(Vector3::new(0.0, y, 0.0));
        for i in 0..=nx {
            let x = i as f64;
            positions.push(Vector3::new(x, y, height(y) + (SLOPE * x)));
        }
    }
    let row = (nx + 2) as i32;
    let foot = |j: usize| base + (j as i32 * row);
    let ceiling = |i: usize, j: usize| base + (j as i32 * row) + 1 + i as i32;
    let mut triangles = Vec::new();
    for j in 0..ny {
        triangles.push([foot(j), foot(j + 1), ceiling(0, j + 1)]);
        triangles.push([foot(j), ceiling(0, j + 1), ceiling(0, j)]);
        for i in 0..nx {
            triangles.push([ceiling(i, j), ceiling(i, j + 1), ceiling(i + 1, j + 1)]);
            triangles.push([ceiling(i, j), ceiling(i + 1, j + 1), ceiling(i + 1, j)]);
        }
    }
    triangles
}

/// The hand answer at chain point y: (centre, A's contact, B's contact). Beside the wall the ball
/// sits in the square corner, centre (r, y, r). Once the ceiling comes within r of that, it rests
/// on the ceiling and the floor: its centre stays r above the floor, in the station's plane y, and
/// r below the ceiling, whose unit normal is g = (-SLOPE, FALL, 1) / N: h(y) + SLOPE x - r = r N,
/// so x = (r N + r - h(y)) / SLOPE, and the contact is the centre plus r g.
fn hand_answer(y: f64) -> (Vector3, Vector3, Vector3) {
    let n = ((SLOPE * SLOPE) + (FALL * FALL) + 1.0).sqrt();
    let x = ((RADIUS * n) + RADIUS - height(y)) / SLOPE;
    if x <= RADIUS {
        let centre = Vector3::new(RADIUS, y, RADIUS);
        return (
            centre,
            Vector3::new(0.0, y, RADIUS),
            Vector3::new(RADIUS, y, 0.0),
        );
    }
    let centre = Vector3::new(x, y, RADIUS);
    let g = Vector3::new(-SLOPE, FALL, 1.0) / n;
    (centre, centre + (g * RADIUS), Vector3::new(x, y, 0.0))
}

#[test]
fn a_wide_round_follows_its_contact_up_onto_the_ceiling_across_the_corners_bisector() {
    // The chain is the wall's foot, y from 0 to 12. In the middle the ball sits in the square
    // corner; past y = 8.26 the ceiling, sinking as y grows, pushes it out along the floor, and at
    // y = 12 its ceiling contact is at x = 11.0, z = 3.76: below the corner's bisector plane x = z,
    // on the floor's half, where reading the wall's side within its own half cannot see it.
    let mut positions = Vec::new();
    let side_a = wall_and_ceiling(&mut positions);
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -2.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 18.0, 0.0),
        30,
        18,
    );
    let (a, b) = (
        RollingBallSide::new(&positions, &side_a),
        RollingBallSide::new(&positions, &floor),
    );
    let part = RollingBallSide::new(&positions, &[side_a, floor].concat());
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: Some(&part),
        radius: RADIUS,
        convex: false,
    };
    let points: Vec<ChainPoint> = (0..=12)
        .map(|k| ChainPoint {
            point: Vector3::new(0.0, k as f64, 0.0),
            tangent: Vector3::UNIT_Y,
            normal_a: Vector3::UNIT_X,
            normal_b: Vector3::UNIT_Z,
        })
        .collect();

    let walk = ball.walk(&points).unwrap();
    assert_eq!(walk.stations.len(), points.len());
    for s in &walk.stations {
        let (centre, on_a, on_b) = hand_answer(s.chain_point.y);
        assert!((s.centre - centre).length() <= EXACT, "{s:?}");
        assert!((s.contact_a.point - on_a).length() <= EXACT, "{s:?}");
        assert!((s.contact_b.point - on_b).length() <= EXACT, "{s:?}");
    }
    let end = walk.stations[12];
    assert!(
        end.contact_a.point.z < end.contact_a.point.x,
        "the last contact lies across the bisector plane: {end:?}"
    );

    // A ball seeded there from nothing reads the wall's side only within its half of the corner,
    // which ends at the bisector plane short of that contact, and finds no ball.
    let seeded = ball.grown_station(&points[12]);
    assert!(
        matches!(seeded, Err(StationFailure::Fault { .. })),
        "{seeded:?}"
    );
}

/// Side B of the wrapped test, one mesh: a channel round under the floor, its cross-section the
/// path from the floor's far rim (2, 0) down the skirt x = 2 to z = -10, across the bottom to the
/// wall x = 0, and up the wall through the chain to z = 30, run along y from -10 to 10. The wall
/// faces +x, the bottom and skirt into the channel. The skirt's triangles come first. Returns the
/// triangles and how many are the skirt's.
fn channel_round_behind(positions: &mut Vec<Vector3>) -> (Vec<[i32; 3]>, i32) {
    let mut path: Vec<(f64, f64)> = vec![(2.0, 0.0), (2.0, -5.0), (2.0, -10.0), (1.0, -10.0)];
    for k in 0..=8 {
        path.push((0.0, -10.0 + (5.0 * k as f64)));
    }
    let cells = 4usize;
    let base = positions.len() as i32;
    for j in 0..=cells {
        let y = -10.0 + (20.0 * (j as f64 / cells as f64));
        for &(x, z) in &path {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let at = |i: usize, j: usize| base + ((j * path.len()) + i) as i32;
    let mut triangles = Vec::new();
    for i in 0..path.len() - 1 {
        for j in 0..cells {
            triangles.push([at(i, j), at(i, j + 1), at(i + 1, j + 1)]);
            triangles.push([at(i, j), at(i + 1, j + 1), at(i + 1, j)]);
        }
    }
    (triangles, (2 * 2 * cells) as i32)
}

#[test]
fn a_tracked_contact_never_moves_onto_surface_wrapped_round_behind_the_other_side() {
    // The floor (side A) is 2 wide beside the wall x = 0. Side B is one connected surface wrapping
    // round behind A: the wall, running on down past the chain under the floor, across a bottom,
    // and up a skirt to the floor's far rim (x = 2), listed first so it wins every tie. The ball
    // rests on the wall and the floor's rim at centre (5, y, 4), where the skirt's top stands
    // exactly as near as the wall, so a whole reading of B could answer with it. Followed from the
    // wall, B's contact stays on the wall at every station, though B's surface runs on unbroken
    // from the wall to the skirt.
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        2,
        4,
    );
    let (outer, skirt_triangles) = channel_round_behind(&mut positions);
    let (a, b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &outer),
    );
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: None,
        radius: 5.0,
        convex: false,
    };
    let points: Vec<ChainPoint> = (0..13)
        .map(|k| ChainPoint {
            point: Vector3::new(0.0, -6.0 + (k as f64), 0.0),
            tangent: Vector3::UNIT_Y,
            normal_a: Vector3::UNIT_Z,
            normal_b: Vector3::UNIT_X,
        })
        .collect();
    let stations = ball.walk(&points).unwrap().stations;
    assert_eq!(stations.len(), 13);
    for s in &stations {
        let y = s.chain_point.y;
        assert!(
            (s.centre - Vector3::new(5.0, y, 4.0)).length() <= 1e-8,
            "{s:?}"
        );
        // The wrapped part is as near as the wall: a whole reading could take it.
        let skirt = b.closest(s.centre).unwrap();
        assert!(skirt.distance <= 5.0 + 1e-8, "{skirt:?}");
        // The contact is the wall's, never the skirt's.
        assert!(s.contact_b.triangle >= skirt_triangles, "{s:?}");
        assert!((s.contact_b.point - Vector3::new(0.0, y, 4.0)).length() <= 1e-8);

        // Lower the ball to z = 1 and the skirt's top is strictly nearer (sqrt 10) than the wall
        // (5): a whole reading jumps to it, the followed contact slides down the wall instead.
        let lower = Vector3::new(5.0, y, 1.0);
        let whole = b.closest(lower).unwrap();
        assert!(whole.triangle < skirt_triangles, "{whole:?}");
        assert!(
            (whole.distance - 10.0_f64.sqrt()).abs() <= EXACT,
            "{whole:?}"
        );
        let followed = b.closest_tracked(lower, &s.contact_b, &[]).unwrap();
        assert!(followed.triangle >= skirt_triangles, "{followed:?}");
        assert!((followed.point - Vector3::new(0.0, y, 1.0)).length() <= EXACT);
    }
}
