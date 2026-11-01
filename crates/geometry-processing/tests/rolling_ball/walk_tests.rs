//! The ball walked along a whole chain (`RollingBall::walk`), held to centres worked out by hand:
//! a straight square corner, where every station's centre is a radius off both faces; a tilted
//! cylinder through a plate, whose angle to the plate changes all the way round, where every
//! centre is a radius above the plate and a radius out from the cylinder; a floor that narrows to
//! a point beside a wall, where the floor's contact moves onto its rim exactly where the floor
//! grows narrower than the radius and the centre then sinks as the circle through the rim says;
//! a side that wraps round the other past a shared rim; the fault that keeps a ball out of
//! the solid; and chains ending on a face that is neither side, where the centre stays on the
//! chain's side of the end face's plane, the end face never refuses the ball, and a side is never
//! read beyond the end face's plane.

use std::f64::consts::PI;

use geometry_processing::bevel::rolling_ball::{
    ChainPoint, ContactFeature, EndFace, RollingBall, RollingBallSide, StationFailure,
    StationFault, WalkError,
};
use vector_math::Vector3;

use crate::surfaces::{cylinder_wall, rectangle};

const RADIUS: f64 = 5.0;

/// Exact answers, give or take the solve's own tolerance.
const EXACT: f64 = 1e-9;

fn ball<'a>(
    a: &'a RollingBallSide,
    b: &'a RollingBallSide,
    part: Option<&'a RollingBallSide>,
) -> RollingBall<'a> {
    RollingBall {
        side_a: a,
        side_b: b,
        part,
        radius: RADIUS,
        convex: false,
    }
}

/// Points along y on the x = 0, z = 0 edge between a floor (normal +z) and a wall (normal +x).
fn along_y(from: f64, to: f64, count: usize) -> Vec<ChainPoint> {
    (0..count)
        .map(|i| ChainPoint {
            point: Vector3::new(
                0.0,
                from + ((to - from) * (i as f64 / (count - 1) as f64)),
                0.0,
            ),
            tangent: Vector3::UNIT_Y,
            normal_a: Vector3::UNIT_Z,
            normal_b: Vector3::UNIT_X,
        })
        .collect()
}

/// The wall x = 0, from y = -12 to 12 and z = 0 to 30, facing +x: longer than every chain here, so
/// the ball never touches its ends.
fn wall(positions: &mut Vec<Vector3>) -> Vec<[i32; 3]> {
    rectangle(
        positions,
        Vector3::new(0.0, -12.0, 0.0),
        Vector3::new(0.0, 24.0, 0.0),
        Vector3::new(0.0, 0.0, 30.0),
        4,
        6,
    )
}

#[test]
fn along_a_straight_square_corner_every_centre_is_a_radius_off_both_faces() {
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        6,
        4,
    );
    let wall = wall(&mut positions);
    let (a, b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &wall),
    );
    let part = RollingBallSide::new(&positions, &[floor, wall].concat());
    let stations = ball(&a, &b, Some(&part))
        .walk(&along_y(-8.0, 8.0, 17))
        .unwrap()
        .stations;
    assert_eq!(stations.len(), 17);
    for s in &stations {
        let y = s.chain_point.y;
        assert!(
            (s.centre - Vector3::new(RADIUS, y, RADIUS)).length() <= EXACT,
            "{s:?}"
        );
        assert!((s.contact_a.point - Vector3::new(RADIUS, y, 0.0)).length() <= EXACT);
        assert!((s.contact_b.point - Vector3::new(0.0, y, RADIUS)).length() <= EXACT);
    }
}

#[test]
fn round_a_tilted_cylinder_the_centre_keeps_a_radius_above_the_plate_and_out_from_the_wall() {
    // A cylinder of radius 10 leaning 30 degrees off upright meets the plate z = 0 in an ellipse,
    // and the angle between them runs from 60 to 120 degrees round it. A ball touching both
    // stands a radius above the plate and a radius out from the cylinder: on the circle of radius
    // 15 about the axis, in the plane z = 5. The cylinder has 192 facets, so its distance is the
    // smooth one less at most the facets' sagitta.
    let (cylinder_radius, lean, sides) = (10.0, PI / 6.0, 192);
    let axis = Vector3::new(lean.sin(), 0.0, lean.cos());
    let mut positions = Vec::new();
    let plate = rectangle(
        &mut positions,
        Vector3::new(-40.0, -40.0, 0.0),
        Vector3::new(80.0, 0.0, 0.0),
        Vector3::new(0.0, 80.0, 0.0),
        8,
        8,
    );
    let first = positions.len();
    let wall = cylinder_wall(&mut positions, cylinder_radius, 60.0, sides, 0.5, 12);
    for p in &mut positions[first..] {
        // Lower it so it passes through the plate, then lean it about the y axis.
        let q = Vector3::new(p.x, p.y, p.z - 30.0);
        *p = Vector3::new(
            (q.x * lean.cos()) + (q.z * lean.sin()),
            q.y,
            (q.z * lean.cos()) - (q.x * lean.sin()),
        );
    }

    // The ellipse the smooth cylinder cuts in the plate, and its tangent, at 72 points.
    let (major, minor) = (cylinder_radius / lean.cos(), cylinder_radius);
    let points: Vec<ChainPoint> = (0..=72)
        .map(|i| {
            let t = 2.0 * PI * (i as f64 / 72.0);
            let point = Vector3::new(major * t.cos(), minor * t.sin(), 0.0);
            let tangent = Vector3::new(-major * t.sin(), minor * t.cos(), 0.0).get_normal();
            let from_axis = point - (axis * point.dot(axis));
            ChainPoint {
                point,
                tangent,
                normal_a: Vector3::UNIT_Z,
                normal_b: from_axis.get_normal(),
            }
        })
        .collect();
    let angles: Vec<f64> = points
        .iter()
        .map(|p| p.normal_a.dot(p.normal_b).acos().to_degrees())
        .collect();
    assert!(angles.iter().any(|&a| a < 61.0) && angles.iter().any(|&a| a > 119.0));

    let (a, b) = (
        RollingBallSide::new(&positions, &plate),
        RollingBallSide::new(&positions, &wall),
    );
    let stations = ball(&a, &b, None).walk(&points).unwrap().stations;
    let sagitta = cylinder_radius * (1.0 - (PI / sides as f64).cos());
    for s in &stations {
        assert!((s.centre.z - RADIUS).abs() <= EXACT, "{s:?}");
        let off_axis = (s.centre - (axis * s.centre.dot(axis))).length();
        assert!(
            off_axis <= cylinder_radius + RADIUS + EXACT
                && off_axis >= cylinder_radius + RADIUS - sagitta - EXACT,
            "{off_axis} from the axis: {s:?}"
        );
    }
}

/// A floor narrowing from 20 wide at y = -10 to a point at y = 10 (its rim x + y = 10), beside
/// the wall x = 0.
fn narrowing_floor() -> (RollingBallSide, RollingBallSide) {
    let mut positions = vec![
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(20.0, -10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ];
    let floor = vec![[0, 1, 2]];
    let wall = wall(&mut positions);
    (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &wall),
    )
}

#[test]
fn where_the_floor_narrows_past_the_radius_its_contact_moves_onto_the_rim() {
    // Where the floor is wider than the radius (y < 5) the ball sits at (5, y, 5). Past that its
    // floor contact is on the rim x + y = 10, which stands (y - 5) / sqrt 2 across from the
    // centre's foot, so the centre sinks to z = sqrt(r^2 - (y - 5)^2 / 2), still a radius off the
    // wall: at the point (y = 10), z = r / sqrt 2.
    let (floor, wall) = narrowing_floor();
    let stations = ball(&floor, &wall, None)
        .walk(&along_y(-8.0, 10.0, 37))
        .unwrap()
        .stations;
    assert_eq!(stations.len(), 37);
    for s in &stations {
        let y = s.chain_point.y;
        let past = if y > 5.0 { y - 5.0 } else { 0.0 };
        let height = ((RADIUS * RADIUS) - ((past * past) / 2.0)).sqrt();
        assert!(
            (s.centre - Vector3::new(RADIUS, y, height)).length() <= 1e-8,
            "at y = {y}: {s:?}"
        );
        // At y = 5 itself the contact stands on the rim and the floor alike.
        if (y - 5.0).abs() > 1e-6 {
            assert_eq!(
                s.contact_a.feature.is_rim(),
                y > 5.0,
                "at y = {y}: {:?}",
                s.contact_a.feature
            );
        }
        assert!(!s.contact_b.feature.is_rim());
    }
    assert!(matches!(
        stations[36].contact_a.feature,
        ContactFeature::Edge { rim: true, .. } | ContactFeature::Vertex { rim: true, .. }
    ));
}

#[test]
fn a_ball_that_would_reach_into_the_part_is_refused_with_what_it_reaches() {
    // A ceiling 8 above the floor, facing down, over the square corner: a ball of radius 5 in the
    // corner would reach to 10, 2 into it.
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        6,
        4,
    );
    let wall = wall(&mut positions);
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
    let part = RollingBallSide::new(&positions, &[floor, wall, ceiling].concat());
    let error = ball(&a, &b, Some(&part))
        .walk(&along_y(-4.0, 4.0, 9))
        .unwrap_err();
    let WalkError::At { index, failure } = error else {
        panic!("{error:?}");
    };
    assert_eq!(index, 4, "the walk starts mid-chain");
    let StationFailure::Fault {
        fault: StationFault::ReachesPastThePart { reach },
        ..
    } = failure
    else {
        panic!("{failure:?}");
    };
    assert!((reach.point.z - 8.0).abs() <= EXACT, "{reach:?}");
}

#[test]
fn a_side_that_wraps_round_the_other_is_read_only_on_its_own_side_of_the_corner() {
    // A floor 2 wide (x from 0 to 2) beside the wall x = 0, and side B going on past the floor's
    // far rim as a skirt hanging down from it (x = 2, z from 0 to -10, facing +x): B wraps round
    // A, so the rim at x = 2 is both sides' boundary. The ball rests on the wall and on the
    // floor's rim: centre (5, y, 4), 5 from the wall and from (2, y, 0), where B's wall and the
    // shared rim stand equally near. (Here the tie still falls to the wall when B is read whole;
    // the T Handle's junction chains are where reading whole fails outright.)
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        2,
        4,
    );
    let mut outer = rectangle(
        &mut positions,
        Vector3::new(2.0, -10.0, 0.0),
        Vector3::new(0.0, 0.0, -10.0),
        Vector3::new(0.0, 20.0, 0.0),
        2,
        4,
    );
    outer.extend(wall(&mut positions));
    let (a, b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &outer),
    );
    let stations = ball(&a, &b, None)
        .walk(&along_y(-6.0, 6.0, 13))
        .unwrap()
        .stations;
    for s in &stations {
        let y = s.chain_point.y;
        assert!(
            (s.centre - Vector3::new(RADIUS, y, 4.0)).length() <= 1e-8,
            "{s:?}"
        );
        assert!((s.contact_a.point - Vector3::new(2.0, y, 0.0)).length() <= 1e-8);
        assert!(s.contact_a.feature.is_rim());
        assert!((s.contact_b.point - Vector3::new(0.0, y, 4.0)).length() <= 1e-8);
    }
}

/// The end face y = `at` of a part whose solid is below the floor z = 0 and behind the wall x = 0
/// (x < 0 or z < 0), facing +y: two rectangles, under the floor and behind the wall.
fn end_face_at_y(positions: &mut Vec<Vector3>, at: f64) -> Vec<[i32; 3]> {
    let mut face = rectangle(
        positions,
        Vector3::new(0.0, at, -5.0),
        Vector3::new(0.0, 0.0, 5.0),
        Vector3::new(30.0, 0.0, 0.0),
        6,
        1,
    );
    face.extend(rectangle(
        positions,
        Vector3::new(-5.0, at, -5.0),
        Vector3::new(0.0, 0.0, 35.0),
        Vector3::new(5.0, 0.0, 0.0),
        7,
        1,
    ));
    face
}

#[test]
fn a_square_corner_ending_square_on_an_end_face_rolls_to_the_end_as_it_does_freely() {
    // Floor and wall both end at y = 8 on the part's end face. The free ball at the last station
    // already stands in the end plane, at (5, 8, 5), so holding it changes nothing.
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 18.0, 0.0),
        6,
        6,
    );
    let wall = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 18.0, 0.0),
        Vector3::new(0.0, 0.0, 30.0),
        6,
        6,
    );
    let end = end_face_at_y(&mut positions, 8.0);
    let (a, b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &wall),
    );
    let part = RollingBallSide::new(&positions, &[floor, wall, end].concat());
    let points = along_y(-8.0, 8.0, 17);
    let ball = ball(&a, &b, Some(&part));
    let held = ball
        .walk_ending_on(
            &points,
            [
                None,
                Some(EndFace {
                    point: Vector3::new(0.0, 8.0, 0.0),
                    normal: Vector3::new(0.0, 3.0, 0.0),
                }),
            ],
        )
        .unwrap()
        .stations;
    let free = ball.walk(&points).unwrap().stations;
    assert_eq!(held.len(), 17);
    for (h, f) in held.iter().zip(&free) {
        assert!(
            (h.centre - f.centre).length() <= EXACT,
            "{h:?} against {f:?}"
        );
    }
    let last = held[16];
    assert!(
        (last.centre - Vector3::new(RADIUS, 8.0, RADIUS)).length() <= EXACT,
        "{last:?}"
    );
    assert!((last.centre.y - 8.0).abs() <= EXACT);
    assert!((last.contact_a.distance - RADIUS).abs() <= EXACT);
    assert!((last.contact_b.distance - RADIUS).abs() <= EXACT);
}

#[test]
fn where_the_free_ball_would_leave_through_the_end_face_its_centre_is_held_in_the_end_plane() {
    // The floor z = 0 and a wall standing square on it but turned 30 degrees (normal
    // (cos b, sin b, 0)), so the chain - the line x = -y tan b on the floor - meets the end face
    // y = 0 at a slant, and the end face is square to the floor but not to the wall. The free
    // ball at chain point (-y tan b, y, 0) stands at r along both normals:
    // (-y tan b + r cos b, y + r sin b, r), which crosses the end plane wherever y > -r sin b.
    // There the centre is held in the plane, a radius above the floor and a radius off the
    // wall's plane: (r / cos b, 0, r) - where the free centres' line meets the plane, the spine
    // of the round the sides would give extended past the end face. It touches the floor on the
    // floor's trace in the end plane, (r / cos b, 0, 0), and the wall a radius back along its
    // normal, (r sin^2 b / cos b, -r sin b, r), short of the end face because the wall meets it
    // at a slant.
    let b = PI / 6.0;
    let (cos, sin, tan) = (b.cos(), b.sin(), b.tan());
    let mut positions = vec![
        Vector3::new(12.0 * tan, -12.0, 0.0),
        Vector3::new(30.0, -12.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(12.0 * tan, -12.0, 30.0),
        Vector3::new(0.0, 0.0, 30.0),
    ];
    let floor = vec![[0, 1, 2], [0, 2, 3]];
    let wall = vec![[0, 3, 5], [0, 5, 4]];
    let end = end_face_at_y(&mut positions, 0.0);
    let (side_a, side_b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &wall),
    );
    let part = RollingBallSide::new(&positions, &[floor, wall, end].concat());
    let points: Vec<ChainPoint> = (0..=20)
        .map(|i| {
            let y = -10.0 + (0.5 * i as f64);
            ChainPoint {
                point: Vector3::new(-y * tan, y, 0.0),
                tangent: Vector3::new(-sin, cos, 0.0),
                normal_a: Vector3::UNIT_Z,
                normal_b: Vector3::new(cos, sin, 0.0),
            }
        })
        .collect();
    let end_face = EndFace {
        point: Vector3::ZERO,
        normal: Vector3::new(0.0, -1.0, 0.0),
    };

    let stations = ball(&side_a, &side_b, Some(&part))
        .walk_ending_on(&points, [None, Some(end_face)])
        .unwrap()
        .stations;
    assert_eq!(stations.len(), 21);
    let held = Vector3::new(RADIUS / cos, 0.0, RADIUS);
    for (s, p) in stations.iter().zip(&points) {
        let y = p.point.y;
        assert!((s.chain_point - p.point).length() <= EXACT);
        let expected = if y > -RADIUS * sin {
            held
        } else {
            Vector3::new((-y * tan) + (RADIUS * cos), y + (RADIUS * sin), RADIUS)
        };
        assert!((s.centre - expected).length() <= EXACT, "at y = {y}: {s:?}");
    }
    let last = stations[20];
    assert!(
        (last.contact_a.point - Vector3::new(RADIUS / cos, 0.0, 0.0)).length() <= EXACT,
        "{last:?}"
    );
    // The floor's trace in the end plane, where the floor runs on into its extension past the end
    // face: an edge of the side read as extended, no rim.
    assert!(
        matches!(
            last.contact_a.feature,
            ContactFeature::Edge { rim: false, .. }
        ),
        "the floor's trace in the end plane: {last:?}"
    );
    let on_wall = Vector3::new(RADIUS * sin * sin / cos, -RADIUS * sin, RADIUS);
    assert!(
        (last.contact_b.point - on_wall).length() <= EXACT,
        "{last:?}"
    );

    // Walked free, the ball rolls out past the end face.
    let free = ball(&side_a, &side_b, None).walk(&points).unwrap().stations;
    assert!(free[20].centre.y > RADIUS * sin - EXACT, "{:?}", free[20]);
}

#[test]
fn a_chain_ending_on_a_floor_the_ball_would_reach_is_walked_to_the_floor() {
    // Two walls, x = 0 and y = 0, meet in a vertical chain standing on the floor z = 0. A ball in
    // the corner stands at (5, 5, z); below z = 5 it reaches into the floor, which refuses it when
    // the floor is only part of the part. As the face the chain ends on, the floor is no
    // obstacle, and the last centre, (5, 5, 0), lies in its plane.
    let mut positions = Vec::new();
    let wall_x = rectangle(
        &mut positions,
        Vector3::ZERO,
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 0.0, 20.0),
        4,
        4,
    );
    let wall_y = rectangle(
        &mut positions,
        Vector3::ZERO,
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(20.0, 0.0, 0.0),
        4,
        4,
    );
    let floor = rectangle(
        &mut positions,
        Vector3::ZERO,
        Vector3::new(20.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        4,
        4,
    );
    let (a, b) = (
        RollingBallSide::new(&positions, &wall_x),
        RollingBallSide::new(&positions, &wall_y),
    );
    let part = RollingBallSide::new(&positions, &[wall_x, wall_y, floor].concat());
    let points: Vec<ChainPoint> = (0..=10)
        .map(|i| ChainPoint {
            point: Vector3::new(0.0, 0.0, i as f64),
            tangent: Vector3::UNIT_Z,
            normal_a: Vector3::UNIT_X,
            normal_b: Vector3::UNIT_Y,
        })
        .collect();
    let ball = ball(&a, &b, Some(&part));

    let refused = ball.walk(&points).unwrap_err();
    let WalkError::At {
        failure:
            StationFailure::Fault {
                fault: StationFault::ReachesPastThePart { reach },
                ..
            },
        ..
    } = refused
    else {
        panic!("{refused:?}");
    };
    assert!(reach.point.z.abs() <= EXACT, "{reach:?}");

    let stations = ball
        .walk_ending_on(
            &points,
            [
                Some(EndFace {
                    point: Vector3::ZERO,
                    normal: Vector3::UNIT_Z,
                }),
                None,
            ],
        )
        .unwrap()
        .stations;
    assert_eq!(stations.len(), 11);
    for s in &stations {
        let z = s.chain_point.z;
        assert!(
            (s.centre - Vector3::new(RADIUS, RADIUS, z)).length() <= EXACT,
            "{s:?}"
        );
    }
    assert!(stations[0].centre.z.abs() <= EXACT);
}

#[test]
fn a_side_that_wraps_past_an_end_face_is_not_read_beyond_its_plane() {
    // The square corner ending on the end face y = 0, with the wall's side going on past the end
    // plane as a flap (x = 3, y from 1 to 10, facing -x), inside the wall's half of the corner and
    // 2 from where the ball stands. Read whole, the flap would be the wall side's nearest surface
    // for the last stations; beyond the end face it is no part of the corner, so every centre stays
    // a radius off the floor and the wall: (5, y, 5).
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        6,
        2,
    );
    let mut wall = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(0.0, 0.0, 30.0),
        2,
        6,
    );
    wall.extend(rectangle(
        &mut positions,
        Vector3::new(3.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(0.0, 9.0, 0.0),
        2,
        2,
    ));
    let (a, b) = (
        RollingBallSide::new(&positions, &floor),
        RollingBallSide::new(&positions, &wall),
    );
    let points = along_y(-8.0, 0.0, 9);
    let end = EndFace {
        point: Vector3::ZERO,
        normal: Vector3::UNIT_Y,
    };
    let stations = ball(&a, &b, None)
        .walk_ending_on(&points, [None, Some(end)])
        .unwrap()
        .stations;
    assert_eq!(stations.len(), 9);
    for s in &stations {
        let y = s.chain_point.y;
        assert!(
            (s.centre - Vector3::new(RADIUS, y, RADIUS)).length() <= EXACT,
            "{s:?}"
        );
        assert!((s.contact_b.point - Vector3::new(0.0, y, RADIUS)).length() <= EXACT);
    }
}
