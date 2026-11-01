//! The round running out at a free end (`WalkEnd::RunOut`), held to a hand answer: a boss, a
//! faceted cylinder of radius R about the x axis standing out of the wall x = 0, where the wall is
//! cut off at the line y = R, tangent to the cylinder at the tip (0, R, 0). The wall narrows to
//! nothing between the chain (the cylinder's foot, x = 0) and its rim y = R as the chain reaches
//! the tip, so the ball rolls on until its two contacts meet there. With the chain at angle phi
//! (the point (0, R sin phi, R cos phi), the tip at 90 degrees), the ball's centre is
//! (a, (R + r) sin phi, (R + r) cos phi): a radius out from the cylinder, and a = r off the wall
//! while the wall reaches past the centre's foot ((R + r) sin phi <= R), then a radius off the
//! wall's rim, a = sqrt(r^2 - ((R + r) sin phi - R)^2), which is 0 at the tip, where both contacts
//! are the tip. Past the tip the same wall mirrored widens again, and a chain going on there is
//! refused where the contacts meet. So is a chain going on past the tip to where the round comes
//! back resting across the corner's bisector plane, beyond what a ball grown afresh can see: there
//! only a ball rolled on from the meeting finds it.

use std::f64::consts::PI;

use geometry_processing::bevel::rolling_ball::{
    ChainPoint, RollingBall, RollingBallSide, StationFailure, StationFault, WalkEnd, WalkError,
};
use vector_math::Vector3;

/// The cylinder's radius.
const R: f64 = 20.0;

/// The ball's.
const RADIUS: f64 = 5.0;

/// The cylinder's facets: a multiple of four, so one vertex column stands at the tip.
const SIDES: usize = 64;

/// Exact answers, give or take the solve's own tolerance.
const EXACT: f64 = 1e-9;

/// The chain's vertex at facet column `k`: angle `k 2 pi / SIDES` from the z axis toward y.
fn phi(k: i32) -> f64 {
    (k as f64) * (2.0 * PI / SIDES as f64)
}

fn on_circle(k: i32, radius: f64, x: f64) -> Vector3 {
    Vector3::new(x, radius * phi(k).sin(), radius * phi(k).cos())
}

/// `[a, b, c]` wound so its normal points along `outward` at its centroid.
fn wound(positions: &[Vector3], t: [i32; 3], outward: impl Fn(Vector3) -> Vector3) -> [i32; 3] {
    let [a, b, c] = t.map(|i| positions[i as usize]);
    let normal = (b - a).cross(c - a);
    if normal.dot(outward((a + b + c) / 3.0)) >= 0.0 {
        t
    } else {
        [t[0], t[2], t[1]]
    }
}

/// The boss and the wall: the cylinder's wall from x = 0 to 30, all the way round, and the wall
/// x = 0 outside it from the chain's vertex at `k = -8` (-45 degrees) to the tip, cut off at
/// y = R; with `mirrored`, also its mirror in z = 0, which runs on from the tip to 225 degrees.
/// Returns the positions, the wall's triangles (side A) and the cylinder's (side B).
fn boss(mirrored: bool) -> (Vec<Vector3>, Vec<[i32; 3]>, Vec<[i32; 3]>) {
    let mut positions = Vec::new();
    let rows = 6;
    for j in 0..=rows {
        for k in 0..SIDES as i32 {
            positions.push(on_circle(k, R, 30.0 * (j as f64 / rows as f64)));
        }
    }
    let at = |k: i32, j: usize| (j * SIDES) as i32 + k.rem_euclid(SIDES as i32);
    let radial = |p: Vector3| Vector3::new(0.0, p.y, p.z);
    let mut cylinder = Vec::new();
    for j in 0..rows {
        for k in 0..SIDES as i32 {
            for t in [
                [at(k, j), at(k + 1, j), at(k + 1, j + 1)],
                [at(k, j), at(k + 1, j + 1), at(k, j + 1)],
            ] {
                cylinder.push(wound(&positions, t, radial));
            }
        }
    }

    // The wall's quarter beyond the chain, fanned from a point on its rim y = R far enough out
    // (z = 3R) to see the whole arc from -45 degrees to the tip.
    let mut wall = Vec::new();
    for sign in if mirrored { vec![1.0, -1.0] } else { vec![1.0] } {
        let point = |y: f64, z: f64| Vector3::new(0.0, y, sign * z);
        let base = positions.len() as i32;
        let start = on_circle(-8, R, 0.0);
        positions.extend([
            point(R, 3.0 * R),
            point(-30.0, start.z),
            point(-30.0, 3.0 * R),
        ]);
        let (fan, low, far) = (base, base + 1, base + 2);
        let arc = |k: i32| {
            at(
                if sign > 0.0 {
                    k
                } else {
                    (SIDES / 2) as i32 - k
                },
                0,
            )
        };
        let facing = |_: Vector3| Vector3::UNIT_X;
        for k in -8..16 {
            wall.push(wound(&positions, [fan, arc(k), arc(k + 1)], facing));
        }
        wall.push(wound(&positions, [fan, low, arc(-8)], facing));
        wall.push(wound(&positions, [fan, far, low], facing));
    }

    (positions, wall, cylinder)
}

/// The chain's points at facet columns `from` to `to`: the cylinder's foot, its tangent the
/// circle's, side A the wall (+x) and side B the cylinder (radial).
fn chain(from: i32, to: i32) -> Vec<ChainPoint> {
    (from..=to)
        .map(|k| ChainPoint {
            point: on_circle(k, R, 0.0),
            tangent: Vector3::new(0.0, phi(k).cos(), -phi(k).sin()),
            normal_a: Vector3::UNIT_X,
            normal_b: Vector3::new(0.0, phi(k).sin(), phi(k).cos()),
        })
        .collect()
}

/// The hand answer's centre at angle `angle`.
fn hand_centre(angle: f64) -> Vector3 {
    let (y, z) = ((R + RADIUS) * angle.sin(), (R + RADIUS) * angle.cos());
    let past = if y > R { y - R } else { 0.0 };
    Vector3::new(((RADIUS * RADIUS) - (past * past)).sqrt(), y, z)
}

#[test]
fn where_the_wall_narrows_to_the_tip_the_round_runs_out_to_nothing_there() {
    let (positions, wall, cylinder) = boss(false);
    let (a, b) = (
        RollingBallSide::new(&positions, &wall),
        RollingBallSide::new(&positions, &cylinder),
    );
    let part = RollingBallSide::new(&positions, &[wall, cylinder].concat());
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: Some(&part),
        radius: RADIUS,
        convex: false,
    };
    let points = chain(-8, 16);
    let walk = ball.walk(&points).unwrap();
    assert_eq!(
        walk.ends,
        [WalkEnd::LastStation, WalkEnd::RunOut { past: 1 }],
        "{walk:?}"
    );
    assert_eq!(walk.stations.len(), points.len());

    // Every chain point short of the tip has the hand answer's ball.
    for (s, k) in walk.stations.iter().zip(-8..16) {
        assert!((s.chain_point - points[(k + 8) as usize].point).length() <= EXACT);
        assert!(
            (s.centre - hand_centre(phi(k))).length() <= EXACT,
            "at {} degrees: {s:?}",
            phi(k).to_degrees()
        );
    }

    // The end station is the tip's, with no width: both contacts the tip, the centre a radius out
    // from it along y. Where the contacts meet, the distances grow only as the square of the
    // ball's offset there over twice the radius, so a solve held to 1e-11 (1 + r) on the distances
    // places that centre only to within the square root of twice the radius times that.
    let end = walk.stations[points.len() - 1];
    let tip = Vector3::new(0.0, R, 0.0);
    let meeting = (2.0 * RADIUS * (1e-11 * (1.0 + RADIUS))).sqrt();
    assert!((end.chain_point - tip).length() <= EXACT, "{end:?}");
    assert_eq!(end.contact_a.point, end.chain_point);
    assert_eq!(end.contact_b.point, end.chain_point);
    assert_eq!(end.contact_a.distance, RADIUS);
    assert_eq!(end.contact_b.distance, RADIUS);
    assert!(((end.centre - end.chain_point).length() - RADIUS).abs() <= EXACT);
    assert!(
        (end.centre - hand_centre(PI / 2.0)).length() <= meeting,
        "{end:?}"
    );
}

#[test]
fn contacts_meeting_where_the_round_widens_again_past_them_are_refused() {
    // The same boss with the wall mirrored past the tip, and the chain going on round to 135
    // degrees: the contacts meet at the tip, but the round comes back beyond it, so the tip is no
    // end of the chain and the walk is refused there.
    let (positions, wall, cylinder) = boss(true);
    let (a, b) = (
        RollingBallSide::new(&positions, &wall),
        RollingBallSide::new(&positions, &cylinder),
    );
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: None,
        radius: RADIUS,
        convex: false,
    };
    let points = chain(-8, 24);
    let error = ball.walk(&points).unwrap_err();
    let WalkError::At { index, .. } = error else {
        panic!("{error:?}");
    };
    assert_eq!(index, 24, "refused at the tip: {error:?}");

    // Walked only to the tip, the same chain runs out there.
    let to_the_tip = ball.walk(&points[..25]).unwrap();
    assert_eq!(to_the_tip.ends[1], WalkEnd::RunOut { past: 1 });
}

#[test]
fn where_the_chain_starts_at_the_tip_the_round_runs_out_at_its_first_end() {
    // The same boss and chain walked the other way, from the tip to -45 degrees: the run-out is
    // the chain's first end, its zero-width station first in the walk, ahead of every station the
    // walk solved.
    let (positions, wall, cylinder) = boss(false);
    let (a, b) = (
        RollingBallSide::new(&positions, &wall),
        RollingBallSide::new(&positions, &cylinder),
    );
    let part = RollingBallSide::new(&positions, &[wall, cylinder].concat());
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: Some(&part),
        radius: RADIUS,
        convex: false,
    };
    let points: Vec<ChainPoint> = chain(-8, 16)
        .into_iter()
        .rev()
        .map(|p| ChainPoint {
            tangent: -p.tangent,
            ..p
        })
        .collect();
    let walk = ball.walk(&points).unwrap();
    assert_eq!(
        walk.ends,
        [WalkEnd::RunOut { past: 1 }, WalkEnd::LastStation],
        "{walk:?}"
    );
    assert_eq!(walk.stations.len(), points.len());

    let end = walk.stations[0];
    let tip = Vector3::new(0.0, R, 0.0);
    let meeting = (2.0 * RADIUS * (1e-11 * (1.0 + RADIUS))).sqrt();
    assert!((end.chain_point - tip).length() <= EXACT, "{end:?}");
    assert_eq!(end.contact_a.point, end.chain_point);
    assert_eq!(end.contact_b.point, end.chain_point);
    assert!(
        (end.centre - hand_centre(PI / 2.0)).length() <= meeting,
        "{end:?}"
    );

    // Every station after it is the hand answer's, in the chain's order.
    for (s, k) in walk.stations[1..].iter().zip((-8..16).rev()) {
        assert!((s.chain_point - on_circle(k, R, 0.0)).length() <= EXACT);
        assert!(
            (s.centre - hand_centre(phi(k))).length() <= EXACT,
            "at {} degrees: {s:?}",
            phi(k).to_degrees()
        );
    }
}

/// How steeply side B falls away from the line y = R past the tip: a flat facet from the tip's
/// column, tilted down by this angle.
const FALL_AWAY: f64 = PI / 8.0;

/// The ceiling's rise across the boss (along x) past the tip, and how fast it sinks along the
/// chain once it starts.
const SLOPE: f64 = 0.5;
const FALL: f64 = 0.5;

/// Where along the chain past the tip the wall starts to rise off the line y = R, the ceiling
/// starts, the far chain point stands and the surfaces end.
const RISE_FROM: i32 = 6;
const CEILING_FROM: i32 = 10;
const FAR: f64 = 22.0;
const RETURN_END: i32 = 28;

/// The point `t` along the chain past the tip, `n` off side B's facet and `x` along the boss.
fn past(t: f64, n: f64, x: f64) -> Vector3 {
    let along = Vector3::new(0.0, -FALL_AWAY.sin(), -FALL_AWAY.cos());
    let off = Vector3::new(0.0, FALL_AWAY.cos(), -FALL_AWAY.sin());
    on_circle(16, R, x) + (along * t) + (off * n)
}

/// The wall's height off side B's facet at `t` past the tip: up to `RISE_FROM` its rim is the line
/// y = R, tangent to the zero-width ball at the tip, so the wall widens from nothing only as B
/// falls away; then it rises to 10, stays level, and from `CEILING_FROM` on sinks at `FALL`.
fn wall_height(t: i32) -> f64 {
    let along_the_line = FALL_AWAY.tan() * (t as f64);
    let rise_from = FALL_AWAY.tan() * (RISE_FROM as f64);
    if t <= RISE_FROM {
        along_the_line
    } else if t < CEILING_FROM - 1 {
        rise_from + ((10.0 - rise_from) * ((t - RISE_FROM) as f64 / 3.0))
    } else if t <= CEILING_FROM {
        10.0
    } else {
        10.0 - (FALL * (t - CEILING_FROM) as f64)
    }
}

/// The boss as `boss(false)` builds it up to the tip, and past the tip a part where the round comes
/// back with its wall contact across the corner's bisector plane. Up to the tip: the cylinder's
/// columns from -67.5 to 90 degrees and the wall x = 0 cut off at y = R. Past it, at `t` along the
/// chain: side B a flat facet falling away from the tip's column (`FALL_AWAY`), side A the wall
/// x = 0 from B up to `wall_height(t)` off it, zero at the tip, so the wall narrows to nothing
/// there from both ways, and folding at `CEILING_FROM` into a ceiling `wall_height(t) + SLOPE x`
/// off B. The chain past the tip is one edge, the tip to the wall's foot vertex at `FAR`. Returns
/// the positions, side A's triangles and side B's.
fn boss_with_a_return() -> (Vec<Vector3>, Vec<[i32; 3]>, Vec<[i32; 3]>) {
    let (first, last, rows) = (-12, 16, 6usize);
    let columns = (last - first + 1) as usize;
    let mut positions = Vec::new();
    for j in 0..=rows {
        for k in first..=last {
            positions.push(on_circle(k, R, 5.0 * j as f64));
        }
    }
    let at = |k: i32, j: usize| (j * columns) as i32 + (k - first);
    let radial = |p: Vector3| Vector3::new(0.0, p.y, p.z);
    let mut side_b = Vec::new();
    for j in 0..rows {
        for k in first..last {
            for t in [
                [at(k, j), at(k + 1, j), at(k + 1, j + 1)],
                [at(k, j), at(k + 1, j + 1), at(k, j + 1)],
            ] {
                side_b.push(wound(&positions, t, radial));
            }
        }
    }

    // The wall up to the tip, fanned from its rim as `boss` fans it.
    let facing = |_: Vector3| Vector3::UNIT_X;
    let mut side_a = Vec::new();
    let base = positions.len() as i32;
    let start = on_circle(-8, R, 0.0);
    positions.extend([
        Vector3::new(0.0, R, 3.0 * R),
        Vector3::new(0.0, -30.0, start.z),
        Vector3::new(0.0, -30.0, 3.0 * R),
    ]);
    let (fan, low, far) = (base, base + 1, base + 2);
    for k in -8..16 {
        side_a.push(wound(&positions, [fan, at(k, 0), at(k + 1, 0)], facing));
    }
    side_a.push(wound(&positions, [fan, low, at(-8, 0)], facing));
    side_a.push(wound(&positions, [fan, far, low], facing));

    // Past the tip: the wall's rim at each whole t (the tip itself at 0), its foot at the far
    // chain point and at the end, the wall fanned from the far foot (every rim point in sight of
    // it: the rim rises from the tip and falls no lower than its height at the far foot).
    let tip = at(16, 0);
    let rim_base = positions.len() as i32;
    for t in 1..=RETURN_END {
        positions.push(past(t as f64, wall_height(t), 0.0));
    }
    let rim = |t: i32| if t == 0 { tip } else { rim_base + t - 1 };
    let foot_far = positions.len() as i32;
    positions.push(past(FAR, 0.0, 0.0));
    let foot_end = positions.len() as i32;
    positions.push(past(RETURN_END as f64, 0.0, 0.0));
    for t in 0..RETURN_END {
        side_a.push(wound(&positions, [foot_far, rim(t), rim(t + 1)], facing));
    }
    side_a.push(wound(
        &positions,
        [foot_far, rim(RETURN_END), foot_end],
        facing,
    ));

    // The ceiling from the fold out over B, facing down onto it.
    let ceiling_base = positions.len() as i32;
    let width = 30;
    for t in CEILING_FROM..=RETURN_END {
        for x in 1..=width {
            let x = x as f64;
            positions.push(past(t as f64, wall_height(t) + (SLOPE * x), x));
        }
    }
    let ceiling = |x: i32, t: i32| {
        if x == 0 {
            rim(t)
        } else {
            ceiling_base + ((t - CEILING_FROM) * width) + x - 1
        }
    };
    let down = past(-FALL, -1.0, SLOPE) - past(0.0, 0.0, 0.0);
    for t in CEILING_FROM..RETURN_END {
        for x in 0..width {
            for tri in [
                [ceiling(x, t), ceiling(x + 1, t), ceiling(x + 1, t + 1)],
                [ceiling(x, t), ceiling(x + 1, t + 1), ceiling(x, t + 1)],
            ] {
                side_a.push(wound(&positions, tri, |_| down));
            }
        }
    }

    // Side B past the tip: the facet from the tip's column out, in rows at the far foot and the
    // end, sharing the tip column's vertices with the cylinder.
    let plane_base = positions.len() as i32;
    for t in [FAR, RETURN_END as f64] {
        for j in 0..=rows {
            positions.push(past(t, 0.0, 5.0 * j as f64));
        }
    }
    let plane = |j: usize, row: usize| match row {
        0 => at(16, j),
        _ => plane_base + (((row - 1) * (rows + 1)) + j) as i32,
    };
    let up = past(0.0, 1.0, 0.0) - past(0.0, 0.0, 0.0);
    for row in 0..2 {
        for j in 0..rows {
            for tri in [
                [plane(j, row), plane(j + 1, row), plane(j + 1, row + 1)],
                [plane(j, row), plane(j + 1, row + 1), plane(j, row + 1)],
            ] {
                side_b.push(wound(&positions, tri, |_| up));
            }
        }
    }

    (positions, side_a, side_b)
}

/// The chain point `t` past the tip, on B's facet along the wall's foot.
fn past_the_tip(t: f64) -> ChainPoint {
    let origin = past(0.0, 0.0, 0.0);
    ChainPoint {
        point: past(t, 0.0, 0.0),
        tangent: past(1.0, 0.0, 0.0) - origin,
        normal_a: Vector3::UNIT_X,
        normal_b: past(0.0, 1.0, 0.0) - origin,
    }
}

/// The hand answer `t` past the tip once the ceiling has pushed the ball out along B's facet:
/// (centre, ceiling contact). Off the facet n, along the boss x and along the chain t, the ceiling
/// is n = h + SLOPE x with h = 10 - FALL (t - 10), its unit normal toward it
/// g = (-SLOPE, 1, FALL) / N (in x, n, t). The centre is a radius off the facet, in the station
/// plane, and a radius below the ceiling: h + SLOPE x - r = r N, so x = (r N + r - h) / SLOPE.
fn hand_return(t: f64) -> (Vector3, Vector3) {
    let h = 10.0 - (FALL * (t - CEILING_FROM as f64));
    let n = ((SLOPE * SLOPE) + 1.0 + (FALL * FALL)).sqrt();
    let x = ((RADIUS * n) + RADIUS - h) / SLOPE;
    let contact = (
        t + (RADIUS * FALL / n),
        RADIUS + (RADIUS / n),
        x - (RADIUS * SLOPE / n),
    );
    (past(t, RADIUS, x), past(contact.0, contact.1, contact.2))
}

#[test]
fn a_round_that_comes_back_past_its_meeting_across_the_bisector_is_refused() {
    // The contacts meet at the tip as on the plain boss, and past it the round comes back: B falls
    // away from the tip as a flat facet, and the wall, widening from nothing there, rises and
    // folds into a ceiling that sinks onto B, pushing the ball out along it, so at the far chain
    // point (t = 22) the ball rests on the ceiling 12.2 along the boss and 9.1 off B: across the
    // corner's bisector plane. A ball grown afresh there reads the wall only within its half of
    // the corner and finds none; a ball rolled on from the tip, its contacts followed up the wall
    // onto the ceiling, does. The tip is no end of the chain.
    let (positions, side_a, side_b) = boss_with_a_return();
    let (a, b) = (
        RollingBallSide::new(&positions, &side_a),
        RollingBallSide::new(&positions, &side_b),
    );
    let ball = RollingBall {
        side_a: &a,
        side_b: &b,
        part: None,
        radius: RADIUS,
        convex: false,
    };

    // The contacts do meet at the tip: walked only to it, the chain runs out there.
    let walked = ball.walk(&chain(-8, 16)).unwrap();
    assert_eq!(walked.ends[1], WalkEnd::RunOut { past: 1 }, "{walked:?}");

    // At the far point the ball is there, rolled on from a neighbour: walked along B's foot from
    // t = 12, the station at 22 is the hand answer, its contact across the bisector plane.
    let along: Vec<ChainPoint> = (12..=22).map(|t| past_the_tip(t as f64)).collect();
    let rolled = ball.walk(&along).unwrap();
    let far = rolled.stations[along.len() - 1];
    let (centre, on_ceiling) = hand_return(FAR);
    assert!((far.centre - centre).length() <= EXACT, "{far:?}");
    assert!(
        (far.contact_a.point - on_ceiling).length() <= EXACT,
        "{far:?}"
    );
    let off_b = (far.contact_a.point - far.chain_point).dot(past_the_tip(FAR).normal_b);
    assert!(
        far.contact_a.point.x > off_b,
        "the contact lies across the bisector plane: {far:?}"
    );

    // Grown afresh there, it is not found: the wall's half of the corner ends at the bisector
    // plane short of the contact, and the ball the seed settles on touches the wall only where
    // that plane cuts it.
    let seeded = ball.grown_station(&past_the_tip(FAR));
    assert!(
        matches!(
            seeded,
            Err(StationFailure::Fault {
                fault: StationFault::OnTheCut { side_b: false },
                ..
            })
        ),
        "{seeded:?}"
    );

    // The whole chain, on past the tip to the far point, is refused at the tip.
    let mut points = chain(-8, 16);
    points.push(past_the_tip(FAR));
    let error = ball.walk(&points).unwrap_err();
    let WalkError::At { index, .. } = error else {
        panic!("{error:?}");
    };
    assert_eq!(index, 24, "refused at the tip: {error:?}");
}
