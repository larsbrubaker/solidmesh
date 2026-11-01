//! The growing crease runs' pierce room read from the rolling ball's centres
//! (`level_line::ball_pierce_room::BallPath`), held to hand answers on the rib of the strip clip
//! tests: the top front edge of a box, the chain from (0, 0, 10) to (20, 0, 10), and the rib's
//! ridge from its tip T (10, 3, 10) up to R (10, 8, 20).

use geometry_processing::bevel::level_line::ball_pierce_room::BallPath;
use vector_math::Vector3;

const CHAIN: [Vector3; 2] = [Vector3::new(0.0, 0.0, 10.0), Vector3::new(20.0, 0.0, 10.0)];

const T: Vector3 = Vector3::new(10.0, 3.0, 10.0);

const R: Vector3 = Vector3::new(10.0, 8.0, 20.0);

fn path(unit_centres: &[Vector3; 2]) -> BallPath<'_> {
    BallPath {
        chain_points: &CHAIN,
        unit_centres,
        closed: false,
        carried: [false, false],
        size: 7.9,
    }
    .checked()
    .expect("one centre per chain point")
}

/// The ball's centre at the box's square edge, (x, s, 10 - s): the ridge's line runs
/// (10, 3 + 5 t, 10 + 10 t), so its distance from the axis is |3 s - 6| / sqrt 5, which leaves the
/// true round (no chords) at s = 6 / (3 - sqrt 5) = 7.854 and the round of n chords to the quarter,
/// whose chords come in to s cos(pi / 2n), at 6 / (3 - cos(pi / 2n) sqrt 5).
#[test]
fn the_ridges_line_leaves_the_balls_round_at_the_hand_answer() {
    let square = [Vector3::new(0.0, 1.0, -1.0), Vector3::new(0.0, 1.0, -1.0)];
    let root5 = 5.0_f64.sqrt();
    let room = path(&square).pierce_room(&[(T, R)], 0);
    assert!((room - (6.0 / (3.0 - root5))).abs() < 1e-12, "room {room}");
    let inscribed = (std::f64::consts::PI / 16.0).cos();
    let room = path(&square).pierce_room(&[(T, R)], 8);
    let expected = 6.0 / (3.0 - (inscribed * root5));
    assert!(
        (room - expected).abs() < 1e-12,
        "room {room}, not {expected}"
    );
}

/// Where a side has moved the ball's contact, the axis is where the ball's centre is, not where
/// the square edge's angle puts it: a centre per unit of (0, 1.5, -1) moves across the ridge's
/// plane 4 / sqrt 5 per unit of size, so the ridge's line leaves the round at 6 / (4 - sqrt 5).
#[test]
fn the_axis_runs_through_the_balls_own_centres() {
    let moved = [Vector3::new(0.0, 1.5, -1.0), Vector3::new(0.0, 1.5, -1.0)];
    let room = path(&moved).pierce_room(&[(T, R)], 0);
    let expected = 6.0 / (4.0 - 5.0_f64.sqrt());
    assert!(
        (room - expected).abs() < 1e-12,
        "room {room}, not {expected}"
    );
}

/// Where the leg's two centres differ the axis tilts with the size: per unit (0, 1, -1) at the
/// chain's first end and (0, 2, -1) at its last, the axis at size s runs from (0, s, 10 - s) to
/// (20, 2 s, 10 - s), direction (20, s, 0). With the ridge's direction (0, 1, 2), n = (0, 1, 2) x
/// (20, s, 0) = (-2 s, 40, -20), and the axis point less T is (-10, s - 3, -s), so the ridge's line
/// stands (80 s - 120) / sqrt(4 s^2 + 2000) from the axis. It leaves the true round where that is
/// s: s^4 - 1100 s^2 + 4800 s - 3600 = 0, which changes sign once between 3 and 3.5 (s = 3.454),
/// halved here to the last bit. (The axis carried along the leg through the centre interpolated at
/// T's foot, (0, 1.5, -1), read 6 / (4 - sqrt 5) = 3.403: the tilt moves the root, so the pinned
/// value changed deliberately with the true axis.)
#[test]
fn the_axis_tilts_through_the_two_centres_of_its_leg() {
    let varying = [Vector3::new(0.0, 1.0, -1.0), Vector3::new(0.0, 2.0, -1.0)];
    let room = path(&varying).pierce_room(&[(T, R)], 0);
    let quartic = |s: f64| (s * s * s * s) - (1100.0 * s * s) + (4800.0 * s) - 3600.0;
    let (mut inside, mut outside) = (3.0, 3.5);
    assert!(quartic(inside) > 0.0 && quartic(outside) < 0.0);
    for _ in 0..200 {
        let middle = 0.5 * (inside + outside);
        if quartic(middle) > 0.0 {
            inside = middle;
        } else {
            outside = middle;
        }
    }
    assert!((room - outside).abs() < 1e-9, "room {room}, not {outside}");
    assert!((outside - 3.454).abs() < 1e-3);
}

/// The ridge's line through `start` and `far` against the axis line through `from` and `to`: the
/// distance between the two lines, and where the axis comes nearest as a share of `from`-`to`.
fn line_gap(start: Vector3, far: Vector3, from: Vector3, to: Vector3) -> (f64, f64) {
    let ridge = (far - start).get_normal();
    let axis = to - from;
    let normal = ridge.cross(axis);
    let distance = ((from - start).dot(normal) / normal.length()).abs();
    let offset = start - from;
    let (a, b, c) = (ridge.dot(ridge), ridge.dot(axis), axis.dot(axis));
    let (d, e) = (ridge.dot(offset), axis.dot(offset));
    (distance, ((a * e) - (b * d)) / ((a * c) - (b * b)))
}

/// The least size, scanned in steps of 0.001 up to 40 and halved to the last bit, at which the
/// ridge's line leaves the true round about the axis `axis(s)` gives (its distance passing s from
/// below), where `keep` accepts the size and the axis's nearest-approach share there.
fn first_exit(
    axis: impl Fn(f64) -> (Vector3, Vector3),
    start: Vector3,
    far: Vector3,
    keep: impl Fn(f64, f64) -> bool,
) -> f64 {
    let gap = |s: f64| {
        let (from, to) = axis(s);
        let (distance, along) = line_gap(start, far, from, to);
        (distance - s, along)
    };
    let mut before = 0.001;
    for step in 2..=40_000 {
        let s = f64::from(step) * 0.001;
        if gap(before).0 <= 0.0 && gap(s).0 > 0.0 {
            let (mut inside, mut outside) = (before, s);
            for _ in 0..200 {
                let middle = 0.5 * (inside + outside);
                if gap(middle).0 > 0.0 {
                    outside = middle;
                } else {
                    inside = middle;
                }
            }
            if keep(outside, gap(outside).1) {
                return outside;
            }
        }
        before = s;
    }
    f64::INFINITY
}

/// A two-leg chain, (0, 0, 10) to (10, 0, 10) to (20, 0, 10), the centres per unit (0, 1, -1) at
/// the first two vertices and (0, 2, -1) at the last. The ridge starts beside the first leg,
/// (9.5, 3, 10), but leans so far along the chain, to (-18, 8, 20), that its line comes nearest the
/// round over the second leg (within it, so no end is carried): the second leg's tilted axis sets
/// the room; the first leg's, read within its own span and the second's, sets none lower.
#[test]
fn a_ridge_is_read_against_the_leg_its_line_comes_nearest() {
    let chain = [
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(10.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
    ];
    let centres = [
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, 2.0, -1.0),
    ];
    let (start, far) = (Vector3::new(9.5, 3.0, 10.0), Vector3::new(-18.0, 8.0, 20.0));
    let axis = |i: usize| {
        move |s: f64| {
            (
                chain[i] + (centres[i] * s),
                chain[i + 1] + (centres[i + 1] * s),
            )
        }
    };
    let second = first_exit(axis(1), start, far, |_, along| {
        (-1.0..=1.0).contains(&along)
    });
    let first = first_exit(axis(0), start, far, |_, along| (0.0..=2.0).contains(&along));
    let (from, to) = axis(1)(second);
    let along = line_gap(start, far, from, to).1;
    assert!(
        (0.0..=1.0).contains(&along),
        "within the second leg: {along}"
    );
    assert!(second.is_finite() && second < first, "{second} {first}");
    let path = BallPath {
        chain_points: &chain,
        unit_centres: &centres,
        closed: false,
        carried: [false, false],
        size: 7.9,
    };
    let room = path.pierce_room(&[(start, far)], 0);
    assert!((room - second).abs() < 1e-9, "room {room}, not {second}");
}

/// One leg whose centres differ, (0, 1, -1) and (0, 2, -1), and a ridge from (`x`, 3, 10) to
/// (`x`, 8, 20) past the chain's last end, read with that end carried as `carried`.
fn past_the_end(x: f64, carried: bool) -> f64 {
    let varying = [Vector3::new(0.0, 1.0, -1.0), Vector3::new(0.0, 2.0, -1.0)];
    BallPath {
        chain_points: &CHAIN,
        unit_centres: &varying,
        closed: false,
        carried: [false, carried],
        size: 7.9,
    }
    .pierce_room(
        &[(Vector3::new(x, 3.0, 10.0), Vector3::new(x, 8.0, 20.0))],
        0,
    )
}

/// Past an end with an end ray the fill carries the round on until its rows have left the end
/// plane, plus a quarter setback: a setback is at most s |c|, so the carry is read up to
/// s |c| (1 + 0.25) past the end, |c| = sqrt 5 at the last end, as a share of the tilted axis's
/// length sqrt(400 + s^2). The ridge 1 past the end, at x = 21, reads the root where its line
/// leaves that carried round. Past an unrayed end the fill builds no round (the end face is cut
/// flat), so the same ridge sets no room; nor does a ridge 20 past a rayed end, beyond the carry.
#[test]
fn a_ridge_past_a_rayed_end_is_read_against_the_carried_round() {
    let varying = [Vector3::new(0.0, 1.0, -1.0), Vector3::new(0.0, 2.0, -1.0)];
    let axis = |s: f64| (CHAIN[0] + (varying[0] * s), CHAIN[1] + (varying[1] * s));
    let reach = |s: f64| 1.0 + (s * 5.0_f64.sqrt() * 1.25 / (400.0 + (s * s)).sqrt());
    let carried_at = |x: f64| {
        let (start, far) = (Vector3::new(x, 3.0, 10.0), Vector3::new(x, 8.0, 20.0));
        first_exit(axis, start, far, |s, along| {
            along > 1.0 && along <= reach(s)
        })
    };
    let carried = carried_at(21.0);
    assert!(carried.is_finite());
    let room = past_the_end(21.0, true);
    assert!((room - carried).abs() < 1e-9, "room {room}, not {carried}");
    assert_eq!(carried_at(40.0), f64::INFINITY);
    assert_eq!(past_the_end(40.0, true), f64::INFINITY);
}

/// The unrayed twin: the end face is cut flat, nothing runs past it, no room.
#[test]
fn a_ridge_past_an_unrayed_end_sets_no_room() {
    assert_eq!(past_the_end(21.0, false), f64::INFINITY);
}

/// A three-leg chain whose middle leg, (0, 0, 10) to (20, 0, 10), tilts a little (centres per unit
/// (0, 1, -1) and (0, 1.001, -1)), and a ridge from (10, 3, 13) all but parallel to it, along
/// (1, 0.005, 0.01). The two lines do part past a radius (the exit found here), but only where they
/// come nearest, far off the middle leg; the outer legs, square to it, stand 2.9 or more plus the
/// size from the ridge's line. No room.
#[test]
fn a_ridge_all_but_parallel_to_a_tilted_leg_sets_no_room() {
    let chain = [
        Vector3::new(0.0, 10.0, 10.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
        Vector3::new(20.0, 10.0, 10.0),
    ];
    let centres = [
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, 1.001, -1.0),
        Vector3::new(0.0, 1.001, -1.0),
    ];
    let start = Vector3::new(10.0, 3.0, 13.0);
    let far = start + Vector3::new(1.0, 0.005, 0.01);
    let middle = |s: f64| (chain[1] + (centres[1] * s), chain[2] + (centres[2] * s));
    let anywhere = first_exit(middle, start, far, |_, _| true);
    assert!(anywhere.is_finite(), "the lines part somewhere");
    let (_, along) = line_gap(start, far, middle(anywhere).0, middle(anywhere).1);
    assert!(!(0.0..=1.0).contains(&along), "nearest at {along}");
    let path = BallPath {
        chain_points: &chain,
        unit_centres: &centres,
        closed: false,
        carried: [false, false],
        size: 7.9,
    };
    assert_eq!(path.pierce_room(&[(start, far)], 0), f64::INFINITY);
}

/// The rib's ridge read with the leg's two centres equal and one bit apart gives one answer: the
/// linear reading and the quartic agree where the axis barely tilts.
#[test]
fn equal_centres_and_centres_a_bit_apart_read_one_room() {
    let equal = [Vector3::new(0.0, 1.0, -1.0), Vector3::new(0.0, 1.0, -1.0)];
    let apart = [
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, f64::from_bits(1.0_f64.to_bits() + 1), -1.0),
    ];
    let (same, near) = (
        path(&equal).pierce_room(&[(T, R)], 0),
        path(&apart).pierce_room(&[(T, R)], 0),
    );
    assert!((same - (6.0 / (3.0 - 5.0_f64.sqrt()))).abs() < 1e-12);
    assert!((same - near).abs() < 1e-9, "{same} and {near}");
}

/// The distance from the ridge's line through `start` and `far` to the polyline through
/// `points` (closed when `closed`): each segment's nearest point to the line, its parameter the
/// minimum of a quadratic clamped to the segment.
fn polyline_gap(start: Vector3, far: Vector3, points: &[Vector3], closed: bool) -> f64 {
    let ridge = (far - start).get_normal();
    let n = points.len();
    let segments = if closed { n } else { n - 1 };
    let mut least = f64::INFINITY;
    for i in 0..segments {
        let (p, q) = (points[i], points[(i + 1) % n]);
        let (a, b) = ((p - start).cross(ridge), (q - p).cross(ridge));
        let t = if b.length_squared() > 0.0 {
            (-a.dot(b) / b.length_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        least = least.min((a + (b * t)).length());
    }
    least
}

/// The least size, scanned in steps of 0.001 up to 40 and halved to the last bit, at which the
/// ridge's line leaves the true round about the polyline of centres `centres(s)`: its distance
/// from that polyline passing s from below.
fn first_polyline_exit(
    centres: impl Fn(f64) -> Vec<Vector3>,
    closed: bool,
    start: Vector3,
    far: Vector3,
) -> f64 {
    let gap = |s: f64| polyline_gap(start, far, &centres(s), closed) - s;
    let mut before = 0.001;
    for step in 2..=40_000 {
        let s = f64::from(step) * 0.001;
        if gap(before) <= 0.0 && gap(s) > 0.0 {
            let (mut inside, mut outside) = (before, s);
            for _ in 0..200 {
                let middle = 0.5 * (inside + outside);
                if gap(middle) > 0.0 {
                    outside = middle;
                } else {
                    inside = middle;
                }
            }
            return outside;
        }
        before = s;
    }
    f64::INFINITY
}

/// The top rim of a cylinder of radius 20 in 256 facets (legs about 0.49), the ball's centre per
/// unit 1 in and 1 down at each vertex, and a ridge rising from 3 inside the rim, (5 in, 10 up)
/// per step as the rib's, leaning 25 degrees along the rim, so its line comes nearest the round
/// several legs from the leg beside its start. Expected by geometry alone: the size at which the
/// ridge's line leaves the round about the whole polyline of centres. It lies 4 legs away, between
/// two legs where each leg's own straight axis misses it and the vertex between them holds it.
#[test]
fn a_ridge_leaning_along_a_faceted_rim_is_read_legs_away() {
    let facets = 256;
    let angle = |i: usize| std::f64::consts::TAU * (i as f64) / f64::from(facets);
    let chain: Vec<Vector3> = (0..facets as usize)
        .map(|i| Vector3::new(20.0 * angle(i).cos(), 20.0 * angle(i).sin(), 10.0))
        .collect();
    let centres: Vec<Vector3> = (0..facets as usize)
        .map(|i| Vector3::new(-angle(i).cos(), -angle(i).sin(), -1.0))
        .collect();
    let beside = 10;
    let middle = angle(beside) + (0.5 * std::f64::consts::TAU / f64::from(facets));
    let inward = Vector3::new(-middle.cos(), -middle.sin(), 0.0);
    let along = Vector3::new(-middle.sin(), middle.cos(), 0.0);
    let start = Vector3::new(17.0 * middle.cos(), 17.0 * middle.sin(), 10.0);
    let lean = 125.0_f64.sqrt() * 25.0_f64.to_radians().tan();
    let far = start + (inward * 5.0) + Vector3::new(0.0, 0.0, 10.0) + (along * lean);
    let at = |s: f64| -> Vec<Vector3> {
        chain
            .iter()
            .zip(&centres)
            .map(|(p, c)| *p + (*c * s))
            .collect()
    };
    let expected = first_polyline_exit(at, true, start, far);
    assert!(expected.is_finite());
    let path = BallPath {
        chain_points: &chain,
        unit_centres: &centres,
        closed: true,
        carried: [false, false],
        size: 7.9,
    };
    let room = path.pierce_room(&[(start, far)], 0);
    assert!(
        (room - expected).abs() < 1e-9,
        "room {room}, not {expected}"
    );
}

/// A square rim, (0, 0, 10) (20, 0, 10) (20, 20, 10) (0, 20, 10), closed, the first leg's centres
/// per unit (0, 1, -1) at both its ends, and a ridge from (21, 3, 10) to (21, 8, 20) beside the
/// corner (20, 0, 10) but past it. The first leg's straight axis carried on past the corner would
/// leave the ridge's line at the rib's 6 / (3 - sqrt 5) = 7.854; but that axis ends at the corner
/// and the round turns there, so the room is where the ridge's line leaves the round about the
/// polyline of centres, nearest at the corner's centre: 7.662, sooner than the phantom, which
/// would let sizes up to 7.854 through.
#[test]
fn a_sharp_turn_reads_its_corner_not_the_straight_axis_past_it() {
    let chain = [
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
        Vector3::new(20.0, 20.0, 10.0),
        Vector3::new(0.0, 20.0, 10.0),
    ];
    let centres = [
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(-1.0, -1.0, -1.0),
        Vector3::new(1.0, -1.0, -1.0),
    ];
    let (start, far) = (Vector3::new(21.0, 3.0, 10.0), Vector3::new(21.0, 8.0, 20.0));
    let at = |s: f64| -> Vec<Vector3> {
        chain
            .iter()
            .zip(&centres)
            .map(|(p, c)| *p + (*c * s))
            .collect()
    };
    let expected = first_polyline_exit(at, true, start, far);
    let phantom = 6.0 / (3.0 - 5.0_f64.sqrt());
    assert!(
        expected.is_finite() && (expected - phantom).abs() > 0.01,
        "{expected}"
    );
    let path = BallPath {
        chain_points: &chain,
        unit_centres: &centres,
        closed: true,
        carried: [false, false],
        size: 7.9,
    };
    let room = path.pierce_room(&[(start, far)], 0);
    assert!(
        (room - expected).abs() < 1e-9,
        "room {room}, not {expected}"
    );
}
