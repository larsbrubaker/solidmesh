//! The room a growing neighbour's far bound in neither side leaves a chain the rolling ball takes,
//! read from the ball's own centres ([`BallPath`]) rather than from the larger side's setback about
//! the chain (`band_room::room_of_cut_mesh` reads it in place of `foreign_edge_room_per_side`'s
//! "either" list for those edges).
//!
//! **Why.** An edge whose triangles are all in neither side (tag 0 on every one) is not on the band
//! of either side, so no side's field says where the round reaches it. Read against the chain's own
//! field, the larger side's setback, it is a cylinder of that setback about the chain, reaching in
//! every direction, including out past a rim the other side's contacts only rest on, where the ball
//! never goes: on the T Handle in the drawing's order a ledge 14.648 from the chain, past the wall's
//! rested rim, read a room of 11.661 though an R12 ball's centres stand 14.28 and 12.82 from it.
//!
//! **What the round reaches.** In the ball's cross-section at a station the round takes the ball
//! and the region between the ball and the chain point P: the convex hull of P and the ball, which
//! is the ball with the two tangent segments from P to it (the contacts, where the sides are
//! planes tangent to the ball through P). At size s the ball is centred at P + s c (c the centre's
//! offset per unit of the size the ball was walked at, `SideSetbacks::centres`) with radius s, so
//! the hull at size s is the hull at size 1 scaled about P by s: the hulls grow nested with the
//! size, and the size at which a point x enters is the gauge of x - P in the unit hull K = hull(0,
//! B(c, 1)) ([`gauge`]). Along the ray from P through x the unit hull runs from P to where the ray
//! leaves the unit ball, t = (d . c) + sqrt((d . c)^2 - (|c|^2 - 1)) with d the ray's unit direction;
//! a ray that misses that ball meets the hull only at P, and the point is never reached from that
//! station.
//!
//! **Which station.** A point is read on each chain leg within reach at its foot on that leg, with
//! the chain point and the centre offset interpolated linearly between the leg's two stations: the
//! cross-section through the point. The offset is read square to the leg: at a sharp turn the
//! walk's centre is the average of both legs' centres carried to the mitre plane, which leans along
//! each leg, and read as it stands the slice shrinks to a point and points beside the turn read as
//! never reached. Past an inner vertex the foot runs on along the leg up to the mitre plane there
//! (square to the sum of the two legs' directions), so each side of the mitre is read by its own
//! leg's round; past an open end it is held at the end vertex. An edge past a carried open end
//! (`BallPath::carried`, an end ray: the fill runs rows on virtually past it, s |c| (1 + 0.25), for
//! the growing neighbour's planes to clip) is not read here at all ([`is_past_a_carried_end`]): the
//! caller reads it as before, against the chain's own field. A leg is
//! within reach of an edge where its span stands within half the edge's length plus twice the
//! round's reach at the size asked (the size times one plus the largest centre) of the edge's
//! middle: an edge farther than that is read by no leg, a room well past the size asked. The room of
//! a point is the least over the legs, and an edge's the least over its points: the edge is read on
//! a grid no coarser than an eighth of the size asked (at least 16 intervals and at most 256), then
//! refined about its least sample by a fixed golden-section search, so the same edge reads the same
//! bits every time. The least is not guaranteed between samples where two legs trade the least,
//! which the grid bounds. Each point read charges one unit of the pass's work (`LevelLineWork`), so
//! the reading stops at the pass's budget or its cancel.
//!
//! **Past an unrayed end's own face** an edge wholly past the plane the round is cut flat in
//! (`rolling_ball_end_planes`) is read by none, as `foreign_edge_room_per_side` files it.
//!
//! **Exact at the size asked only where the sides are flat, an estimate otherwise.** The hull of
//! the chain point and the ball is the round's region where both sides are planes tangent to the
//! ball through the chain point. A side curving outward between the chain and its contact (a
//! cylinder of radius R) stands off its tangent plane by up to a^2 / 2R over a contact a from the
//! chain, so the band's cut can reach a sliver that far outside the hull, which this reading does
//! not see. At another size the centres are scaled linearly, as in `ball_pierce_room`, and a
//! contact's distance, so a centre, is not linear in the size on a curved side. A free end's
//! centres are the walk's, not the taper's, so near a free tip the reading takes the full ball and
//! can only read less room than the tapered round has. A NaN reading stays NaN, so the room reads
//! no number rather than clear.

use vector_math::{ieee_math, Vector3};

use super::ball_pierce_room::BallPath;
use super::end_face_sense::PointPlane;
use super::rolling_ball_end_planes;
use super::{LevelLineWork, WorkError};

/// The fewest intervals an edge is sampled on before the least is refined.
const LEAST_INTERVALS: usize = 16;

/// The most intervals an edge is sampled on, so one long edge cannot run the reading on without
/// bound (each sample is also charged to the pass's work).
const MOST_INTERVALS: usize = 256;

/// How many golden-section steps refine an edge's least sample: fixed, so the room is the same bits
/// every time.
const REFINE_STEPS: usize = 40;

/// How far past the size asked the legs read reach: an edge farther than this many times the
/// round's reach at the size asked from every leg is read against none of them.
const REACH_SLACK: f64 = 2.0;

/// The size at which the round's hull at one station (chain point `p`, centre offset per unit `c`,
/// square to the leg) first holds `x`: 0 at `p`, infinity where the ray from `p` through `x` misses
/// the unit ball about `c` (see the file header), NaN where the reading has no number.
pub fn gauge(x: Vector3, p: Vector3, c: Vector3) -> f64 {
    let d = x - p;
    let length = d.length();
    if length == 0.0 {
        return 0.0;
    }

    let direction = d * (1.0 / length);
    let along = direction.dot(c);
    let discriminant = (along * along) - (c.length_squared() - 1.0);
    if discriminant.is_nan() {
        return f64::NAN;
    }
    if discriminant < 0.0 {
        return f64::INFINITY;
    }

    let far = along + discriminant.sqrt();
    if far.is_nan() {
        return f64::NAN;
    }
    if far <= 0.0 {
        return f64::INFINITY;
    }

    length / far
}

/// `v` less its part along the unit direction `along`.
fn square_to(v: Vector3, along: Vector3) -> Vector3 {
    v - (along * v.dot(along))
}

/// The unit direction of the chain's leg `j`, zero for a leg of no length.
fn direction(path: &BallPath, j: usize) -> Vector3 {
    let n = path.chain_points.len();
    let ab = path.chain_points[(j + 1) % n] - path.chain_points[j];
    if ab.length_squared() > 0.0 {
        ab.get_normal()
    } else {
        Vector3::ZERO
    }
}

/// The size at which leg `j`'s round first holds `x`, read at `x`'s foot on the leg. The foot runs
/// on past an inner vertex up to the mitre plane there (the plane through the vertex square to the
/// sum of the two legs' directions), past a carried open end by up to the carry the fill runs
/// (`ball_pierce_room`: s |c| (1 + 0.25)), and is held at the vertex past an uncarried open end. The
/// centre's offset is read square to the leg: a mitre's averaged centre leans along the leg, and
/// read as it stands its slice shrinks to a point.
fn room_on_leg(path: &BallPath, j: usize, x: Vector3) -> f64 {
    let n = path.chain_points.len();
    let k = (j + 1) % n;
    let (a, b) = (path.chain_points[j], path.chain_points[k]);
    let ab = b - a;
    let along = direction(path, j);
    let raw = if ab.length_squared() > 0.0 {
        (x - a).dot(ab) / ab.length_squared()
    } else {
        0.0
    };
    let (ca, cb) = (path.unit_centres[j], path.unit_centres[k]);
    let open = !path.closed;
    // Before the leg's start: round the turn at its start vertex, or past the chain's first end
    // (held at the end vertex: an edge past a carried end is not read here, `is_past_a_carried_end`).
    if raw < 0.0 {
        if !(open && j == 0) {
            let previous = direction(path, (j + n - 1) % n);
            if (x - a).dot(previous + along) >= 0.0 {
                return gauge(x, a + (ab * raw), square_to(ca, along));
            }
        }
        return gauge(x, a, square_to(ca, along));
    }
    // Past the leg's end: round the turn at its end vertex, or past the chain's last end.
    if raw > 1.0 {
        if !(open && k == n - 1) {
            let next = direction(path, k);
            if (x - b).dot(along + next) <= 0.0 {
                return gauge(x, a + (ab * raw), square_to(cb, along));
            }
        }
        return gauge(x, b, square_to(cb, along));
    }
    let foot = a + (ab * raw);
    let c = ca + ((cb - ca) * raw);
    gauge(x, foot, square_to(c, along))
}

/// Whether any of the edge `from`-`to`'s ends or its middle stands past a carried open end
/// (`BallPath::carried`, an end ray): its nearest point on the chain is that end vertex and it lies
/// beyond it along the end leg. Past such an end the fill runs rows on virtually for the growing
/// neighbour's planes to clip, a shape this reading does not hold, so such an edge is read as
/// before, against the chain's own field.
pub fn is_past_a_carried_end(path: &BallPath, from: Vector3, to: Vector3) -> bool {
    let n = path.chain_points.len();
    if path.closed || n < 2 {
        return false;
    }
    let past = |x: Vector3, end: usize, inner: usize| {
        let (v, w) = (path.chain_points[end], path.chain_points[inner]);
        let outward = v - w;
        if (x - v).dot(outward) <= 0.0 {
            return false;
        }
        let nearest_is_the_end = path
            .chain_points
            .iter()
            .all(|&p| (x - v).length() <= (x - p).length());
        nearest_is_the_end
    };
    [from, to, (from + to) * 0.5]
        .iter()
        .any(|&x| (path.carried[0] && past(x, 0, 1)) || (path.carried[1] && past(x, n - 1, n - 2)))
}

/// The size at which the round first reaches the point `x`: the least of [`room_on_leg`] over
/// `legs`; NaN where any reading is.
pub fn room_of_point(path: &BallPath, legs: &[usize], x: Vector3) -> Result<f64, WorkError> {
    LevelLineWork::charge(1)?;
    let mut least = f64::INFINITY;
    for &j in legs {
        let room = room_on_leg(path, j, x);
        if room.is_nan() {
            return Ok(f64::NAN);
        }
        least = ieee_math::min_f64(least, room);
    }

    Ok(least)
}

/// Every leg of the chain, in order.
pub fn every_leg(path: &BallPath) -> Vec<usize> {
    let n = path.chain_points.len();
    (0..if path.closed { n } else { n - 1 }).collect()
}

/// The legs within reach of the edge `from`-`to`: those whose span stands within half the edge's
/// length plus [`REACH_SLACK`] times the round's reach at the size asked (the size times one plus
/// the largest centre) of the edge's middle, in chain order.
fn legs_within_reach(path: &BallPath, from: Vector3, to: Vector3) -> Vec<usize> {
    let largest = path
        .unit_centres
        .iter()
        .fold(0.0, |most: f64, centre| most.max(centre.length()));
    let middle = (from + to) * 0.5;
    let reach = ((to - from).length() * 0.5) + (REACH_SLACK * path.size * (1.0 + largest));
    let n = path.chain_points.len();
    every_leg(path)
        .into_iter()
        .filter(|&j| {
            let (a, b) = (path.chain_points[j], path.chain_points[(j + 1) % n]);
            let ab = b - a;
            let t = if ab.length_squared() > 0.0 {
                ((middle - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            (middle - (a + (ab * t))).length() <= reach
        })
        .collect()
}

/// The least room over the edge `from`-`to` and the point of the edge that sets it.
fn room_of_edge(path: &BallPath, from: Vector3, to: Vector3) -> Result<(f64, Vector3), WorkError> {
    let legs = legs_within_reach(path, from, to);
    if legs.is_empty() {
        return Ok((f64::INFINITY, from));
    }
    let along = to - from;
    let spacing = path.size / 8.0;
    // The interval count is capped, so the cast never overflows; a non-finite ratio takes the
    // fewest.
    let wanted = (along.length() / spacing).ceil();
    let intervals = if wanted.is_finite() && wanted > LEAST_INTERVALS as f64 {
        (wanted.min(MOST_INTERVALS as f64)) as usize
    } else {
        LEAST_INTERVALS
    };
    let at = |e: f64| from + (along * e);
    let mut best = (f64::INFINITY, from, 0usize);
    for i in 0..=intervals {
        let e = i as f64 / intervals as f64;
        let room = room_of_point(path, &legs, at(e))?;
        if room.is_nan() {
            return Ok((f64::NAN, at(e)));
        }
        if room < best.0 {
            best = (room, at(e), i);
        }
    }

    if !best.0.is_finite() {
        return Ok((best.0, best.1));
    }

    // Golden-section refinement in the sample's two neighbouring intervals.
    let step = 1.0 / intervals as f64;
    let mut low = (best.2 as f64 - 1.0).max(0.0) * step;
    let mut high = ((best.2 as f64 + 1.0) * step).min(1.0);
    let ratio = (5f64.sqrt() - 1.0) / 2.0;
    for _ in 0..REFINE_STEPS {
        let first = high - ((high - low) * ratio);
        let second = low + ((high - low) * ratio);
        if room_of_point(path, &legs, at(first))? <= room_of_point(path, &legs, at(second))? {
            high = second;
        } else {
            low = first;
        }
    }

    let middle = (low + high) / 2.0;
    let refined = room_of_point(path, &legs, at(middle))?;
    Ok(if refined < best.0 {
        (refined, at(middle))
    } else {
        (best.0, best.1)
    })
}

/// The least room over `edges` (each as its two end points, in order) and the point that sets it:
/// infinity and `None` where nothing is reached at any size, or there are no edges; NaN where a
/// reading has no number. An edge wholly past one of `end_planes` (by more than `snap`) is read by
/// none: past an unrayed end's own face the round is cut flat. A tie keeps the first edge's point.
pub fn room_of_far_bounds(
    path: &BallPath,
    edges: &[(Vector3, Vector3)],
    end_planes: &[Option<PointPlane>; 2],
    snap: f64,
) -> Result<(f64, Option<Vector3>), WorkError> {
    let mut least = (f64::INFINITY, None);
    for &(from, to) in edges {
        if rolling_ball_end_planes::is_wholly_past(end_planes, &[from, to], snap) {
            continue;
        }
        let (room, at) = room_of_edge(path, from, to)?;
        if room.is_nan() {
            return Ok((f64::NAN, Some(at)));
        }
        if room < least.0 {
            least = (room, Some(at));
        }
    }

    Ok(least)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The path over `points` with `centres`, open, carried at `carried`, walked at size 1.
    fn path<'a>(points: &'a [Vector3], centres: &'a [Vector3], carried: [bool; 2]) -> BallPath<'a> {
        BallPath {
            chain_points: points,
            unit_centres: centres,
            closed: false,
            carried,
            size: 1.0,
        }
    }

    fn room(path: &BallPath, x: Vector3) -> f64 {
        room_of_point(path, &every_leg(path), x).expect("no budget in force")
    }

    /// A square corner along x: sides y = 0 and z = 0, the ball's centre per unit at (0, 1, 1)
    /// (|c| = sqrt 2), the same at both stations.
    const CORNER: [Vector3; 2] = [Vector3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 0.0, 0.0)];
    const CORNER_CENTRES: [Vector3; 2] = [Vector3::new(0.0, 1.0, 1.0), Vector3::new(0.0, 1.0, 1.0)];

    #[test]
    fn a_point_on_the_balls_far_side_is_reached_at_the_size_whose_ball_holds_it() {
        // The ray along (0, 1, 1) leaves the unit ball about (0, 1, 1) at 1 + sqrt 2 per unit.
        let path = path(&CORNER, &CORNER_CENTRES, [false, false]);
        let read = room(&path, Vector3::new(5.0, 3.0, 3.0));
        let expected = (18f64).sqrt() / (1.0 + 2f64.sqrt());
        assert!((read - expected).abs() < 1e-12, "{read} {expected}");
    }

    #[test]
    fn a_point_outside_the_cone_from_the_chain_to_the_ball_is_never_reached() {
        // The tangent cone's half-angle is 45 degrees about (0, 1, 1): a point below the side
        // z = 0's plane lies outside it.
        let path = path(&CORNER, &CORNER_CENTRES, [false, false]);
        assert_eq!(room(&path, Vector3::new(5.0, 3.0, -0.5)), f64::INFINITY);
    }

    #[test]
    fn a_ledge_beyond_a_rested_rim_reads_more_room_than_the_larger_sides_cylinder() {
        // A wall resting on its rim lifts the ball's centre off the wall's plane: centre per unit
        // (0, 1, 1.25), 1.25 from the wall z = 0, 1 from the side y = 0. A ledge 1.2 per unit out
        // along the wall's plane reads room 1.2 / 1.25 = 0.96 against a cylinder of the larger
        // setback (1.25 per unit) about the chain; the ray to it misses the unit ball, so neither
        // the ball nor the band between it and the chain ever reaches it.
        let centres = [Vector3::new(0.0, 1.0, 1.25), Vector3::new(0.0, 1.0, 1.25)];
        let path = path(&CORNER, &centres, [false, false]);
        let from = Vector3::new(4.0, 1.2, -0.05);
        let to = Vector3::new(6.0, 1.2, -0.05);
        let cylinder = from.y.hypot(from.z) / 1.25;
        let (read, at) =
            room_of_far_bounds(&path, &[(from, to)], &[None, None], 1e-9).expect("no budget");
        assert!(read > cylinder, "{read} against {cylinder}");
        assert_eq!(read, f64::INFINITY);
        assert!(at.is_none());
    }

    /// A 90 degree outside turn: the chain runs up +y to the vertex at the origin and on along +x,
    /// the first leg's centre per unit (1, 0, 1), the second's (0, 1, 1), the vertex's the mitre's
    /// average (1, 1, 1), which leans along both legs.
    const TURN: [Vector3; 3] = [
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    const TURN_CENTRES: [Vector3; 3] = [
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(0.0, 1.0, 1.0),
    ];

    /// The size at which the second leg's round, centre (0, 1, 1) per unit about the foot
    /// (x, 0, 0), first holds (x, 0.99, 0.001), by hand.
    fn on_the_second_leg() -> f64 {
        let d = 0.99f64.hypot(0.001);
        let along = (0.99 + 0.001) / d;
        d / (along + ((along * along) - 1.0).sqrt())
    }

    #[test]
    fn beside_a_sharp_turn_the_round_on_each_side_of_the_mitre_reads_its_own_slice() {
        let path = path(&TURN, &TURN_CENTRES, [false, false]);
        // Before the second leg's start, on its side of the mitre plane, and just past it on the
        // leg: both are held by the second leg's round at a little under 0.99 of the size.
        for x in [-0.2, 0.2] {
            let read = room(&path, Vector3::new(x, 0.99, 0.001));
            let expected = on_the_second_leg();
            assert!(
                (read - expected).abs() < 1e-9,
                "at x = {x}: {read} against {expected}"
            );
        }
    }

    #[test]
    fn an_edge_past_a_carried_end_is_left_to_the_chains_own_field() {
        // The square corner: an edge 1 past its last end, beside the ball, is past that end only
        // where the end is carried; one beside the chain's middle never is.
        let past = (Vector3::new(11.0, 0.5, 0.5), Vector3::new(11.0, 0.6, 0.6));
        let beside = (Vector3::new(5.0, 0.5, 0.5), Vector3::new(5.0, 0.6, 0.6));
        let carried = path(&CORNER, &CORNER_CENTRES, [false, true]);
        let uncarried = path(&CORNER, &CORNER_CENTRES, [false, false]);
        assert!(is_past_a_carried_end(&carried, past.0, past.1));
        assert!(!is_past_a_carried_end(&uncarried, past.0, past.1));
        assert!(!is_past_a_carried_end(&carried, beside.0, beside.1));
    }

    #[test]
    fn a_reading_with_no_number_stays_no_number() {
        let c = Vector3::new(f64::NAN, 1.0, 1.0);
        assert!(gauge(Vector3::new(0.0, 1.0, 1.0), Vector3::ZERO, c).is_nan());
    }

    #[test]
    fn an_edge_wholly_past_an_unrayed_ends_own_plane_is_read_by_none() {
        // The end face x = 0, faced toward the chain (+x): an edge at x = -0.1, within the end
        // vertex's cone, lies past it.
        let path = path(&CORNER, &CORNER_CENTRES, [false, false]);
        let planes = [Some((Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0))), None];
        let edge = (Vector3::new(-0.1, 1.0, 1.0), Vector3::new(-0.1, 1.2, 1.2));
        let (read, _) = room_of_far_bounds(&path, &[edge], &planes, 1e-9).expect("no budget");
        assert_eq!(read, f64::INFINITY);
        let (unplaned, _) =
            room_of_far_bounds(&path, &[edge], &[None, None], 1e-9).expect("no budget");
        assert!(unplaned.is_finite());
    }
}
