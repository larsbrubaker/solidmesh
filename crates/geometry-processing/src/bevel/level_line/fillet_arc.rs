//! A fillet's arc across the band: its collinear angle, the whole-leg test ([`on_a_whole_leg`]), a
//! sharp joint's seam row
//! ([`seam_row`]) and its tolerance, the carried skew arc ([`carried`]) with its rigid turn
//! ([`turn`]), the smooth step, the bend threshold's cosine, the joint search and the carry's
//! weight ([`carry_weight`]), and a row end on a joint's seam re-read on its partner's leg
//! ([`on_the_partners_leg`]); the arcs that read the level-line mesh are in `fillet_arc_rows`. The
//! arc point itself is [`super::gap_fill::arc_point`].

use vector_math::Vector3;

use super::gap_fill::arc_point;
use super::{ChainField, LevelLineMesh};

/// How far a chain joint may turn, in degrees, and still be read as straight through: far above
/// the float noise of an edge an importer split into collinear pieces (well under a thousandth of
/// a degree), and far below the few degrees a tessellated round turns at each vertex (5.6 on Many
/// Holes' 64-facet mouths); a round would need over 360 facets to read as one leg, and a closed
/// one never does (it has no ends). A row carried across a 1 degree joint is off its true axis by
/// at most 1.7% of its skew length.
pub const COLLINEAR_DEGREES: f64 = 1.0;

/// True when the chain `parameter` lies on a whole LEG: the run of collinear segments round it
/// is bounded at both ends by a sharp joint or a chain end.
pub fn on_a_whole_leg(chain: &ChainField, parameter: f64) -> bool {
    let points = chain.points();
    let n = points.len() as i32;
    let count = if chain.is_closed { n } else { n - 1 };
    let at_point = |vertex: i32| points[vertex.rem_euclid(n) as usize];
    let mut at = 0.0;
    let mut segment = count - 1;
    for k in 0..count {
        let next = at + (at_point(k + 1) - at_point(k)).length();
        if parameter <= next {
            segment = k;
            break;
        }

        at = next;
    }

    let is_end = |vertex: i32| !chain.is_closed && (vertex <= 0 || vertex >= n - 1);
    let turn = |vertex: i32| {
        let into = (at_point(vertex) - at_point(vertex - 1)).get_normal();
        let out_of = (at_point(vertex + 1) - at_point(vertex)).get_normal();
        into.dot(out_of).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI
    };
    let sharp_or_end = |vertex: i32| {
        is_end(vertex)
            || (turn(vertex) > chain.sharp_turn_degrees
                && !chain.is_smooth_joint(vertex.rem_euclid(n)))
    };

    // Walk out from the segment's two ends across collinear joints to the leg's bounding vertices.
    // A closed chain that never turns past the collinear angle has no ends - a fine round - and is
    // no leg.
    let mut start = segment;
    let mut steps = 0;
    while !is_end(start) && turn(start) < COLLINEAR_DEGREES {
        if steps >= count {
            return false;
        }

        steps += 1;
        start -= 1;
    }

    let mut end = segment + 1;
    steps = 0;
    while !is_end(end) && turn(end) < COLLINEAR_DEGREES {
        if steps >= count {
            return false;
        }

        steps += 1;
        end += 1;
    }

    sharp_or_end(start) && sharp_or_end(end)
}

/// The seam row's points (index 0 and `segments` are `p` and `q`): the
/// incoming leg's fillet section through the two ends - the arc about the leg's axis through the
/// apex - each point slid along that leg onto the joint's bisector plane.
pub fn seam_row(
    apex: Vector3,
    into: Vector3,
    out_of: Vector3,
    p: Vector3,
    q: Vector3,
    segments: i32,
) -> Vec<Vector3> {
    let normal = (into + out_of).get_normal();
    let slide = into.dot(normal);
    let section = |x: Vector3| (x - apex) - (into * (x - apex).dot(into));
    let from_section = section(p);
    let to_section = section(q);
    let mut row = vec![Vector3::ZERO; (segments + 1) as usize];
    row[0] = p;
    row[segments as usize] = q;
    for s in 1..segments {
        let on_section = arc_point(
            from_section,
            Vector3::ZERO,
            to_section,
            s as f64 / segments as f64,
        );
        row[s as usize] = apex + on_section - (into * (on_section.dot(normal) / slide));
    }

    row
}

/// How far off a sharp joint's bisector plane (or mitre line) a rail
/// vertex may stand and still be on it: a millionth of the joint's setback plus a millionth of how
/// far the joint is from the origin.
///
/// Scaled by the coordinates' own size, not the chain's length: rails that went through floats
/// carry about 1e-7 of their coordinate magnitude, so a small chain far from the origin stands
/// 1e-5 off the plane its double-precision chain puts there - past a chain-length tolerance, and
/// the tip rows fell back to a single leg's arc
/// (ChainFieldMitreTests.AroundCenterFarFromTheOriginReadsTheSameWorst).
pub fn seam_tolerance(apex: Vector3, setback: f64) -> f64 {
    1e-6 * (setback + apex.length())
}

/// 0 at or below 0, 1 at or above 1, and a smooth cubic between.
pub fn smooth_step(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - (2.0 * x))
}

/// A chain point and the chain's unit direction there (point, tangent).
pub type ChainSection = (Vector3, Vector3);

/// A skew spoke's arc points across a turn of the chain (index 0 and
/// `segments` are the ends): as the translating overload, but each end's section is given as a
/// chain point and the chain's direction there, and is carried to the mean section by the rigid
/// turn taking its direction onto the mean's. Where all three directions agree it is that
/// overload.
pub fn carried(
    from_position: Vector3,
    to_position: Vector3,
    from_section: ChainSection,
    mean: ChainSection,
    to_section: ChainSection,
    segments: i32,
) -> Vec<Vector3> {
    let p = mean.0 + turn(from_section.1, mean.1, from_position - from_section.0);
    let q = mean.0 + turn(to_section.1, mean.1, to_position - to_section.0);
    let mut arc = vec![Vector3::ZERO; (segments + 1) as usize];
    arc[0] = from_position;
    arc[segments as usize] = to_position;
    for s in 1..segments {
        let t = s as f64 / segments as f64;
        let on_mean = arc_point(p, mean.0, q, t) - mean.0;
        let back_to_from = from_section.0 + turn(mean.1, from_section.1, on_mean);
        let back_to_to = to_section.0 + turn(mean.1, to_section.1, on_mean);
        arc[s as usize] = (back_to_from * (1.0 - t)) + (back_to_to * t);
    }

    arc
}

/// The cosine of the chain's bend threshold: a joint whose segments'
/// directions dot below it is sharp. Asked only of a finite threshold (`AlongTheChain` carries
/// nothing under an infinite one).
pub fn sharp_cosine(chain: &ChainField) -> f64 {
    (chain.sharp_turn_degrees * std::f64::consts::PI / 180.0).cos()
}

/// The index of the first joint (sorted by parameter) at or
/// past `parameter`, by binary search; the count where there is none.
pub fn first_joint_at_or_past(joints: &[(f64, f64)], parameter: f64) -> usize {
    let mut low = 0;
    let mut high = joints.len();
    while low < high {
        let middle = (low + high) / 2;
        if joints[middle].0 >= parameter {
            high = middle;
        } else {
            low = middle + 1;
        }
    }

    low
}

/// `vector` turned by the rotation taking unit `from` onto unit `to`.
pub fn turn(from: Vector3, to: Vector3, vector: Vector3) -> Vector3 {
    let mut axis = from.cross(to);
    let sin = axis.length();
    let cos = from.dot(to);
    if sin < 1e-12 {
        return vector;
    }

    axis /= sin;
    (vector * cos) + (axis.cross(vector) * sin) + (axis * (axis.dot(vector) * (1.0 - cos)))
}

/// How much of the carried drawing a row takes: 1 well inside a gentle
/// run, falling smoothly to 0 within one setback of a sharp joint - that joint's own setback, so one
/// steep spot does not widen every window - measured from the span `low` to `high` (unwrapped
/// parameters), and falling from half the bend threshold to 0 at it as the turn between the row's
/// end directions grows.
pub fn carry_weight(
    chain: &ChainField,
    low: f64,
    high: f64,
    from_tangent: Vector3,
    to_tangent: Vector3,
) -> f64 {
    let turn_degrees =
        from_tangent.dot(to_tangent).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI;
    let gentle = smooth_step(2.0 * (1.0 - (turn_degrees / chain.sharp_turn_degrees)));
    let joints = chain.sharp_joints();
    if gentle <= 0.0 || joints.is_empty() {
        return gentle;
    }

    // Only a joint within two of the largest setbacks can pull the weight under 1: visit just
    // those, by binary search, on every lap a closed chain's unwrapped span can reach.
    let reach = 2.0 * chain.max_setback;
    let length = chain.length;
    let mut away_from_joints: f64 = 1.0;
    // Floored, then cast: the laps are small counts, so the cast never saturates.
    let first_lap = if chain.is_closed {
        ((low - reach) / length).floor() as i32
    } else {
        0
    };
    let last_lap = if chain.is_closed {
        ((high + reach) / length).floor() as i32
    } else {
        0
    };
    for lap in first_lap..=last_lap {
        let offset = lap as f64 * length;
        let mut k = first_joint_at_or_past(joints, low - reach - offset);
        while k < joints.len() && joints[k].0 + offset <= high + reach {
            let at = joints[k].0 + offset;
            let distance = if at < low {
                low - at
            } else if at > high {
                at - high
            } else {
                0.0
            };
            let setback = joints[k].1;
            let weight = if setback > 0.0 {
                smooth_step((distance - setback) / setback)
            } else if distance > 0.0 {
                1.0
            } else {
                0.0
            };
            away_from_joints = vector_math::cs_math::min_f64(away_from_joints, weight);
            k += 1;
        }
    }

    gentle * away_from_joints
}

/// The chain's unit tangent at `parameter`, blended across each joint
/// between the two segments' midpoints so it turns smoothly - except at a sharp joint, where it
/// keeps its own segment's direction (the bisector exactly at the vertex).
pub fn tangent_at(chain: &ChainField, parameter: f64) -> Vector3 {
    let points = &chain.points;
    let n = points.len() as i32;
    let count = if chain.is_closed { n } else { n - 1 };
    let (segment, along) = chain.segment_holding(parameter);

    let direction = |k: i32| (points[((k + 1) % n) as usize] - points[k as usize]).get_normal();
    let half_length = |k: i32| (points[((k + 1) % n) as usize] - points[k as usize]).length() / 2.0;
    let half = half_length(segment);
    let own = direction(segment);
    let before = along < half;
    let mut neighbour = if before { segment - 1 } else { segment + 1 };

    if chain.is_closed {
        neighbour = (neighbour + count) % count;
    } else if neighbour < 0 || neighbour >= count {
        return own;
    }

    let other = direction(neighbour);

    if own.dot(other) < sharp_cosine(chain) {
        let at_vertex = chain.length * 1e-7;
        let off_the_vertex = if before { along } else { (2.0 * half) - along };
        return if off_the_vertex <= at_vertex {
            (own + other).get_normal()
        } else {
            own
        };
    }

    // The share of the neighbour's direction: this half's length over both halves' at the joint,
    // none at this segment's midpoint.
    let span = half + half_length(neighbour);
    let share = if before {
        (half - along) / span
    } else {
        (along - half) / span
    };

    ((own * (1.0 - share)) + (other * share)).get_normal()
}

/// A row end as the carry reads it: its vertex, chain parameter, tie span (low, high) and lap.
pub type RowEnd = (i32, f64, (f64, f64), f64);

/// A row end standing on a sharp joint's SEAM - equidistant from
/// the joint's two legs, so either leg's fillet passes through it - re-read on the leg its partner
/// stands nearer, at its foot on that leg; any other end as it was.
///
/// Around Center's tips have no inner rail vertex of their own: the one nearest a tip lies on the
/// seam plane and reads the incoming leg, so its spoke to a wedge vertex on the outgoing leg's side
/// was drawn about a foreign leg's chain point and cut 0.124 inside the control. Read on the
/// partner's leg, both ends lie on that leg's fillet and the row is carried along it. A partner
/// itself on the seam (a row lying in the seam plane) is left alone: neither leg is its. A reading
/// ON the joint counts as the incoming leg's, so it is re-read only toward the outgoing one - both
/// other choices were measured worse on Around Center.
pub fn on_the_partners_leg(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    end: RowEnd,
    partner_end: RowEnd,
) -> RowEnd {
    let length = chain.length;
    let position = mesh.position(end.0);
    let partner = mesh.position(partner_end.0);
    // The joint's own wrap, round the loop whatever the chain.
    let wrapped = |parameter: f64| {
        let parameter = parameter % length;
        if parameter < 0.0 {
            parameter + length
        } else {
            parameter
        }
    };

    // The end as the carry reads it against this partner: its tie span clamped toward the
    // partner's reading. Which leg it is on is decided by that reading, not the bare station.
    let reads = (partner_end.1 - end.3).clamp(end.2 .0, end.2 .1) + end.3;
    for &(joint_at, joint_setback) in chain.sharp_joints() {
        // The joint's parameter in the end's own unwrapped frame.
        let mut offset = reads - joint_at;
        offset -= (offset / length).round_ties_even() * length;
        if offset.abs() > 2.0 * joint_setback {
            continue;
        }

        let apex = chain.point_at(joint_at);
        let tolerance = seam_tolerance(apex, joint_setback);
        let step = 1e-3 * joint_setback;
        let into = (apex - chain.point_at(wrapped(joint_at - step))).get_normal();
        let out_of = (chain.point_at(wrapped(joint_at + step)) - apex).get_normal();
        // The seam is the joint's bisector plane: a point on it is equally far from both legs'
        // lines, and its side says which leg's fillet it is nearer.
        let normal = (into + out_of).get_normal();
        let e = position - apex;
        if e.dot(normal).abs() > tolerance || e.dot(into) > 0.0 || e.dot(out_of) < 0.0 {
            continue;
        }

        let side = (partner - apex).dot(normal);
        if side.abs() <= tolerance {
            continue;
        }

        let on_out = side > 0.0;
        if on_out == (offset > 0.0) {
            continue;
        }

        let at = reads - offset + if on_out { e.dot(out_of) } else { e.dot(into) };
        return (end.0, at, (at - end.3, at - end.3), end.3);
    }

    end
}
