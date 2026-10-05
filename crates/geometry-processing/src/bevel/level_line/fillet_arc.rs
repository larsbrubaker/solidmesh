//! A fillet's arc across the band. Ports `Bevel/LevelLine/FilletArc.cs` (a C# static class): so far
//! its collinear angle and the whole-leg test ([`on_a_whole_leg`]); the arcs land with later
//! packages.

use super::ChainField;

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
