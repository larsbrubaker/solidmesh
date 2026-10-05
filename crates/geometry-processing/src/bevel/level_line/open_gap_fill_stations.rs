//! The open fill's row stations. Ports `Bevel/LevelLine/OpenGapFill.Stations.cs` (a partial of the
//! C# static class `OpenGapFill`): so far its row-spacing share, the mitred-turn test
//! ([`is_mitred_turn_at`]) and the thinning of rows to where the round needs them
//! ([`rows_where_the_round_needs_them`]), the parameters each rail vertex stands for
//! ([`tie_spans`], [`on_the_edge`]); the stations land with later packages.

use vector_math::{cs_math, Vector3};

use super::{ChainField, LevelLineMesh, OpenGapFillOpenRail, WorkError};

/// The least spacing between kept rows, as a share of the row's own width.
pub const MIN_ROW_SPACING_SHARE: f64 = 1.0 / 8.0;

/// True when the chain parameter `parameter` stands (within `tie`) at an interior chain vertex
/// that turns past the sharp-turn threshold and is not a smooth joint.
pub fn is_mitred_turn_at(chain: &ChainField, parameter: f64, tie: f64) -> bool {
    let points = chain.points();
    let mut at = 0.0;
    let mut k = 1;
    while k + 1 < points.len() {
        at += (points[k] - points[k - 1]).length();
        if (at - parameter).abs() <= tie {
            let into = (points[k] - points[k - 1]).get_normal();
            let out_of = (points[k + 1] - points[k]).get_normal();
            let turn = into.dot(out_of).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI;
            return turn > chain.sharp_turn_degrees && !chain.is_smooth_joint(k as i32);
        }

        k += 1;
    }

    false
}

/// The spokes that get a row: the first and last always, and every spoke between unless it stands
/// inside the chain with both ends nearer the last kept row than [`MIN_ROW_SPACING_SHARE`] of its
/// own width. Whatever kind of cell is left from the last row kept to the next spoke - both rails
/// advancing, or one - the fill takes the rail vertices between them into its base triangles.
pub fn rows_where_the_round_needs_them(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    spokes: &[(i32, i32)],
) -> Vec<(i32, i32)> {
    let mut kept = vec![spokes[0]];
    let mut k = 1;
    while k + 1 < spokes.len() {
        let previous = kept[kept.len() - 1];
        let spoke = spokes[k];
        k += 1;
        let l = mesh.position(left.vertices[spoke.0 as usize]);
        let r = mesh.position(right.vertices[spoke.1 as usize]);
        let spacing = MIN_ROW_SPACING_SHARE * (r - l).length();
        let inside = left.parameters[spoke.0 as usize] > 0.0
            && left.parameters[spoke.0 as usize] < chain.length
            && right.parameters[spoke.1 as usize] > 0.0
            && right.parameters[spoke.1 as usize] < chain.length;
        if inside
            && (l - mesh.position(left.vertices[previous.0 as usize])).length() < spacing
            && (r - mesh.position(right.vertices[previous.1 as usize])).length() < spacing
        {
            continue;
        }

        kept.push(spoke);
    }

    kept.push(spokes[spokes.len() - 1]);
    kept
}

/// C# `OpenGapFill.TieSpans`: per rail vertex, the parameters it stands for: its own, widened to
/// the reading just inside each edge it ends ([`on_the_edge`]) where that jumps - a vertex
/// equidistant from two legs of a turning chain stands for the whole jump between them. Low = High
/// = its own parameter everywhere else.
pub fn tie_spans(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    rail: &OpenGapFillOpenRail,
    tie: f64,
) -> Result<Vec<(f64, f64)>, WorkError> {
    let mut spans: Vec<(f64, f64)> = rail.parameters.iter().map(|&p| (p, p)).collect();
    for i in 0..rail.parameters.len().saturating_sub(1) {
        let (from, to) = (rail.parameters[i], rail.parameters[i + 1]);
        if to <= from {
            continue;
        }

        let (read_from, read_to) = on_the_edge(
            chain,
            mesh.position(rail.vertices[i]),
            mesh.position(rail.vertices[i + 1]),
            from,
            to,
            tie,
        )?;
        spans[i].1 = cs_math::max_f64(spans[i].1, read_from);
        spans[i + 1].0 = cs_math::min_f64(spans[i + 1].0, read_to);
    }

    Ok(spans)
}

/// C# `OpenGapFill.OnTheEdge`: the parameters the chain reads just inside each end of the rail
/// edge p -> q, where they differ from the stored ones (a tie vertex) and still lie within them;
/// else the stored ones, bit for bit - so an edge past a chain end, whose extended parameters the
/// chain cannot read, keeps its own. Read 1e-6 of the edge in, a continuous edge moves its
/// parameter by about that share; only a jump moves it by a thousandth of the edge's span or more.
pub fn on_the_edge(
    chain: &ChainField,
    p: Vector3,
    q: Vector3,
    from: f64,
    to: f64,
    tie: f64,
) -> Result<(f64, f64), WorkError> {
    if to - from <= tie || from < 0.0 || to > chain.length {
        return Ok((from, to));
    }

    let near_from = chain.evaluate(p + ((q - p) * 1e-6))?.parameter;
    let near_to = chain.evaluate(q + ((p - q) * 1e-6))?.parameter;
    let jump = (1e-3 * (to - from)) + tie;
    let read_from = if near_from > from + jump && near_from < to {
        near_from
    } else {
        from
    };
    let read_to = if near_to < to - jump && near_to > read_from {
        near_to
    } else {
        to
    };
    Ok((read_from, read_to))
}

/// C# `OpenGapFill.Spokes`: the pairs of rail stations the fill's rows join, from both rails'
/// starts to both ends - a station both rails carry advancing both at once (when `together`),
/// otherwise the rail whose next edge's MIDDLE comes first between the chain's ends (the same
/// order read from either end), or whose next station comes first past them. Each step charged.
pub fn spokes(
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    length: f64,
    together: bool,
) -> Result<Vec<(i32, i32)>, WorkError> {
    let tie = super::open_gap_fill::PARAMETER_TIE_RELATIVE * cs_math::max_f64(1.0, length);
    let mut steps = Vec::new();
    let (mut a, mut b) = (0usize, 0usize);
    let left_last = left.count() as usize - 1;
    let right_last = right.count() as usize - 1;
    steps.push((0, 0));

    while a < left_last || b < right_last {
        super::LevelLineWork::charge(1)?;

        // A station both rails carry is one spoke: both advance together.
        if together
            && a < left_last
            && b < right_last
            && (left.parameters[a + 1] - right.parameters[b + 1]).abs() <= tie
        {
            a += 1;
            b += 1;
            steps.push((a as i32, b as i32));
            continue;
        }

        // Merged by the MIDDLE of each rail's next edge, so the rows come out the same whichever
        // way the chain was walked - only between the chain's ends; past them the far end rules.
        let can_left = a < left_last;
        let can_right = b < right_last;
        let left_middle = if can_left {
            left.parameters[a] + left.parameters[a + 1]
        } else {
            f64::INFINITY
        };
        let right_middle = if can_right {
            right.parameters[b] + right.parameters[b + 1]
        } else {
            f64::INFINITY
        };
        let inside = can_left
            && can_right
            && left.parameters[a] >= -tie
            && left.parameters[a + 1] <= length + tie
            && right.parameters[b] >= -tie
            && right.parameters[b + 1] <= length + tie;
        let advances_left = if inside {
            left_middle <= right_middle + (2.0 * tie)
        } else {
            b >= right_last || (can_left && left.parameters[a + 1] <= right.parameters[b + 1] + tie)
        };

        if advances_left {
            a += 1;
        } else {
            b += 1;
        }

        steps.push((a as i32, b as i32));
    }

    Ok(steps)
}
