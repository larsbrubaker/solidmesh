//! The open fill's row stations: each rail split at the other's parameters
//! ([`with_stations_of`]), the row-spacing share, the mitred-turn test ([`is_mitred_turn_at`]), the
//! parameters each rail vertex stands for ([`tie_spans`], [`on_the_edge`]), the thinning of rows to
//! where the round needs them ([`rows_where_the_round_needs_them`]) and the zipper that pairs the
//! two rails ([`spokes`]).

use vector_math::{ieee_math, Vector3};

use super::{ChainField, LevelLineMesh, LevelLineMeshError, OpenGapFillOpenRail, WorkError};
use crate::bevel::float_grid;

/// The least spacing between kept rows, as a share of the row's own width.
pub const MIN_ROW_SPACING_SHARE: f64 = 1.0 / 8.0;

/// `rail` with a vertex split into its edge at every parameter
/// where `other` has one strictly inside that edge. The split point is the edge's point at that
/// parameter, read linearly along the edge, so it lies on the level line the edge already is.
///
/// AN EDGE THAT STARTS OR ENDS AT A TIE: a rail vertex equidistant from two legs of a turning
/// chain (the inside corner of a chamfer turning over a cube corner, at (7, 7, 10) for d 3)
/// carries one leg's parameter, but the edge leaving it runs along the other leg, so its
/// parameters do not start there: read linearly from 17 to 40, the station for the turn's own
/// parameter 20 landed 2.2 along the edge at (4.78, 7, 10) instead of on the corner, its spoke
/// crossed from one chamfer plane to the other, and the turn removed 174.33 where the mitre
/// removes 171. So each end's parameter is read as the chain sees the edge just inside that end,
/// and a station the edge never reaches (inside the tie's jump) is not split in: the zipper pairs
/// it with the tie vertex itself, which is the mitre.
///
/// Returns `rail` itself (cloned) where nothing was split in or snapped.
pub fn with_stations_of(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    rail: &OpenGapFillOpenRail,
    other: &OpenGapFillOpenRail,
    length: f64,
) -> Result<OpenGapFillOpenRail, LevelLineMeshError> {
    let tie = super::open_gap_fill::PARAMETER_TIE_RELATIVE * ieee_math::max_f64(1.0, length);
    let rail_count = rail.vertices.len();
    let other_count = other.parameters.len();
    let mut vertices = vec![rail.vertices[0]];
    let mut parameters = vec![rail.parameters[0]];
    let mut j = 0usize;
    let mut snapped = false;
    let mut last_end_reads: Option<f64> = None;
    // Read before any split: a split rail edge stands on no triangle afterwards either.
    let virtual_edges: Vec<bool> = (0..rail_count.saturating_sub(1))
        .map(|k| {
            mesh.triangles_on(rail.vertices[k], rail.vertices[k + 1])
                .is_empty()
        })
        .collect();
    let is_virtual_edge = |k: i64| k >= 0 && k + 1 < rail_count as i64 && virtual_edges[k as usize];

    // Rail vertex k is where the rail meets a crease and runs on virtually past it: one of its two
    // edges stands on no triangle and the other does.
    let starts_a_virtual_stretch =
        |k: i64| is_virtual_edge(k - 1) != is_virtual_edge(k) && k > 0 && k + 1 < rail_count as i64;
    let mut i = 0usize;
    while i + 1 < rail_count {
        let (mut from, mut to) = (rail.parameters[i], rail.parameters[i + 1]);
        let mut low = rail.vertices[i];
        let mut high_reads: Option<f64> = None;
        let (skip_below, skip_above) = (from, to);
        (from, to) = on_the_edge(
            chain,
            mesh.position(low),
            mesh.position(rail.vertices[i + 1]),
            from,
            to,
            tie,
        )?;
        while j < other_count && other.parameters[j] <= ieee_math::max_f64(from, skip_below) + tie {
            j += 1;
        }

        while j < other_count && other.parameters[j] < ieee_math::min_f64(to, skip_above) - tie {
            let t = other.parameters[j];
            let high = rail.vertices[i + 1];
            let point = mesh.position(low)
                + ((mesh.position(high) - mesh.position(low)) * ((t - from) / (to - from)));

            // A station within a float step of the RAIL's own end IS that end: the two rails' ends
            // on an end face read parameters a float step apart where the part came from float
            // storage, so the other rail's end asked for a station a hair along this rail's first
            // edge (Encroaching Side at its saved 3.4: 1.15e-7 above the bottom and 5e-7 below the
            // top). Split in, it made a second row a float step from the end row, a zero-width
            // strip between them. So the end takes the station's parameter instead - the two
            // rails' ends then read the same parameter and the zipper pairs them one to one. Merely
            // skipping the station left the other rail's end reading past this one's, and the
            // zipper paired it a row away: Encroaching Side's front round near the top stood 0.209
            // off its control. The parameter stays monotone by construction: the station lies
            // strictly inside this end edge. Only at the rail's two ends: an interior rail vertex a
            // float step from a station is a real station of its own (Thin Front's fin rows moved
            // 0.065 off the control when every edge end paired).
            let step = float_grid::step_at_point(point);
            let at_the_first_end = i == 0
                && vertices.len() == 1
                && (point - mesh.position(rail.vertices[0])).length() <= step;
            let at_the_last_end =
                i + 2 == rail_count && (point - mesh.position(high)).length() <= step;
            if (at_the_first_end || at_the_last_end) && !is_mitred_turn_at(chain, t, tie) {
                if at_the_first_end {
                    parameters[0] = t;
                    from = t;
                } else {
                    last_end_reads = Some(t);
                }

                snapped = true;
                j += 1;
                continue;
            }

            // A VIRTUAL rail edge (carried past a crease, standing on no triangle) takes its
            // station as a plain vertex: there is no triangle to split. A station within a float
            // step of the vertex where the rail meets the crease and runs on virtually (from either
            // edge meeting there) IS that vertex, as at the rail's own ends: the other rail's
            // crease vertex read a parameter a float step off this one's where the part came from
            // float storage, and split in it made a whole row a hair from the crease row - Inner
            // Right Sides and Center's ridge alone, rows 8.7e-7 apart at both end faces, zero-area
            // triangles pass B's cut then read as creases and crossed with zero-length rail edges
            // (RoundsFirstTests.TheRoundsLeaveNoTwinVertices).
            let is_virtual = mesh.triangles_on(low, high).is_empty();
            if !is_mitred_turn_at(chain, t, tie) {
                if low == rail.vertices[i]
                    && starts_a_virtual_stretch(i as i64)
                    && (point - mesh.position(low)).length() <= step
                {
                    let last = parameters.len() - 1;
                    parameters[last] = t;
                    from = t;
                    snapped = true;
                    j += 1;
                    continue;
                }

                if starts_a_virtual_stretch(i as i64 + 1)
                    && (point - mesh.position(high)).length() <= step
                {
                    high_reads = Some(t);
                    j += 1;
                    continue;
                }
            }

            // Rounds first's pass B: a station nearer a vertex this rail already has than a row is
            // worth (rows_where_the_round_needs_them) IS that vertex, as a station a float step
            // from one is above: split in, it cut the side triangle on the rail edge into slivers
            // beside that vertex.
            if chain.has_smooth_joints() && !is_mitred_turn_at(chain, t, tie) {
                let spacing =
                    MIN_ROW_SPACING_SHARE * 2.0 * chain.setback_at(t.clamp(0.0, chain.length));
                if (point - mesh.position(low)).length() < spacing && vertices.len() > 1 {
                    let last = parameters.len() - 1;
                    parameters[last] = t;
                    from = t;
                    snapped = true;
                    j += 1;
                    continue;
                }

                if (point - mesh.position(high)).length() < spacing && i + 2 < rail_count {
                    high_reads = Some(t);
                    j += 1;
                    continue;
                }
            }

            let split = if is_virtual {
                mesh.add_vertex(point)
            } else {
                mesh.split_edge(low, high, point, true)?
            };
            if (split == low || split == high) && !is_mitred_turn_at(chain, t, tie) {
                // The mesh answered with an end of the edge: the station lies within the build's
                // snap of a tie vertex whose reading jumps past it (the other rail's station a hair
                // beyond the jump's top), so the zipper pairs it with that vertex, as it does a
                // station inside the jump. Adding it again made a zero-length rail edge, which the
                // strip clip read as a virtual edge with no growing neighbour and refused
                // GrowingNeighbourClipRefused (Top Face Should stil bevel c68 at 3.2 and 3.59). Not
                // at a MITRED turn: there the same repeat marks the fold
                // MitreInsideCornerKeptEdgeTests names - read past, that part gains volume - so the
                // repeat is kept and the clip still refuses until the fill emits the mitre's inside
                // corner right.
                j += 1;
                continue;
            }

            vertices.push(split);
            parameters.push(t);
            low = split;
            from = t;
            j += 1;
        }

        vertices.push(rail.vertices[i + 1]);
        parameters.push(match last_end_reads {
            Some(reads) if i + 2 == rail_count => reads,
            _ => high_reads.unwrap_or(rail.parameters[i + 1]),
        });
        snapped |= high_reads.is_some();
        i += 1;
    }

    Ok(if vertices.len() == rail_count && !snapped {
        rail.clone()
    } else {
        OpenGapFillOpenRail {
            vertices,
            parameters,
        }
    })
}

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

/// Per rail vertex, the parameters it stands for: its own, widened to
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
        spans[i].1 = ieee_math::max_f64(spans[i].1, read_from);
        spans[i + 1].0 = ieee_math::min_f64(spans[i + 1].0, read_to);
    }

    Ok(spans)
}

/// The parameters the chain reads just inside each end of the rail
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

/// The pairs of rail stations the fill's rows join, from both rails'
/// starts to both ends - a station both rails carry advancing both at once (when `together`),
/// otherwise the rail whose next edge's MIDDLE comes first between the chain's ends (the same
/// order read from either end), or whose next station comes first past them. Each step charged.
pub fn spokes(
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    length: f64,
    together: bool,
) -> Result<Vec<(i32, i32)>, WorkError> {
    let tie = super::open_gap_fill::PARAMETER_TIE_RELATIVE * ieee_math::max_f64(1.0, length);
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
