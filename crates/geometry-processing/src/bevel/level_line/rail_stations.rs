//! A rail's stations. Ports `Bevel/LevelLine/RailStations.cs` (a C# static class): so far the
//! pieces an edge was split into ([`pieces`]), the edge key ([`key`]) and the parameters each
//! rail vertex stands for ([`tie_spans`], with [`jump_near`]); the stations land with later
//! packages.

use std::collections::HashMap;

use vector_math::{cs_math, Vector3};

use super::open_gap_fill_stations::on_the_edge;
use super::{ChainField, LevelLineMesh, WorkError};

/// The pieces edge `a b` stands as after its splits (each split edge's key mapped to the vertex
/// put in it), in order from `a` to `b`.
pub fn pieces(a: i32, b: i32, splits: &HashMap<(i32, i32), i32>) -> Vec<(i32, i32)> {
    let Some(&m) = splits.get(&key(a, b)) else {
        return vec![(a, b)];
    };

    let mut found = pieces(a, m, splits);
    found.extend(pieces(m, b, splits));
    found
}

/// The (low, high) key of an edge.
pub fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// C# `RailStations.TieSpans`: per rail vertex, the parameters it stands for - its own, widened
/// to the reading just inside each edge it ends ([`on_the_edge`]) and to where the reading jumps
/// near that end ([`jump_near`]). A closed rail (`closed_length` > 0) also reads its wrap edge,
/// which climbs from the last parameter to the first a lap on: the chain reads its ends in
/// [0, length), so each reading is carried into that lap before the jump test - missed, a rim's
/// corner vertex whose jump straddles the chain's start read as its own parameter only.
pub fn tie_spans(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    vertices: &[i32],
    parameters: &[f64],
    tie: f64,
    closed_length: f64,
) -> Result<Vec<(f64, f64)>, WorkError> {
    let mut spans: Vec<(f64, f64)> = parameters.iter().map(|&p| (p, p)).collect();
    for i in 0..vertices.len().saturating_sub(1) {
        if parameters[i + 1] <= parameters[i] {
            continue;
        }

        let (p, q) = (mesh.position(vertices[i]), mesh.position(vertices[i + 1]));
        let (read_from, read_to) = on_the_edge(chain, p, q, parameters[i], parameters[i + 1], tie)?;
        let near_p = jump_near(
            chain,
            p,
            q,
            parameters[i],
            parameters[i + 1],
            tie,
            1,
            closed_length,
        )?;
        spans[i].1 = cs_math::max_f64(spans[i].1, cs_math::max_f64(read_from, near_p));
        let near_q = jump_near(
            chain,
            q,
            p,
            parameters[i + 1],
            parameters[i],
            tie,
            -1,
            closed_length,
        )?;
        spans[i + 1].0 = cs_math::min_f64(spans[i + 1].0, cs_math::min_f64(read_to, near_q));
    }

    let last = vertices.len().saturating_sub(1);
    if closed_length > 0.0 && last > 0 {
        let from = parameters[last];
        let to = parameters[0] + closed_length;
        let lapped = |reading: f64| {
            if reading < from - tie {
                reading + closed_length
            } else {
                reading
            }
        };
        let p = mesh.position(vertices[last]);
        let q = mesh.position(vertices[0]);
        let near_from = lapped(chain.evaluate(p + ((q - p) * 1e-6))?.parameter);
        let near_to = lapped(chain.evaluate(q + ((p - q) * 1e-6))?.parameter);
        let jump = (1e-3 * (to - from)) + tie;
        if to - from > tie {
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
            let near_p = jump_near(chain, p, q, from, to, tie, 1, closed_length)?;
            spans[last].1 = cs_math::max_f64(spans[last].1, cs_math::max_f64(read_from, near_p));
            let near_q = jump_near(chain, q, p, to, from, tie, -1, closed_length)?;
            spans[0].0 = cs_math::min_f64(
                spans[0].0,
                cs_math::min_f64(read_to, near_q) - closed_length,
            );
        }
    }

    Ok(spans)
}

/// C# `RailStations.JumpNear`: where the chain's reading jumps just off p along the edge p -> q
/// (read `sense` +1 forward, -1 backward in parameter), found by bisection within the first
/// twentieth of the edge: a reading that climbs past twice the distance walked (plus a tie) has
/// jumped. `at_p` where nothing jumps there, or the jump overshoots `at_q`. On a closed chain each
/// reading is carried into the lap that runs from a little behind `at_p` onward in the sense of
/// travel, so a true climb of more than half a lap stays a climb.
#[allow(clippy::too_many_arguments)]
pub fn jump_near(
    chain: &ChainField,
    p: Vector3,
    q: Vector3,
    at_p: f64,
    at_q: f64,
    tie: f64,
    sense: i32,
    closed_length: f64,
) -> Result<f64, WorkError> {
    const WINDOW: f64 = 0.05;
    let sense = sense as f64;
    let length = (q - p).length();
    let read = |t: f64| -> Result<f64, WorkError> {
        let reading = chain.evaluate(p + ((q - p) * t))?.parameter;
        if closed_length <= 0.0 {
            return Ok(reading);
        }

        let behind = closed_length / 8.0;
        let ahead = sense * (reading - at_p) + behind;
        Ok(reading - (sense * (ahead / closed_length).floor() * closed_length))
    };
    let jumped = |t: f64| -> Result<bool, WorkError> {
        Ok(sense * (read(t)? - at_p) > (2.0 * t * length) + (1e3 * tie))
    };
    if length <= tie || !jumped(WINDOW)? {
        return Ok(at_p);
    }

    let mut lo = 0.0;
    let mut hi = WINDOW;
    let mut step = 0;
    while step < 50 && (hi - lo) * length > tie {
        let mid = (lo + hi) / 2.0;
        if jumped(mid)? {
            hi = mid;
        } else {
            lo = mid;
        }

        step += 1;
    }

    let across = read(hi)?;
    Ok(if sense * (at_q - across) > 0.0 {
        across
    } else {
        at_p
    })
}
