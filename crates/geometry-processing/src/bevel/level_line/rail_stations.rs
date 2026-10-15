//! A rail's stations: the pieces an edge was split into ([`pieces`]), the edge key ([`key`]) and
//! the parameters each
//! rail vertex stands for ([`tie_spans`], with [`jump_near`]), and the stations a long rail edge
//! is given where it spans several of the other rail's vertices ([`densify`]).

use std::collections::HashMap;

use vector_math::{ieee_math, Vector3};

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

/// Per rail vertex, the parameters it stands for - its own, widened
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
        spans[i].1 = ieee_math::max_f64(spans[i].1, ieee_math::max_f64(read_from, near_p));
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
        spans[i + 1].0 = ieee_math::min_f64(spans[i + 1].0, ieee_math::min_f64(read_to, near_q));
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
            spans[last].1 =
                ieee_math::max_f64(spans[last].1, ieee_math::max_f64(read_from, near_p));
            let near_q = jump_near(chain, q, p, to, from, tie, -1, closed_length)?;
            spans[0].0 = ieee_math::min_f64(
                spans[0].0,
                ieee_math::min_f64(read_to, near_q) - closed_length,
            );
        }
    }

    Ok(spans)
}

/// Where the chain's reading jumps just off p along the edge p -> q
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

/// Adds a station to every edge of the rail (`vertices`, climbing
/// `parameters`) whose parameter span holds at least `fewest` of `others` - the point ON the edge
/// whose chain parameter is the other rail's vertex parameter, split through the mesh's own split -
/// inserting each into the rail's lists in order and recording each split in `splits` (the split
/// edge, undirected, to its new vertex), and each far end standing for a jump in `jump_ends` (at
/// the jump's foot). Answers -1, or the rail vertex starting an edge whose parameter FALLS along it
/// where stations were asked for - they would come out of order, so the caller refuses.
/// `fewest` is 2 for the zip's pairing, 1 where a strip end is set back.
#[allow(clippy::too_many_arguments)]
pub fn densify(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    vertices: &mut Vec<i32>,
    parameters: &mut Vec<f64>,
    others: &[f64],
    tie: f64,
    splits: &mut HashMap<(i32, i32), i32>,
    jump_ends: &mut HashMap<i32, f64>,
    fewest: usize,
) -> Result<i32, super::LevelLineMeshError> {
    let mut i = 0;
    while i + 1 < vertices.len() {
        let from = parameters[i];
        let to = parameters[i + 1];
        let mut inside: Vec<f64> = others
            .iter()
            .copied()
            .filter(|&p| p > from + tie && p < to - tie)
            .collect();

        if inside.len() < fewest {
            i += 1;
            continue;
        }

        // Values strictly inside a span, so never NaN: the total order is the plain numeric one.
        inside.sort_by(f64::total_cmp);
        let a = vertices[i];
        let b = vertices[i + 1];
        let pa = mesh.position(a);
        let pb = mesh.position(b);
        let length = (pb - pa).length();
        let mut stations: Vec<(i32, f64)> = Vec::new();
        let mut t_last = 0.0;
        let mut near = a;
        let note_jump = |jump_ends: &mut HashMap<i32, f64>, foot: f64| {
            let entry = jump_ends.entry(b).or_insert(foot);
            *entry = ieee_math::min_f64(*entry, foot);
        };
        for target in inside {
            super::LevelLineWork::charge(1)?;

            // Bisection from the last station on: where the parameter climbs along the edge the
            // reading brackets the target; where it jumps, the bisection closes on the jump.
            let mut lo = t_last;
            let mut hi = 1.0;
            let mut step = 0;
            while step < 60 && (hi - lo) * length > tie {
                let mid = (lo + hi) / 2.0;
                if chain.evaluate(pa + ((pb - pa) * mid))?.parameter < target {
                    lo = mid;
                } else {
                    hi = mid;
                }
                step += 1;
            }

            let t = (lo + hi) / 2.0;
            let point = pa + ((pb - pa) * t);
            let reading = chain.evaluate(point)?.parameter;
            if t * length <= 1e3 * tie || (1.0 - t) * length <= 1e3 * tie {
                // The target sits in a jump at one of the edge's ends, so that end vertex is itself
                // the station; the FAR end stands for the whole jump and is ordered at its foot.
                if t * length > 1e3 * tie {
                    let foot = chain.evaluate(pa + ((pb - pa) * lo))?.parameter;
                    note_jump(jump_ends, foot);
                }

                continue;
            }

            if (reading - target).abs() > 1e3 * tie {
                if (1.0 - t) * length <= 0.05 * length {
                    // A jump in the edge's last twentieth reads as one at the far end.
                    let foot = chain.evaluate(pa + ((pb - pa) * lo))?.parameter;
                    note_jump(jump_ends, foot);
                }

                // A JUMP inside the edge: no point on it means that parameter, and the fan to it is
                // the honest pairing.
                continue;
            }

            if stations.last().is_some_and(|last| reading < last.1) {
                // The parameter FALLS along the edge: the stations would come out of order.
                return Ok(a);
            }

            let v = mesh.split_edge(near, b, point, true)?;
            if v == near || v == b {
                continue;
            }

            splits.insert(key(near, b), v);
            stations.push((v, reading));
            near = v;
            t_last = t;
        }

        for (s, &(vertex, parameter)) in stations.iter().enumerate() {
            vertices.insert(i + 1 + s, vertex);
            parameters.insert(i + 1 + s, parameter);
        }

        i += stations.len() + 1;
    }

    Ok(-1)
}
