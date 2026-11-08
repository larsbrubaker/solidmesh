//! A rail's stations: the pieces an edge was split into ([`pieces`]), the edge key ([`key`]) and
//! the parameters each
//! rail vertex stands for ([`tie_spans`], with [`jump_near`]), the station a rail edge is given
//! where the chain's reading jumps along its middle ([`stations_at_jumps`]), and the stations a
//! long rail edge is given where it spans the other rail's vertices ([`densify`]).
//!
//! A JUMP is where the chain's reading of points along a rail edge climbs faster than the edge is
//! walked: the edge crosses the bisector on the inside of the chain's turn, and the one point on
//! it equidistant from both legs stands for every parameter between them. A jump within a
//! twentieth of an edge's end is read as that end vertex's span ([`jump_near`]); one further in
//! has no vertex to stand for it, so it is given a station there ([`stations_at_jumps`]) whose
//! span [`tie_spans`] then reads as the whole jump.

use std::collections::HashMap;

use solidmesh_math::{ieee_math, Vector3};

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

/// The share of a rail edge at each end where a jump is read as that end vertex's span
/// ([`jump_near`]) rather than given a station of its own ([`stations_at_jumps`]).
pub const END_WINDOW: f64 = 0.05;

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
    if length <= tie || !jumped(END_WINDOW)? {
        return Ok(at_p);
    }

    let mut lo = 0.0;
    let mut hi = END_WINDOW;
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
/// `fewest` is 1 at every call: the corner fill's zip and a strip end set back.
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

/// Gives each edge of the rail (`vertices`, climbing `parameters`) a station at every JUMP of the
/// chain's reading along it, between its end windows ([`END_WINDOW`]), that a parameter of
/// `others` (the other rail's vertices) falls inside. Without the station that vertex would pair
/// with an end of the edge reading none of the jump's parameters; a jump nothing reads into needs
/// none. Each station is the point on the edge at its jump, split through the mesh's own split,
/// inserted into the rail's lists at the reading just short of the jump (its foot, held within the
/// edge's readings and after the station before it) and recorded in `splits`. The station then
/// ends two edges with the jump at their shared end, so [`tie_spans`] reads its span as the whole
/// jump. A closed rail (`closed_length` > 0) also reads its wrap edge, a lap on, and stays cut
/// where the chain starts: a station past the chain's end leads the rail a lap back.
///
/// The jumps are found from the readers, not from the edge's slope: for each parameter of
/// `others` inside the edge's readings, the point reading it is bisected for along the edge, and
/// where the bracket closes on a step in the reading of more than `1e3 * tie` that step is a jump
/// the parameter falls inside; a bracket whose ends read within that of each other holds none, and
/// the search for that parameter stops there. So every jump with a reader is found, however many
/// the edge crosses. Only an edge whose reading climbs faster than it is walked between its end
/// windows is searched, which keeps the search off the many edges with no jump at all; a jump
/// smaller than the walk's shortfall on an edge slanting across the chain's legs is not found.
#[allow(clippy::too_many_arguments)]
pub fn stations_at_jumps(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    vertices: &mut Vec<i32>,
    parameters: &mut Vec<f64>,
    others: &[f64],
    tie: f64,
    closed_length: f64,
    splits: &mut HashMap<(i32, i32), i32>,
) -> Result<(), super::LevelLineMeshError> {
    let closed = closed_length > 0.0 && !vertices.is_empty();
    if closed {
        vertices.push(vertices[0]);
        parameters.push(parameters[0] + closed_length);
    }

    let mut i = 0;
    while i + 1 < vertices.len() {
        let (from, to) = (parameters[i], parameters[i + 1]);
        let (a, b) = (vertices[i], vertices[i + 1]);
        let (pa, pb) = (mesh.position(a), mesh.position(b));
        let length = (pb - pa).length();
        if to <= from || length <= tie {
            i += 1;
            continue;
        }

        let read = |t: f64| -> Result<f64, WorkError> {
            let reading = chain.evaluate(pa + ((pb - pa) * t))?.parameter;
            Ok(if closed && reading < from - (closed_length / 2.0) {
                reading + closed_length
            } else {
                reading
            })
        };
        // Away from a jump a straight edge's reading climbs no faster than the edge is walked (the
        // closest point on a straight leg moves no further than the point read), so an edge whose
        // reading between its end windows climbs no more than that has no jump to search.
        super::LevelLineWork::charge(1)?;
        let climb = read(1.0 - END_WINDOW)? - read(END_WINDOW)?;
        if climb - ((1.0 - (2.0 * END_WINDOW)) * length) <= 1e3 * tie {
            i += 1;
            continue;
        }

        let mut readers: Vec<f64> = others
            .iter()
            .copied()
            .filter(|&p| p > from + tie && p < to - tie)
            .collect();
        // Values strictly inside a span, so never NaN: the total order is the plain numeric one.
        readers.sort_by(f64::total_cmp);

        // Each jump as (where along the edge, its foot, the reading across it), in edge order.
        let mut jumps: Vec<(f64, f64, f64)> = Vec::new();
        for target in readers {
            if jumps
                .last()
                .is_some_and(|&(_, foot, across)| target < across && target > foot)
            {
                continue;
            }

            // The bracket stops closing once the readings at its ends are within a jump's least
            // size: the reading never falls along the edge, so nothing larger can stand between.
            let (mut lo, mut hi) = (0.0, 1.0);
            let (mut at_lo, mut at_hi) = (from, to);
            let mut step = 0;
            while step < 60 && (hi - lo) * length > tie && at_hi - at_lo > 1e3 * tie {
                super::LevelLineWork::charge(1)?;
                let mid = (lo + hi) / 2.0;
                let at_mid = read(mid)?;
                if at_mid < target {
                    (lo, at_lo) = (mid, at_mid);
                } else {
                    (hi, at_hi) = (mid, at_mid);
                }
                step += 1;
            }

            if at_hi - at_lo <= 1e3 * tie {
                // A point of the edge reads it, to within a jump's least size.
                continue;
            }

            let (foot, across) = (at_lo, at_hi);
            let at = (lo + hi) / 2.0;
            let inside = target > foot + tie && target < across - tie;
            if across - foot <= 1e3 * tie
                || !inside
                || at < END_WINDOW
                || at > 1.0 - END_WINDOW
                || jumps.last().is_some_and(|&(t, _, _)| at <= t)
            {
                // A point reads it, the jump is an end vertex's to stand for ([`jump_near`]), or
                // it is the jump already found.
                continue;
            }

            jumps.push((at, foot, across));
        }

        let mut near = a;
        let mut stations: Vec<(i32, f64)> = Vec::new();
        for (at, foot, _) in jumps {
            let v = mesh.split_edge(near, b, pa + ((pb - pa) * at), true)?;
            if v == near || v == b || mesh.has_edge(near, b) {
                // The mesh already carries a vertex there and nothing was split: edge `near b`
                // still stands, so it is not recorded as split.
                continue;
            }

            splits.insert(key(near, b), v);
            let least = stations.last().map_or(from, |&(_, p)| p);
            stations.push((v, ieee_math::max_f64(least, ieee_math::min_f64(foot, to))));
            near = v;
        }

        for (s, &(vertex, parameter)) in stations.iter().enumerate() {
            vertices.insert(i + 1 + s, vertex);
            parameters.insert(i + 1 + s, parameter);
        }

        i += stations.len() + 1;
    }

    if closed {
        vertices.pop();
        parameters.pop();
        let past = parameters
            .iter()
            .position(|&p| p >= closed_length)
            .unwrap_or(parameters.len());
        let mut led_vertices = vertices.split_off(past);
        let mut led_parameters: Vec<f64> = parameters
            .split_off(past)
            .into_iter()
            .map(|p| p - closed_length)
            .collect();
        led_vertices.append(vertices);
        led_parameters.append(parameters);
        *vertices = led_vertices;
        *parameters = led_parameters;
    }

    Ok(())
}
