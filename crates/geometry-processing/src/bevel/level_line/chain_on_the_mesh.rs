//! A picked chain read on the mesh it was picked from: its edges in its own direction and the two
//! triangles each stands between, with the setback-step fraction ([`SETBACK_STEP_FRACTION`]), the
//! setback steps (and the cove-to-round flips cut with them, and whether a set of flips cuts more
//! than the last: [`flips_cut_more`]), each edge's convexity and the unit setbacks.

use super::triangle_edges::{self, StandingMap};
use super::ChainEdgeNotOnTwoTriangles;

/// The chain's edges, each in the chain's own direction.
pub fn chain_edges(chain: &[i32], is_closed: bool) -> Vec<(i32, i32)> {
    let mut edges = Vec::new();
    let mut i = 0;
    while i + 1 < chain.len() {
        edges.push((chain[i], chain[i + 1]));
        i += 1;
    }

    if is_closed {
        edges.push((chain[chain.len() - 1], chain[0]));
    }

    edges
}

/// The two triangles a chain edge `from -> to` stands between, as (left, right): the left one
/// traverses the edge in the chain's direction. Errors where the edge does not stand on exactly
/// two triangles, or its two triangles do not wind it consistently.
pub fn sides_of(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    from: i32,
    to: i32,
) -> Result<(i32, i32), ChainEdgeNotOnTwoTriangles> {
    let pair = match standing.get(&triangle_edges::key(from, to)) {
        Some(pair) if pair.len() == 2 => pair,
        _ => {
            return Err(ChainEdgeNotOnTwoTriangles::new(
                from,
                to,
                format!("chain edge {from}-{to} does not stand on exactly two triangles"),
            ))
        }
    };

    let left = if triangle_edges::traverses(indices[pair[0] as usize], from, to) {
        pair[0]
    } else {
        pair[1]
    };
    if !triangle_edges::traverses(indices[left as usize], from, to) {
        return Err(ChainEdgeNotOnTwoTriangles::new(
            from,
            to,
            format!("chain edge {from}-{to} is not wound consistently by its two triangles"),
        ));
    }

    Ok((left, if left == pair[0] { pair[1] } else { pair[0] }))
}

/// How much two neighbouring edges' setbacks may differ,
/// as a fraction of the larger, and still share a sharp turn of one chain. Below it the vertex
/// takes their mean; above it the chain is split there.
pub const SETBACK_STEP_FRACTION: f64 = 0.05;

/// Each chain edge's setback at size 1 - the chamfer's
/// own, or the fillet's at the edge's dihedral between its two triangles.
pub fn per_edge_unit_setbacks(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    profile: super::BandProfile,
) -> Result<Vec<f64>, ChainEdgeNotOnTwoTriangles> {
    let standing = triangle_edges::standing_map(indices);
    let mut per_edge = Vec::with_capacity(chain_edges.len());

    for &(from, to) in chain_edges {
        let (left, right) = sides_of(indices, &standing, from, to)?;
        per_edge.push(if profile == super::BandProfile::Chamfer {
            super::chain_setback::for_chamfer(1.0)
        } else {
            super::chain_setback::for_fillet(
                1.0,
                triangle_edges::normal(positions, indices[left as usize]),
                triangle_edges::normal(positions, indices[right as usize]),
            )
        });
    }

    Ok(per_edge)
}

/// Whether an unpicked feature edge runs into chain vertex `v`, other than the chain's own edges
/// to `a` and `b` (always, without an `is_feature_edge`): where a chain may be cut, at a setback
/// step or a cove-to-round flip.
pub fn a_crease_runs_in(
    indices: &[(i32, i32, i32)],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
    a: i32,
    v: i32,
    b: i32,
) -> bool {
    let Some(is_feature_edge) = is_feature_edge else {
        return true;
    };

    indices.iter().any(|&(x, y, z)| {
        [(x, y), (y, z), (z, x)].into_iter().any(|(u, w)| {
            let other = if u == v {
                w
            } else if w == v {
                u
            } else {
                -1
            };
            other >= 0 && other != a && other != b && is_feature_edge(v, other)
        })
    })
}

/// The vertices within `reach` of `point` where a picked chain's crease flips between a cove and a
/// round and a crease runs in, so the chain could be cut there: the narrow trigger of the fallback
/// that cuts a rim at its flips. A rim refused because a rail ran backward is cut only at the
/// flips next to where the refusal stands; every other flip stays blended.
pub fn sense_flips_near(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
    point: vector_math::Vector3,
    reach: f64,
) -> Result<Vec<i32>, ChainEdgeNotOnTwoTriangles> {
    let mut near = Vec::new();
    for (k, chain) in chains.iter().enumerate() {
        let edges = chain_edges(chain, closed[k]);
        let convex = per_edge_convexity(positions, indices, &edges)?;
        let n = edges.len();
        let joints = if closed[k] { n } else { n.saturating_sub(1) };
        for i in 0..joints {
            let j = (i + 1) % n;
            let vertex = edges[j].0;
            if convex[i] != convex[j]
                && (positions[vertex as usize] - point).length() <= reach
                && a_crease_runs_in(indices, is_feature_edge, edges[i].0, vertex, edges[j].1)
                && !near.contains(&vertex)
            {
                near.push(vertex);
            }
        }
    }

    Ok(near)
}

/// Whether each chain edge's crease is convex: the far triangle's opposite corner lies behind the
/// near triangle's plane (`end_face_sense::is_convex`).
pub fn per_edge_convexity(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
) -> Result<Vec<bool>, ChainEdgeNotOnTwoTriangles> {
    let standing = triangle_edges::standing_map(indices);
    chain_edges
        .iter()
        .map(|&(from, to)| {
            let (left, right) = sides_of(indices, &standing, from, to)?;
            Ok(super::end_face_sense::is_convex(
                positions,
                indices[left as usize],
                indices[right as usize],
                from,
            ))
        })
        .collect()
}

/// Each chain vertex's setback at size 1, the mean of the
/// edges either side of it (the one edge at an open chain's end).
pub fn unit_setbacks(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    is_closed: bool,
    chain_edges: &[(i32, i32)],
    profile: super::BandProfile,
) -> Result<Vec<f64>, ChainEdgeNotOnTwoTriangles> {
    let per_edge = per_edge_unit_setbacks(positions, indices, chain_edges, profile)?;
    let edge_count = chain_edges.len() as i64;
    let mut setbacks = Vec::with_capacity(chain.len());

    for v in 0..chain.len() as i64 {
        let before = if is_closed {
            (v - 1 + edge_count) % edge_count
        } else {
            v - 1
        };
        let after = if is_closed { v % edge_count } else { v };
        let mut sum = 0.0;
        let mut count = 0.0;

        for side in [before, after] {
            if side >= 0 && (side as usize) < per_edge.len() {
                sum += per_edge[side as usize];
                count += 1.0;
            }
        }

        setbacks.push(sum / count);
    }

    Ok(setbacks)
}

/// Each chain cut into pieces at its STEPS - sharp
/// turns (past `sharp_turn_degrees`) where the unit setback jumps by more than
/// [`SETBACK_STEP_FRACTION`], and the vertices a build names
/// ([`WorkBudget::sense_flips_to_cut`](super::WorkBudget)) where the crease flips between concave
/// and convex ([`per_edge_convexity`]); in both cases where an unpicked crease runs into the
/// vertex ([`a_crease_runs_in`]). A closed chain needs two steps to be cut, and its pieces wrap past its
/// start. A chamfer, or no threshold, never steps; nothing cut hands the chains back as they are.
#[allow(clippy::too_many_arguments)]
pub fn split_at_setback_steps(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    profile: super::BandProfile,
    sharp_turn_degrees: f64,
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
) -> Result<(Vec<Vec<i32>>, Vec<bool>), ChainEdgeNotOnTwoTriangles> {
    if profile == super::BandProfile::Chamfer || !sharp_turn_degrees.is_finite() {
        // A chamfer's setback is its size on every edge: there is never a step.
        return Ok((chains.to_vec(), closed.to_vec()));
    }

    let mut split: Option<(Vec<Vec<i32>>, Vec<bool>)> = None;
    for (k, chain) in chains.iter().enumerate() {
        let edges = chain_edges(chain, closed[k]);
        let per_edge = per_edge_unit_setbacks(positions, indices, &edges, profile)?;
        let convex = per_edge_convexity(positions, indices, &edges)?;
        let flips_to_cut: Vec<i32> = super::LevelLineWork::current()
            .map(|budget| budget.sense_flips_to_cut.borrow().clone())
            .unwrap_or_default();
        let cos_sharp = (sharp_turn_degrees * std::f64::consts::PI / 180.0).cos();
        let steps_at = |before: usize, after: usize| {
            // Edge i runs from chain[i]; the vertex between edges before and after is chain[after].
            let d_in = positions[edges[before].1 as usize] - positions[edges[before].0 as usize];
            let d_out = positions[edges[after].1 as usize] - positions[edges[after].0 as usize];
            let sharp = d_in.length_squared() > 0.0
                && d_out.length_squared() > 0.0
                && d_in.get_normal().dot(d_out.get_normal()) < cos_sharp;
            let setback_steps = sharp
                && (per_edge[before] - per_edge[after]).abs()
                    > SETBACK_STEP_FRACTION
                        * vector_math::ieee_math::max_f64(per_edge[before], per_edge[after]);
            // A crease that flips from a cove to a round (or back) is cut there too, whatever its
            // setbacks: one run cannot be both, and each stops flush at the corner.
            let flips = convex[before] != convex[after] && flips_to_cut.contains(&edges[after].0);
            (setback_steps || flips)
                && a_crease_runs_in(
                    indices,
                    is_feature_edge,
                    edges[before].0,
                    edges[after].0,
                    edges[after].1,
                )
        };

        // The chain positions a step stands at: vertex i sits between edge i - 1 and edge i.
        let (first, end) = if closed[k] {
            (0, chain.len())
        } else {
            (1, chain.len().saturating_sub(1))
        };
        let steps: Vec<usize> = (first..end)
            .filter(|&i| steps_at((i + edges.len() - 1) % edges.len(), i))
            .collect();

        if steps.is_empty() || (closed[k] && steps.len() < 2) {
            if let Some((split_chains, split_closed)) = split.as_mut() {
                split_chains.push(chain.clone());
                split_closed.push(closed[k]);
            }
            continue;
        }

        let (split_chains, split_closed) =
            split.get_or_insert_with(|| (chains[..k].to_vec(), closed[..k].to_vec()));

        if closed[k] {
            // Each piece runs from one step round to the next, wrapping past the chain's start.
            for s in 0..steps.len() {
                let from = steps[s];
                let to = steps[(s + 1) % steps.len()];
                let length = ((to + chain.len() - from - 1) % chain.len()) + 1;
                split_chains.push(
                    (0..=length)
                        .map(|j| chain[(from + j) % chain.len()])
                        .collect(),
                );
                split_closed.push(false);
            }
        } else {
            let mut bounds = vec![0];
            bounds.extend(&steps);
            bounds.push(chain.len() - 1);
            for s in 0..bounds.len() - 1 {
                split_chains.push(chain[bounds[s]..=bounds[s + 1]].to_vec());
                split_closed.push(false);
            }
        }
    }

    Ok(split.unwrap_or_else(|| (chains.to_vec(), closed.to_vec())))
}

/// Whether naming `flips` changes the build from naming `last`: the split the build itself runs
/// ([`split_at_setback_steps`]), read with each set named on a budget of its own
/// (`WorkBudget::sense_flips_to_cut`), gives more pieces, or a newly named flip is a piece's end -
/// even one a setback step cuts already, since a named flip's end stops flush where a step's does
/// not. A flip that changes neither (a closed chain's lone flip, a vertex no crease runs into, so
/// no piece ends there) asks for a build that would only repeat the last one.
#[allow(clippy::too_many_arguments)]
pub fn flips_cut_more(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    profile: super::BandProfile,
    sharp_turn_degrees: f64,
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
    last: &[i32],
    flips: &[i32],
) -> bool {
    let split = |named: &[i32]| -> Option<(Vec<Vec<i32>>, Vec<bool>)> {
        let budget = super::WorkBudget::new(1).ok()?;
        *budget.sense_flips_to_cut.borrow_mut() = named.to_vec();
        let _work = super::LevelLineWork::begin(Some(std::rc::Rc::new(budget)));
        split_at_setback_steps(
            positions,
            indices,
            chains,
            closed,
            profile,
            sharp_turn_degrees,
            is_feature_edge,
        )
        .ok()
    };
    let (Some((before, _)), Some((after, after_closed))) = (split(last), split(flips)) else {
        return false;
    };
    // A closed piece's first vertex is only where it starts, no end.
    let is_an_end = |v: i32| {
        after
            .iter()
            .zip(&after_closed)
            .any(|(piece, &piece_closed)| {
                !piece_closed && (piece.first() == Some(&v) || piece.last() == Some(&v))
            })
    };
    after.len() > before.len()
        || flips
            .iter()
            .any(|&flip| !last.contains(&flip) && is_an_end(flip))
}
