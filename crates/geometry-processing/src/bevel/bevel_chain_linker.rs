//! Linking a group's selected edges into the runs a single swept tool is built along: one such run
//! ([`BevelSweptChain`]) and the linking - walking the selected edges into runs ([`link`],
//! [`link_resolution`]), and splitting a run where its convexity turns over
//! ([`split_by_convexity`]) or where one mitred sweep cannot carry it
//! ([`split_where_the_mitre_cannot_fit`]).

use std::collections::HashMap;

use solidmesh_math::ieee_math;

use super::bevel_chain_sweep_sections::try_build_segments;
use super::{bevel_mitre_fit, bevel_sweep_back_depth};
use crate::topology::{BevelEdgeGroupResolution, ChainEdge, FeatureEdge};

/// One run of edges a single swept tool is built along - the edges head to tail, and whether the
/// run closes on itself.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelSweptChain {
    /// The edges in sweep order, each oriented along the sweep so consecutive edges meet head to
    /// tail.
    pub edges: Vec<ChainEdge>,
    /// Whether the last edge runs back into the first, in which case the sweep welds rather than
    /// caps.
    pub is_closed: bool,
}

impl BevelSweptChain {
    /// A run of `edges`, closed into a loop or not.
    pub fn new(edges: Vec<ChainEdge>, is_closed: bool) -> Self {
        BevelSweptChain { edges, is_closed }
    }
}

/// How near the two side readings of a joint have to come to be read as a tie.
pub const SIDE_EVIDENCE_TIE: f64 = 1e-9;

/// The runs a resolved group sweeps along: its one chain where the resolver found one, and the
/// runs its edges link into otherwise.
pub fn link_resolution(resolution: &BevelEdgeGroupResolution) -> Vec<BevelSweptChain> {
    if let Some(chain) = &resolution.chain {
        return vec![BevelSweptChain::new(
            chain.edges().to_vec(),
            chain.is_closed(),
        )];
    }

    link(&resolution.edges)
}

/// `chain` split into runs of one convexity each - started, on a closed chain, where its
/// convexity turns over - or `chain` itself where it has one convexity throughout.
pub fn split_by_convexity(chain: BevelSweptChain) -> Vec<BevelSweptChain> {
    let edges = &chain.edges;
    let first = edges[0].edge.convexity;

    if edges.iter().all(|e| e.edge.convexity == first) {
        return vec![chain];
    }

    let mut start = 0;

    if chain.is_closed {
        let count = edges.len();
        if let Some(i) = (0..count)
            .find(|&i| edges[i].edge.convexity != edges[(i + count - 1) % count].edge.convexity)
        {
            start = i;
        }
    }

    let mut runs = Vec::new();
    let mut current: Vec<ChainEdge> = Vec::new();

    for step in 0..edges.len() {
        let edge = edges[(start + step) % edges.len()];

        if !current.is_empty() && edge.edge.convexity != current[0].edge.convexity {
            runs.push(BevelSweptChain::new(std::mem::take(&mut current), false));
        }

        current.push(edge);
    }

    runs.push(BevelSweptChain::new(current, false));

    runs
}

/// The selected `edges` walked into runs: each unclaimed edge seeds a run that grows forward and
/// then backward through vertices with exactly two selected edges whose sides agree; a run that
/// comes back to its seed (either way round, or end to start) is closed.
pub fn link(edges: &[FeatureEdge]) -> Vec<BevelSweptChain> {
    let mut chains = Vec::new();

    if edges.is_empty() {
        return chains;
    }

    let mut at_vertex: HashMap<i32, Vec<i32>> = HashMap::new();

    for (i, edge) in edges.iter().enumerate() {
        add_incidence(&mut at_vertex, edge.start_vertex, i as i32);
        add_incidence(&mut at_vertex, edge.end_vertex, i as i32);
    }

    let mut used = vec![false; edges.len()];

    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }

        used[seed] = true;

        let mut walk = vec![ChainEdge::new(seed as i32, edges[seed], false)];

        let mut closed = grow(edges, &at_vertex, &mut used, &mut walk, seed as i32);

        if !closed {
            // The other direction is the same walk over the run turned around, so it goes
            // through the same code and is flipped back afterwards. Its answer counts as much as
            // the forward one's: a closure missed here becomes an OPEN run whose two capped ends
            // sit on top of each other.
            let mut backward = vec![walk[0].reversed()];

            closed = grow(edges, &at_vertex, &mut used, &mut backward, seed as i32);

            backward.remove(0);
            backward.reverse();

            let mut joined: Vec<ChainEdge> = backward.iter().map(|e| e.reversed()).collect();
            joined.extend(walk);
            walk = joined;
        }

        let closes = closed || closes_on_itself(&walk);
        chains.push(BevelSweptChain::new(walk, closes));
    }

    chains
}

/// Whether an open walk ends where it began, with its sides agreeing across that joint.
pub fn closes_on_itself(walk: &[ChainEdge]) -> bool {
    let first = walk[0];
    let last = walk[walk.len() - 1];

    walk.len() > 1 && first.from_vertex() == last.to_vertex() && sides_agree(&last, &first)
}

/// Grows `walk` forward through vertices with exactly two selected edges whose sides agree,
/// claiming each edge it takes; true where it comes back to the `seed`.
pub fn grow(
    edges: &[FeatureEdge],
    at_vertex: &HashMap<i32, Vec<i32>>,
    used: &mut [bool],
    walk: &mut Vec<ChainEdge>,
    seed: i32,
) -> bool {
    loop {
        let current = walk[walk.len() - 1];
        let vertex = current.to_vertex();

        let Some(incident) = at_vertex
            .get(&vertex)
            .filter(|incident| incident.len() == 2)
        else {
            // A vertex with one selected edge is where the run ends; three or more is a junction
            // the sweep would have to guess its way through.
            return false;
        };

        let next_index = if incident[0] == current.edge_index {
            incident[1]
        } else {
            incident[0]
        };

        let next_edge = edges[next_index as usize];
        let next = ChainEdge::new(next_index, next_edge, next_edge.end_vertex == vertex);

        if !sides_agree(&current, &next) {
            return false;
        }

        if next_index == seed {
            return true;
        }

        if used[next_index as usize] {
            // Already claimed by another run: it cannot be walked twice, and it is not the
            // closure of this one either.
            return false;
        }

        used[next_index as usize] = true;
        walk.push(next);
    }
}

/// Whether `next` keeps `current`'s sides - its left face on the left - read off shared faces
/// first, and off the side normals where the two share none: the pairing whose normals agree
/// best, with a near tie settled by the sums.
pub fn sides_agree(current: &ChainEdge, next: &ChainEdge) -> bool {
    if current.left_face() == next.left_face() || current.right_face() == next.right_face() {
        return true;
    }

    if current.left_face() == next.right_face() || current.right_face() == next.left_face() {
        return false;
    }

    let agreeing_left = current.left_normal().dot(next.left_normal());
    let agreeing_right = current.right_normal().dot(next.right_normal());
    let swapped_left = current.left_normal().dot(next.right_normal());
    let swapped_right = current.right_normal().dot(next.left_normal());

    let agreeing_best = ieee_math::max_f64(agreeing_left, agreeing_right);
    let swapped_best = ieee_math::max_f64(swapped_left, swapped_right);

    if (agreeing_best - swapped_best).abs() > SIDE_EVIDENCE_TIE {
        return agreeing_best > swapped_best;
    }

    agreeing_left + agreeing_right >= swapped_left + swapped_right
}

/// Records `edge_index` as standing at `vertex`.
pub fn add_incidence(at_vertex: &mut HashMap<i32, Vec<i32>>, vertex: i32, edge_index: i32) {
    at_vertex
        .entry(vertex)
        .or_insert_with(|| Vec::with_capacity(2))
        .push(edge_index);
}

/// `chain` split where a single mitred sweep cannot carry it - or kept whole at a shallower back
/// depth where that alone lets every joint fit - and that back depth (zero where none was needed).
/// The short form is [`split_where_the_mitre_cannot_fit`].
pub fn split_where_the_mitre_cannot_fit_with_depth(
    chain: BevelSweptChain,
    size: f64,
    segments: i32,
    round_profile: bool,
) -> (Vec<BevelSweptChain>, f64) {
    let Some(cannot_fit) = bevel_mitre_fit::joints_the_mitre_cannot_fit(
        &chain.edges,
        chain.is_closed,
        size,
        segments,
        round_profile,
        0.0,
    ) else {
        return (vec![chain], 0.0);
    };

    if let Ok(swept) = try_build_segments(&chain.edges, size, segments, round_profile, 0.0) {
        let shallower = bevel_sweep_back_depth::retry_depth(
            &chain.edges,
            chain.is_closed,
            size,
            bevel_sweep_back_depth::measure_back_reach(&swept),
            f64::INFINITY,
        );

        if shallower > 0.0
            && bevel_mitre_fit::joints_the_mitre_cannot_fit(
                &chain.edges,
                chain.is_closed,
                size,
                segments,
                round_profile,
                shallower,
            )
            .is_none()
        {
            return (vec![chain], shallower);
        }
    }

    let edges = &chain.edges;
    let mut start = 0;

    if chain.is_closed {
        if let Some(joint) = cannot_fit.iter().position(|&c| c) {
            start = joint;
        }
    }

    let mut runs = Vec::new();
    let mut current: Vec<ChainEdge> = Vec::new();

    for step in 0..edges.len() {
        let index = (start + step) % edges.len();

        if !current.is_empty() && cannot_fit[index] {
            runs.push(BevelSweptChain::new(std::mem::take(&mut current), false));
        }

        current.push(edges[index]);
    }

    runs.push(BevelSweptChain::new(current, false));

    (runs, 0.0)
}

/// [`split_where_the_mitre_cannot_fit_with_depth`] without the back depth.
pub fn split_where_the_mitre_cannot_fit(
    chain: BevelSweptChain,
    size: f64,
    segments: i32,
    round_profile: bool,
) -> Vec<BevelSweptChain> {
    split_where_the_mitre_cannot_fit_with_depth(chain, size, segments, round_profile).0
}
