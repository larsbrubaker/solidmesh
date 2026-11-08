//! What a pick must be before a level-line build will take it: a pick corner ([`PickCorner`]),
//! with the pick's corners ([`corners_of`]) and the unpicked-crease test
//! ([`an_unpicked_crease_runs_into`]), the argument check ([`check_pick_arguments`], refusing with
//! [`PickArgumentError`]) and the touching-chain pairs ([`every_touching_pair`]), with the
//! validation on the input itself
//! ([`validate_on_the_input`], [`touch_is_not_a_corner`]).

use std::collections::{BTreeMap, HashSet};

use solidmesh_math::Vector3;

use super::{chain_on_the_mesh, triangle_edges, BandGapRefusal, PickFault};

/// A vertex where picked chains turn into each other - see
/// `PickValidation.CornersOf`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PickCorner {
    /// The input vertex the chains meet at.
    pub vertex: i32,
    /// Every chain of the pick holding the vertex, ascending.
    pub chains: Vec<i32>,
}

impl PickCorner {
    /// A corner at `vertex`, held by `chains`.
    pub fn new(vertex: i32, chains: Vec<i32>) -> Self {
        Self { vertex, chains }
    }
}

/// The pick's corners, in vertex order: each vertex two or more chains hold where at least one of
/// them ends, and the chains turn into each other - three or more chain edges meet there, or (with
/// `setback_steps_are_corners`) two chains end there.
pub fn corners_of(
    chains: &[Vec<i32>],
    closed: &[bool],
    setback_steps_are_corners: bool,
) -> Vec<PickCorner> {
    // Read in ascending vertex order, so the corners come out the same on every run: a sorted map.
    let mut holders: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    for (k, chain) in chains.iter().enumerate() {
        for &v in chain {
            holders.entry(v).or_default().push(k as i32);
        }
    }

    let mut corners = Vec::new();
    for (&v, members) in holders.iter().filter(|h| h.1.len() >= 2) {
        let ends = members
            .iter()
            .filter(|&&k| {
                let chain = &chains[k as usize];
                !closed[k as usize] && (chain[0] == v || chain[chain.len() - 1] == v)
            })
            .count();
        let mut edges = HashSet::new();
        for &k in members {
            for (from, to) in
                chain_on_the_mesh::chain_edges(&chains[k as usize], closed[k as usize])
            {
                if from == v || to == v {
                    edges.insert(triangle_edges::key(from, to));
                }
            }
        }

        if ends >= 1 && (edges.len() >= 3 || (setback_steps_are_corners && ends >= 2)) {
            let mut sorted = members.clone();
            sorted.sort_unstable();
            corners.push(PickCorner::new(v, sorted));
        }
    }

    corners
}

/// True when a feature edge at `v` that is not one of the picked edges stands in the mesh.
pub fn an_unpicked_crease_runs_into(
    indices: &[(i32, i32, i32)],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    picked_keys: &HashSet<(i32, i32)>,
    v: i32,
) -> bool {
    for &(a, b, c) in indices {
        for (from, to) in [(a, b), (b, c), (c, a)] {
            if (from == v || to == v)
                && !picked_keys.contains(&triangle_edges::key(from, to))
                && is_feature_edge(from, to)
            {
                return true;
            }
        }
    }

    false
}

/// Why [`check_pick_arguments`] refused: a caller's bug, reported with a message rather than a
/// panic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickArgumentError {
    /// No chains: a pick is at least one chain.
    NoChains,
    /// A closed flag missing or extra: every chain of the pick says whether it is closed.
    ClosedFlagsDoNotMatch,
    /// A chain of fewer than two vertices.
    ChainTooShort,
    /// A closed chain of fewer than three.
    ClosedChainTooShort,
}

impl std::fmt::Display for PickArgumentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PickArgumentError::NoChains => "a pick is at least one chain",
            PickArgumentError::ClosedFlagsDoNotMatch => {
                "every chain of the pick says whether it is closed"
            }
            PickArgumentError::ChainTooShort => "a chain is at least two vertices",
            PickArgumentError::ClosedChainTooShort => "a closed chain is at least three vertices",
        })
    }
}

impl std::error::Error for PickArgumentError {}

/// Checks a pick's arguments before a level-line build takes it: at least one chain, a closed
/// flag per chain, every chain at least two vertices and every closed one at least three.
pub fn check_pick_arguments(
    _positions: &[Vector3],
    _indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
) -> Result<(), PickArgumentError> {
    if chains.is_empty() {
        return Err(PickArgumentError::NoChains);
    }

    if closed.len() != chains.len() {
        return Err(PickArgumentError::ClosedFlagsDoNotMatch);
    }

    for (chain, &is_closed) in chains.iter().zip(closed) {
        if chain.len() < 2 {
            return Err(PickArgumentError::ChainTooShort);
        }

        if is_closed && chain.len() < 3 {
            return Err(PickArgumentError::ClosedChainTooShort);
        }
    }

    Ok(())
}

/// Every pair of chains that share a vertex, as `(chain, other, the first shared vertex of
/// chain)`, chain before other, in order.
pub fn every_touching_pair(chains: &[Vec<i32>]) -> Vec<(i32, i32, i32)> {
    touching_pairs(chains).collect()
}

/// The pairs of [`every_touching_pair`], lazily.
pub fn touching_pairs(chains: &[Vec<i32>]) -> impl Iterator<Item = (i32, i32, i32)> + '_ {
    (0..chains.len()).flat_map(move |i| {
        ((i + 1)..chains.len()).filter_map(move |j| {
            let theirs: HashSet<i32> = chains[j].iter().copied().collect();

            chains[i]
                .iter()
                .find(|v| theirs.contains(v))
                .map(|&v| (i as i32, j as i32, v))
        })
    })
}

/// The pick's fault on the input mesh, if it has one - a chain that comes
/// back to a vertex it has been through, a chain edge not on two triangles, or two chains touching
/// anywhere but at a corner (or at a corner an unpicked crease runs into) - and otherwise the pick's
/// corners. `is_feature_edge` `None` means the part has no feature edges but the picked ones.
/// `_positions` is not read: the checks need only the topology.
pub fn validate_on_the_input(
    _positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
) -> (Option<PickFault>, Vec<PickCorner>) {
    let no_feature_edges = |_: i32, _: i32| false;
    let is_feature_edge: &dyn Fn(i32, i32) -> bool = is_feature_edge.unwrap_or(&no_feature_edges);

    // First: a chain that comes back to a vertex it has already been through is a figure eight.
    for (k, chain) in chains.iter().enumerate() {
        let mut seen = HashSet::new();
        for &v in chain {
            if !seen.insert(v) {
                let fault = PickFault {
                    refusal: BandGapRefusal::ChainTouchesItself,
                    chain: k as i32,
                    vertex: v,
                    ..PickFault::default()
                };
                return (Some(fault), Vec::new());
            }
        }
    }

    let standing = triangle_edges::standing_map(indices);
    for (k, chain) in chains.iter().enumerate() {
        for (from, to) in chain_on_the_mesh::chain_edges(chain, closed[k]) {
            if let Err(bad) = chain_on_the_mesh::sides_of(indices, &standing, from, to) {
                let fault = PickFault {
                    refusal: BandGapRefusal::ChainEdgeNotOnTwoTriangles,
                    chain: k as i32,
                    edge: (bad.from, bad.to),
                    ..PickFault::default()
                };
                return (Some(fault), Vec::new());
            }
        }
    }

    // Chains that meet are built only at a CORNER; any other touch is refused by name.
    let found = corners_of(chains, closed, false);
    let corner_at: HashSet<i32> = found.iter().map(|c| c.vertex).collect();
    let mut picked_keys: Option<HashSet<(i32, i32)>> = None;
    for (chain, other, _) in touching_pairs(chains) {
        if let Some(fault) = touch_is_not_a_corner(
            indices,
            chains,
            closed,
            is_feature_edge,
            &corner_at,
            chain,
            other,
            &mut picked_keys,
        ) {
            return (Some(fault), Vec::new());
        }
    }

    (None, found)
}

/// The fault where chain `i` touches chain `j` other than at a corner
/// (sharing an edge, or at a vertex no corner stands at), or at a corner an unpicked crease runs
/// into; `None` where every touch is a good corner. `picked_keys` is the picked edges' keys, read
/// once on first need and kept by the caller. `corner_at` holds the corners' vertices (asked only
/// whether it has one).
#[allow(clippy::too_many_arguments)]
pub fn touch_is_not_a_corner(
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    corner_at: &HashSet<i32>,
    i: i32,
    j: i32,
    picked_keys: &mut Option<HashSet<(i32, i32)>>,
) -> Option<PickFault> {
    let (iu, ju) = (i as usize, j as usize);
    let theirs: HashSet<i32> = chains[ju].iter().copied().collect();
    let their_edges: HashSet<(i32, i32)> = chain_on_the_mesh::chain_edges(&chains[ju], closed[ju])
        .into_iter()
        .map(|(from, to)| triangle_edges::key(from, to))
        .collect();
    let shares_an_edge = chain_on_the_mesh::chain_edges(&chains[iu], closed[iu])
        .into_iter()
        .any(|(from, to)| their_edges.contains(&triangle_edges::key(from, to)));

    for &v in &chains[iu] {
        if !theirs.contains(&v) {
            continue;
        }

        if shares_an_edge || !corner_at.contains(&v) {
            return Some(PickFault {
                refusal: BandGapRefusal::ChainsTouch,
                chain: i,
                other: j,
                vertex: v,
                ..PickFault::default()
            });
        }

        let keys = picked_keys.get_or_insert_with(|| {
            chains
                .iter()
                .enumerate()
                .flat_map(|(k, c)| chain_on_the_mesh::chain_edges(c, closed[k]))
                .map(|(from, to)| triangle_edges::key(from, to))
                .collect()
        });
        if an_unpicked_crease_runs_into(indices, is_feature_edge, keys, v) {
            return Some(PickFault {
                refusal: BandGapRefusal::CornerOnAnUnpickedFace,
                chain: i,
                other: j,
                vertex: v,
                ..PickFault::default()
            });
        }
    }

    None
}
