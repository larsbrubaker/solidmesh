//! What a pick must be before a level-line build will take it. Ports the type of
//! `Bevel/LevelLine/PickValidation.cs` that is not the C# static class `PickValidation`: a pick
//! corner ([`PickCorner`]), with the pick's corners ([`corners_of`]) and the unpicked-crease test
//! ([`an_unpicked_crease_runs_into`]). The argument check ([`check_pick_arguments`],
//! refusing with [`PickArgumentError`] where C# throws) and the touching-chain pairs
//! ([`every_touching_pair`]) land here too; the validation's other functions land with later
//! packages.

use std::collections::{BTreeMap, HashSet};

use vector_math::Vector3;

use super::{chain_on_the_mesh, triangle_edges};

/// C# `PickCorner`: a vertex where picked chains turn into each other - see
/// `PickValidation.CornersOf`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PickCorner {
    /// The input vertex the chains meet at.
    pub vertex: i32,
    /// Every chain of the pick holding the vertex, ascending.
    pub chains: Vec<i32>,
}

impl PickCorner {
    /// C#'s constructor.
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
    // Read in ascending vertex order (C#'s `OrderBy(h => h.Key)`), so a sorted map.
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

/// Where C# `PickValidation.CheckPickArguments` throws an `ArgumentException`: a caller's bug,
/// reported with C#'s message rather than a panic. (C#'s `ArgumentNullException`s for the
/// positions and indices have no Rust case.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickArgumentError {
    /// No chains (C# "a pick is at least one chain").
    NoChains,
    /// A closed flag missing or extra (C# "every chain of the pick says whether it is closed").
    ClosedFlagsDoNotMatch,
    /// A chain of fewer than two vertices (C# "a chain is at least two vertices").
    ChainTooShort,
    /// A closed chain of fewer than three (C# "a closed chain is at least three vertices").
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

/// The pairs of [`every_touching_pair`], lazily (C#'s iterator).
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
