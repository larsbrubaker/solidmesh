//! What a pick must be before a level-line build will take it. Ports the type of
//! `Bevel/LevelLine/PickValidation.cs` that is not the C# static class `PickValidation`: a pick
//! corner ([`PickCorner`]), with the pick's corners ([`corners_of`]) and the unpicked-crease test
//! ([`an_unpicked_crease_runs_into`]). The validation's other functions land with later packages.

use std::collections::{BTreeMap, HashSet};

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
