//! Every tangent chain of one [`FeatureEdgeSet`], walked once ([`TangentChainCache`]), so that the
//! chain through any edge of the set is read off a table instead of walked again. Held by the set
//! itself ([`FeatureEdgeSet::prepare_tangent_chains`], read through
//! [`FeatureEdgeSet::tangent_chain_through`]) and read by [`super::pick_interpretations`]: a hover
//! onto each new edge of a rim twenty thousand edges long would otherwise walk all twenty thousand
//! of them again, several milliseconds per edge.
//!
//! The table answers exactly what [`FeatureEdgeChain::propagate_with`] answers, because a tangent
//! walk's chain does not depend on which of its edges seeded it, only on how that edge is stored:
//!
//! - The walk continues only through a vertex where exactly two feature edges meet, so from any
//!   edge of a chain there is one way on in each direction, the same way the first walk went.
//! - Its test is symmetric: walked the other way, every direction is negated (an exact operation)
//!   and the left and right faces swap, and the test checks both faces with the same tolerance,
//!   so a joint one walk refuses is refused by every walk that reaches it.
//!
//! So the chain through edge `k` of a cached chain is the cached chain itself when the cache walks
//! `k` in its stored direction, and the cached chain turned round (every edge reversed, the two
//! ends swapped) when it walks `k` against it; a closed chain is in addition rotated to start at
//! `k`, its loop vertex being where `k` is entered. It follows too that every edge lies in exactly
//! one chain, which the build checks in debug builds.
//!
//! A chain is kept as its edges' indices into the set and their walk directions, not copies of the
//! edges: a reading is rebuilt from the set's own edges, which are what a walk copies anyway.

use std::sync::atomic::{AtomicUsize, Ordering};

use super::feature_edge_chain::{ChainEdge, ChainEnd, ChainStopReason, FeatureEdgeChain};
use super::feature_edge_set::FeatureEdgeSet;

/// One walked chain: each edge as its index in the set and whether the walk runs it against its
/// stored direction, the chain's ends, and the sorted mesh edge indices it selects (how readings
/// are told apart, the same whichever edge seeded the chain).
#[derive(Debug)]
struct CachedChain {
    edges: Vec<(i32, bool)>,
    is_closed: bool,
    start: ChainEnd,
    end: ChainEnd,
    signature: Vec<i32>,
}

/// Every tangent chain of one feature edge set at one tolerance (used for both the direction and
/// the face test, as a stored group re-propagates it), with each edge's place in its chain.
#[derive(Debug)]
pub struct TangentChainCache {
    tolerance_degrees: f64,
    chains: Vec<CachedChain>,
    /// Per feature edge: its chain and its position in that chain's walk order. Every edge has
    /// one, because every edge seeds a chain of its own if no earlier one reached it.
    places: Vec<(u32, u32)>,
    // How many chains have been read off the table; only a test asks, to see that a hover walked
    // nothing. An atomic because the cache is read from whichever thread holds the set.
    served: AtomicUsize,
}

impl TangentChainCache {
    /// Walks every chain of `features` at `tolerance_degrees`: each edge is walked as part of the
    /// first chain that reaches it, so the whole set costs one walk of each edge.
    pub fn build(features: &FeatureEdgeSet, tolerance_degrees: f64) -> TangentChainCache {
        const UNPLACED: (u32, u32) = (u32::MAX, u32::MAX);
        let count = features.count();
        let mut chains: Vec<CachedChain> = Vec::new();
        let mut places = vec![UNPLACED; count];

        for seed in 0..count {
            if places[seed] != UNPLACED {
                continue;
            }
            // The seed is in range, which is the only thing the walk refuses.
            let Ok(chain) = FeatureEdgeChain::propagate_with(
                features,
                seed as i32,
                tolerance_degrees,
                tolerance_degrees,
            ) else {
                continue;
            };

            // A chain count and a chain length are both bounded by the edge count, which an
            // `i32` edge index already bounds, so neither cast can overflow.
            let chain_index = chains.len() as u32;
            for (position, edge) in chain.edges().iter().enumerate() {
                let place = &mut places[edge.edge_index as usize];
                debug_assert_eq!(
                    *place, UNPLACED,
                    "a tangent walk reaches each edge from one chain only (module header)"
                );
                *place = (chain_index, position as u32);
            }
            let mut signature: Vec<i32> = chain
                .edges()
                .iter()
                .map(|edge| edge.edge.mesh_edge_index)
                .collect();
            signature.sort_unstable();
            chains.push(CachedChain {
                edges: chain
                    .edges()
                    .iter()
                    .map(|edge| (edge.edge_index, edge.is_reversed))
                    .collect(),
                is_closed: chain.is_closed(),
                start: chain.start(),
                end: chain.end(),
                signature,
            });
        }

        TangentChainCache {
            tolerance_degrees,
            chains,
            places,
            served: AtomicUsize::new(0),
        }
    }

    /// How many chains [`Self::chain_through`] has handed out, each one a walk saved.
    pub fn served_count(&self) -> usize {
        self.served.load(Ordering::Relaxed)
    }

    /// The tolerance the chains were walked at.
    pub fn tolerance_degrees(&self) -> f64 {
        self.tolerance_degrees
    }

    /// The chain [`FeatureEdgeChain::propagate_with`] walks over `features` (the set this table
    /// was built from) from `seed_edge_index` at `tolerance` for both tests, and the sorted mesh
    /// edge indices it selects. `None` when the table was walked at another tolerance, the seed
    /// is out of range, or `features` is plainly not the set the table was built from (another
    /// edge count), rather than a chain of edges that set does not have.
    pub fn chain_through(
        &self,
        features: &FeatureEdgeSet,
        seed_edge_index: i32,
        tolerance_degrees: f64,
    ) -> Option<(FeatureEdgeChain, &[i32])> {
        if tolerance_degrees != self.tolerance_degrees
            || seed_edge_index < 0
            || features.count() != self.places.len()
        {
            return None;
        }
        let &(chain, position) = self.places.get(seed_edge_index as usize)?;
        let cached = self.chains.get(chain as usize)?;
        self.served.fetch_add(1, Ordering::Relaxed);
        Some((
            seeded_at(features, cached, position as usize),
            &cached.signature,
        ))
    }
}

/// `chain` as a walk seeded at its edge `position`, in that edge's stored direction, would give it.
fn seeded_at(features: &FeatureEdgeSet, chain: &CachedChain, position: usize) -> FeatureEdgeChain {
    let edges = &chain.edges;
    let n = edges.len();
    let turned_round = edges[position].1;
    // The edge as the walk ran it, or turned round.
    let edge_at = |i: usize, flip: bool| {
        let (index, reversed) = edges[i];
        ChainEdge::new(index, features.edges()[index as usize], reversed != flip)
    };

    if chain.is_closed {
        // A walk round a loop starts at its seed and comes back to it; walking the seed against
        // the cached direction goes round the other way.
        let walk: Vec<ChainEdge> = if turned_round {
            (0..n)
                .map(|i| edge_at((position + n - i) % n, true))
                .collect()
        } else {
            (0..n).map(|i| edge_at((position + i) % n, false)).collect()
        };
        let entry = ChainEnd::new(ChainStopReason::Loop, walk[0].from_vertex());
        return FeatureEdgeChain::from_parts(walk, true, entry, entry);
    }

    if turned_round {
        let walk: Vec<ChainEdge> = (0..n).rev().map(|i| edge_at(i, true)).collect();
        FeatureEdgeChain::from_parts(walk, false, chain.end, chain.start)
    } else {
        let walk: Vec<ChainEdge> = (0..n).map(|i| edge_at(i, false)).collect();
        FeatureEdgeChain::from_parts(walk, false, chain.start, chain.end)
    }
}
