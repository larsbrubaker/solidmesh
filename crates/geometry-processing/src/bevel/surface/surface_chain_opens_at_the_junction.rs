//! A chain that runs THROUGH a vertex where another chain ENDS, published as the open runs it is
//! from the junction's point of view (`Bevel/Surface/SurfaceChainOpensAtTheJunction.cs`, a C#
//! `internal static class`, so a module here). Runs after [`super::surface_chain_join::join`] and
//! before any plan is drawn; [`super::surface_interrupted_crease`] and the junction readings
//! ([`super::SurfaceMixedJunction`]) read the pieces it leaves.
//!
//! A junction is read off chain ENDS: `SurfaceMixedJunction.TryGatherTheEnds` wants three of them
//! at the vertex, because an end is what an arm of the patch is handed over at. A face rim taken
//! whole is one closed chain, and a closed chain has no ends anywhere - so the same three picked
//! edges that close a junction when they arrive as three runs were refused when one of them was
//! picked as a face. A pick is a set of edges however it was made, and this is where the two
//! readings are made one.
//!
//! Cutting is safe because nothing has been planned yet. What a chain buys at a joint is a MITRE -
//! two bands sharing one seam vertex - and a junction vertex is precisely a joint that may not be
//! mitred: the third band is taken out from between the two rails a mitre would have to share. So
//! a chain loses nothing it was going to use, its other corners still mitring, inside one run or
//! across the cut. A chain with junctions at BOTH of its ends is cut at neither, being three ends'
//! worth of chain already. Cutting an OPEN chain at an interior vertex of degree three is right for
//! the same reason it is right for a ring, and for three more: the corner there would have yielded
//! its mitre in any case (which is the whole of what the deleted yield survey did), a corner the
//! blend swallows or a mixed corner turns at belongs to the arm reading inside
//! `SurfaceMixedJunction` and is reached through the cut rather than through a mitre, and the
//! degree tallied is over EDGES and not over plans, so a chain that passes through counts its two
//! whether it arrived as a ring or as a run.
//!
//! Only degree exactly THREE opens a ring. Four or more picked edges at one vertex is no junction
//! any construction here reads, and two closed chains through one vertex - two rims meeting at a
//! point - counts four by the same tally; both stay closed and are refused downstream by the names
//! they have always had, rather than being opened into a shape nothing would build either.

use std::collections::HashMap;

use crate::bevel::BevelSweptChain;
use crate::topology::ChainEdge;

/// C# `SurfaceChainOpensAtTheJunction.Open`: the same chains, each cut open at every junction
/// vertex it passes through rather than stops at. A chain that passes through none of them comes
/// back exactly as it arrived (C# hands back the same reference; here the same value, moved).
pub fn open(chains: Vec<BevelSweptChain>) -> Vec<BevelSweptChain> {
    // How many picked edges stand at each vertex of the part, over ALL the chains: an interior
    // vertex of a run counts two, a run's own end counts one, and a vertex a second chain arrives at
    // counts one more. Three is a junction. Only looked up, never iterated, so order cannot leak.
    let mut degree: HashMap<i32, i32> = HashMap::new();

    for chain in &chains {
        for edge in &chain.edges {
            bump(&mut degree, edge.from_vertex());
            bump(&mut degree, edge.to_vertex());
        }
    }

    let mut answer = Vec::with_capacity(chains.len());

    for chain in chains {
        if chain.edges.is_empty() {
            answer.push(chain);
            continue;
        }

        // Where the chain is cut: the index of every edge that LEAVES a junction vertex the chain
        // does not already stop at. A closed chain may be cut anywhere, since it has no end of its
        // own; an open one only INSIDE itself, its first vertex being an end already.
        let mut cuts: Vec<usize> = Vec::new();

        for i in (if chain.is_closed { 0 } else { 1 })..chain.edges.len() {
            if degree.get(&chain.edges[i].from_vertex()) == Some(&3) {
                cuts.push(i);
            }
        }

        if cuts.is_empty() {
            answer.push(chain);
            continue;
        }

        // An open chain keeps the stretch before its first cut, which a closed one has no separate
        // start for - its last run wraps round into it instead.
        if !chain.is_closed {
            answer.push(run(&chain, 0, cuts[0]));
        }

        for cut in 0..cuts.len() {
            let from = cuts[cut];

            // The run reaches to the next junction; the last one reaches the chain's own finish, or
            // for a ring all the way round to the first cut - which for a single junction is the
            // whole ring, rotated to start there.
            let length = if cut + 1 < cuts.len() {
                cuts[cut + 1] - from
            } else {
                chain.edges.len() - from + if chain.is_closed { cuts[0] } else { 0 }
            };

            answer.push(run(&chain, from, length));
        }
    }

    answer
}

/// C# `Run`: one run of a chain, `length` edges from `from`, wrapping round the end for a ring, in
/// the order and the direction the chain walked them.
fn run(chain: &BevelSweptChain, from: usize, length: usize) -> BevelSweptChain {
    let mut run: Vec<ChainEdge> = Vec::with_capacity(length);

    for step in 0..length {
        run.push(chain.edges[(from + step) % chain.edges.len()]);
    }

    BevelSweptChain::new(run, false)
}

/// C# `Bump`.
fn bump(degree: &mut HashMap<i32, i32>, vertex: i32) {
    *degree.entry(vertex).or_insert(0) += 1;
}
