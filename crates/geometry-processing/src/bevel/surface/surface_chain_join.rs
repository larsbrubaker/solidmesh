//! Chains from DIFFERENT groups that run into each other end to end, joined into one chain before
//! anything is planned. Its answer goes on to [`super::surface_chain_opens_at_the_junction::open`].
//!
//! The linker works one group at a time (`bevel_chain_linker::link(BevelEdgeGroupResolution)`),
//! because a group is what a user picked; but two edges of a cube corner picked one at a time are
//! two groups and one RUN, and the bevel over them is one strip mitred at the corner rather than
//! two strips that have to find each other there. Joining first means the mitre is the mitre a
//! closed rim already takes, with the chain itself fixing which rail of one strip meets which rail
//! of the next - and the corner cases left over are the ones that really are corners: one end alone
//! against a face, or three ends at a vertex no walk goes through.
//!
//! Only where exactly TWO chain ends meet. Three is a junction - the linker refuses to walk one for
//! the same reason, there being no single way on - and it is the corner patch's, not a joint's. A
//! chain whose own two ends come back together arrives that way and is left as it arrived: whether
//! that is a loop or a run that happens to return is the linker's question and it has already
//! answered it.
//!
//! What the join does answer is the ring it ASSEMBLED itself. The last pair of a ring is the pair
//! whose two ends belong to one chain by then, which is skipped for the rule above - so four edges
//! picked round a face would be published as a run whose first vertex is also its last, and
//! everything downstream would read that one vertex as two free chain ends (two ends of one plan in
//! `SurfaceStripEnds`, and a wrap pair `SurfaceMixedCornerReadings` never reads). Several chains
//! put end to end until they come back to the start is a LOOP, and nobody but the join is in a
//! position to say so.
//!
//! It says so on the same terms the linker does. A ring whose last edge meets its first with their
//! two surfaces SWAPPED is not a loop the sweep can carry one frame round, and
//! [`crate::bevel::bevel_chain_linker::sides_agree`] is what says which it is - so the same edges
//! read the same whether they were picked as one group or an edge at a time.
//!
//! And a ring it assembled is published OPEN on purpose where a THIRD end stands at the vertex it
//! closes on. Closing there would take the two ends the junction is read from away from it and
//! leave one arrival where there are three, so the joint that closes is asked how many ends stand
//! there as well as which way their surfaces face, and two is the only answer that makes a loop.

use std::collections::HashMap;

use indexmap::IndexMap;

use crate::bevel::bevel_chain_linker;
use crate::bevel::BevelSweptChain;
use crate::topology::ChainEdge;

/// The same chains with every end-to-end pair joined, repeatedly, until
/// no two ends meet at a vertex on their own. Chains that join nothing come back unchanged, closed
/// ones untouched (and after the open ones).
pub fn join(chains: &[BevelSweptChain]) -> Vec<BevelSweptChain> {
    let mut joined: Vec<Vec<ChainEdge>> = Vec::new();
    let mut closed: Vec<BevelSweptChain> = Vec::new();

    // Whether each working chain is still exactly the chain it arrived as. Only a chain the join
    // itself put together may be closed by the join.
    let mut assembled: Vec<bool> = Vec::new();

    for chain in chains {
        if chain.is_closed || chain.edges.is_empty() {
            closed.push(chain.clone());
            continue;
        }

        joined.push(chain.edges.clone());
        assembled.push(false);
    }

    let mut working = true;

    while working {
        working = false;

        // Which chain ends stand at each vertex. A vertex with exactly two of them is a joint; with
        // one it is a run's own end; with three or more it is the corner patch's junction. Walked
        // in the order the ends were met, since the first joint found is
        // the one joined.
        let mut ends_at: IndexMap<i32, Vec<(usize, bool)>> = IndexMap::new();

        for (chain, edges) in joined.iter().enumerate() {
            remember(&mut ends_at, edges[0].from_vertex(), (chain, true));
            remember(
                &mut ends_at,
                edges[edges.len() - 1].to_vertex(),
                (chain, false),
            );
        }

        for ends in ends_at.values() {
            if ends.len() != 2 || ends[0].0 == ends[1].0 {
                continue;
            }

            let first = ends[0];
            let second = ends[1];

            // The first chain has to arrive at the joint and the second to leave it, so each is
            // turned round if it is facing the other way. A chain reversed is its edges in the
            // other order, each walked the other way.
            let mut ahead = if first.1 {
                turned(&joined[first.0])
            } else {
                joined[first.0].clone()
            };
            let behind = if second.1 {
                joined[second.0].clone()
            } else {
                turned(&joined[second.0])
            };

            ahead.extend(behind);

            joined[first.0] = ahead;
            assembled[first.0] = true;

            joined.remove(second.0);
            assembled.remove(second.0);

            working = true;
            break;
        }
    }

    let mut answer = Vec::with_capacity(joined.len() + closed.len());

    // How many chain ends are left standing at each vertex, now that no two of them meet on their
    // own any more. A ring's own closing vertex carries its two and nobody else's; three there is a
    // junction, and a run that closed itself over one would take its third arm's ends away from
    // the reading that wants them (SurfaceChainOpensAtTheJunction opens rings for the same reason).
    let mut ends_left: HashMap<i32, i32> = HashMap::new();

    for edges in &joined {
        count(&mut ends_left, edges[0].from_vertex());
        count(&mut ends_left, edges[edges.len() - 1].to_vertex());
    }

    for (chain, edges) in joined.into_iter().enumerate() {
        // The closing joint gets the side test every other joint of a run gets
        // (bevel_chain_linker::closes_on_itself applies the same one): a run that meets itself with
        // its two surfaces swapped is one the sweep cannot carry a frame round, and it is published
        // open - which is what the linker hands back for those same edges picked as ONE group.
        let last = &edges[edges.len() - 1];
        let closes = assembled[chain]
            && edges[0].from_vertex() == last.to_vertex()
            && ends_left[&edges[0].from_vertex()] == 2
            && bevel_chain_linker::sides_agree(last, &edges[0]);

        answer.push(BevelSweptChain::new(edges, closes));
    }

    answer.extend(closed);

    answer
}

/// Counts one more chain end at `vertex`.
fn count(ends_left: &mut HashMap<i32, i32>, vertex: i32) {
    *ends_left.entry(vertex).or_insert(0) += 1;
}

/// Remembers which chain end stands at `vertex`.
fn remember(ends_at: &mut IndexMap<i32, Vec<(usize, bool)>>, vertex: i32, end: (usize, bool)) {
    ends_at
        .entry(vertex)
        .or_insert_with(|| Vec::with_capacity(2))
        .push(end);
}

/// The chain walked the other way.
fn turned(chain: &[ChainEdge]) -> Vec<ChainEdge> {
    chain.iter().rev().map(ChainEdge::reversed).collect()
}
