//! What is done to a RING of pool vertices once the clipping is over: tidying a ring the clip left
//! with a point repeated, and knitting the pieces one facet was cut into so that none of them is
//! bounded part way along an edge another one carries whole. Ports
//! `Bevel/Surface/SurfaceRing.cs` (C# `internal static class`; a module of free functions here).
//!
//! Kept apart from `SurfaceFacetCut` because neither reading needs a cut at all - they are about
//! the rings a cut leaves behind, and every caller that cuts a facet by hand (the end cut's wedge
//! sides, the mixed corner's bites) wants them without having a `SurfaceFacetCut` in hand.
//!
//! The directed-edge set and the onward map are only looked up and counted, never iterated into
//! an output: the boundary is started from the rings' own order, so a `HashSet`/`HashMap` cannot
//! leak its order here (C# says the same of its dictionary).

use std::collections::{HashMap, HashSet};

use super::SurfaceVertexPool;

/// The boundary of the UNION of several rings, as one ring: the edges used by exactly one of
/// them, linked end to end in the rings' own direction. `None` (C# false) where that boundary is
/// not a single simple loop - two rings winding against each other, a region shaped like a figure
/// of eight, or one pinched to a point - because there is then no one outline for the caller to
/// stand a face on.
///
/// What a face of the part cut into triangles looks like from outside: the triangulation's own
/// diagonals are each used twice, once each way, and cancel, leaving the face's outline. The rings
/// have to be wound the same way round for that to be true, which every facet of one mesh face is.
pub fn try_boundary_of_the_union<R: AsRef<[i32]>>(rings: &[R]) -> Option<Vec<i32>> {
    let mut used: HashSet<(i32, i32)> = HashSet::new();
    // The same edges in insertion order, so the onward map is filled in the order C#'s
    // HashSet enumerates (insertion order where nothing was removed).
    let mut used_in_order: Vec<(i32, i32)> = Vec::new();

    for ring in rings {
        let ring = ring.as_ref();
        for i in 0..ring.len() {
            let edge = (ring[i], ring[(i + 1) % ring.len()]);
            // The same directed edge twice is two facets of the region facing OPPOSITE ways,
            // which is a fold rather than a face - there is nothing to take the boundary of.
            if !used.insert(edge) {
                return None;
            }
            used_in_order.push(edge);
        }
    }

    let mut onward: HashMap<i32, i32> = HashMap::new();

    for &(from, to) in &used_in_order {
        if used.contains(&(to, from)) {
            // Used both ways: interior to the region, and the two facets sharing it are what
            // makes it so.
            continue;
        }

        if onward.contains_key(&from) {
            // Two boundary edges leaving one point: the outline forks there, so it is not one
            // loop.
            return None;
        }

        onward.insert(from, to);
    }

    if onward.is_empty() {
        return None;
    }

    // Started from the first corner of the first RING that is on the boundary at all, and never
    // from whichever key the map happens to hand back first. The outline goes on to be fanned,
    // and an apex - or the order the ears come off in - that depends on hash order is a
    // triangulation that can change between runs of the same build for no reason anybody can see.
    let start = rings
        .iter()
        .flat_map(|ring| ring.as_ref().iter().copied())
        .find(|vertex| onward.contains_key(vertex))?;

    let mut boundary = Vec::with_capacity(onward.len());
    boundary.push(start);

    let mut at = onward[&start];

    while at != start {
        let next = *onward.get(&at)?;
        if boundary.len() > onward.len() {
            return None;
        }

        boundary.push(at);
        at = next;
    }

    if boundary.len() != onward.len() {
        // Closed early: what is left over is a second loop, so the region has a hole in it or is
        // two pieces touching at a point.
        return None;
    }

    Some(boundary)
}

/// The same pieces with every vertex that stands part way along another piece's edge inserted
/// into it. Splitting a facet at an end plane and then cutting one of the halves leaves exactly
/// that: the tangent line meets the end plane INSIDE the facet, and the piece on the far side of
/// the end knows nothing of the point where it did. Left alone that is a T junction in the middle
/// of a face.
///
/// The vertices found along one edge are ordered by where they fall along it. C# sorts them with
/// the unstable `List.Sort`; two distinct pool vertices at the same parameter would have to sit
/// within a weld of each other, which the pool welds into one, so ties do not arise - the stable
/// sort here keeps first-standing order should they ever.
// C#'s `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn knit(pieces: Vec<Vec<i32>>, pool: &SurfaceVertexPool) -> Vec<Vec<i32>> {
    if pieces.len() < 2 {
        return pieces;
    }

    let mut standing: Vec<i32> = Vec::new();

    for piece in &pieces {
        for &vertex in piece {
            if !standing.contains(&vertex) {
                standing.push(vertex);
            }
        }
    }

    let mut knitted = Vec::with_capacity(pieces.len());

    for piece in &pieces {
        let mut ring = Vec::with_capacity(piece.len());

        for i in 0..piece.len() {
            let from = pool.position_of(piece[i]);
            let run = pool.position_of(piece[(i + 1) % piece.len()]) - from;

            ring.push(piece[i]);

            if !(run.length_squared() > 0.0) {
                continue;
            }

            let mut along: Vec<(f64, i32)> = Vec::new();

            for &vertex in &standing {
                if piece.contains(&vertex) {
                    continue;
                }

                let at = pool.position_of(vertex) - from;
                let where_ = at.dot(run) / run.length_squared();

                if where_ > 0.0
                    && where_ < 1.0
                    && (at - (run * where_)).length() <= pool.weld_distance()
                {
                    along.push((where_, vertex));
                }
            }

            along.sort_by(|first, second| first.0.total_cmp(&second.0));

            ring.extend(along.iter().map(|on| on.1));
        }

        knitted.push(without_repeats(&ring));
    }

    knitted
}

/// The same ring of vertices with any point repeated straight after itself dropped - what a clip
/// leaves when it crosses within a weld of a vertex that was already there.
pub fn without_repeats(ring: &[i32]) -> Vec<i32> {
    let mut tidied = Vec::with_capacity(ring.len());

    for i in 0..ring.len() {
        if ring[i] != ring[(i + ring.len() - 1) % ring.len()] {
            tidied.push(ring[i]);
        }
    }

    tidied
}
