//! The surface construction's answer to an edge two groups both name: groups that share an edge
//! are linked as ONE selection rather than one of them losing the edge: [`link_together`] and the
//! union-find it is read with.
//!
//! **Why no group loses the edge.** In polygon space one mesh edge is one strip between its two
//! faces, and the same strip whichever group named it. Taking the edge out of one group would cost
//! it its CLOSURE - a face region's rim is a closed loop, and a loop with an edge missing is an open
//! run whose two ends would have to be cut somewhere in the middle of the other group's own strip,
//! which every multi-region case of "Every Chamfer Works.mcx" showed as "strip does not meet the
//! walkback" or as a shell left open along the shared arc.
//!
//! **What is linked with what.** Only the groups that actually share an edge are unioned, as the
//! connected components of "names an edge this one also names". A group sharing nothing keeps its
//! own resolution, and with it the walk order a `TangentChain` pick carries in its resolution's
//! chain, which a re-link from an unordered bag would throw away. Every group of a component is
//! handed to `BevelChainLinker` as one selection, which is what makes the two caps' rims and the
//! wall's rims one network: the arcs, the seams and the cube edges link into runs that stop only
//! where three selected edges meet, and those junctions are the corners the build already knows how
//! to make.
//!
//! Orientation costs nothing to reconcile. A rim edge is classified off the MESH
//! (`FeatureEdge::try_classify`) rather than against the region that named it, so two groups
//! naming one edge describe it identically and the first copy of it is the only one kept.

use std::collections::{HashMap, HashSet};

use crate::bevel::bevel_chain_linker::{link, link_resolution};
use crate::bevel::BevelSweptChain;
use crate::topology::{BevelEdgeGroupResolution, FeatureEdge};

/// The runs to plan for a whole pass - every group's edges, with the groups that name an edge in
/// common linked together. `resolutions` are the groups this pass is building, in the operation's
/// own order.
pub fn link_together(resolutions: &[BevelEdgeGroupResolution]) -> Vec<BevelSweptChain> {
    let mut chains = Vec::new();

    for together in components(resolutions) {
        if together.len() == 1 {
            // Nobody else names any of its edges, so it is linked exactly as it always was - stored
            // walk order and all.
            chains.extend(link_resolution(&resolutions[together[0]]));

            continue;
        }

        let mut seen = HashSet::new();
        let mut union: Vec<FeatureEdge> = Vec::new();

        for &index in &together {
            for edge in &resolutions[index].edges {
                if seen.insert(edge.mesh_edge_index) {
                    union.push(*edge);
                }
            }
        }

        chains.extend(link(&union));
    }

    chains
}

/// The groups gathered into the sets that have to be linked as one - two groups are in the same set
/// when they name a common mesh edge, or are joined through a third group that does.
fn components(resolutions: &[BevelEdgeGroupResolution]) -> Vec<Vec<usize>> {
    // Union-find over the groups, joined through the edges they name. Each group starts as its own
    // set and meets the first group that named each of its edges.
    let mut of: Vec<usize> = (0..resolutions.len()).collect();
    let mut named_by: HashMap<i32, usize> = HashMap::new();

    for (i, resolution) in resolutions.iter().enumerate() {
        for edge in &resolution.edges {
            if let Some(&first) = named_by.get(&edge.mesh_edge_index) {
                join(&mut of, first, i);
            } else {
                named_by.insert(edge.mesh_edge_index, i);
            }
        }
    }

    let mut components: Vec<Vec<usize>> = Vec::new();
    let mut slot: HashMap<usize, usize> = HashMap::new();

    for i in 0..resolutions.len() {
        let root = root_of(&mut of, i);

        let which = *slot.entry(root).or_insert_with(|| {
            components.push(Vec::new());
            components.len() - 1
        });

        components[which].push(i);
    }

    components
}

/// The root of `index`'s set, halving the path as it walks.
fn root_of(of: &mut [usize], mut index: usize) -> usize {
    while of[index] != index {
        of[index] = of[of[index]];
        index = of[index];
    }

    index
}

/// Joins the sets of `left` and `right`.
fn join(of: &mut [usize], left: usize, right: usize) {
    let left_root = root_of(of, left);
    let right_root = root_of(of, right);

    if left_root != right_root {
        // The EARLIER group's root wins, so a component is named by the first group in it and the
        // order the chains come out in is the operation's own.
        of[right_root] = left_root;
    }
}
