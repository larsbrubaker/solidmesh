//! One edge, one tool. Where two groups of the same operation name the SAME mesh edge, the edge is
//! awarded to one of them and taken out of the other's selection before any chain is linked or any
//! sweep is run. Ports `Bevel/BevelSharedEdges.cs`.
//!
//! **Why it has to happen at all.** A user can easily name an edge twice without meaning to: pick
//! a crease as an `Edge`, then pick the face beside it as a `FaceGroup`, and the face hands back its
//! whole rim - that crease among it ("Fillet Should go to Sides and Stay Within Bounds.mcx").
//! Left alone, the two groups each sweep a bead along that crease, two solids sharing an entire
//! curved surface; the kernel folds them into a mesh it calls self-intersecting and the operation
//! is refused with a message about a boolean, for a selection whose only fault is a double pick.
//!
//! **Which group gets it: the FIRST one that names it.** The order the user built the selection
//! in. Awarding the edge to whichever group carries the longest run through it was tried and is
//! WORSE: it hands the face group a run turning 70 degrees at each end, whose void wedge folds
//! through itself at those bends. Straight runs meeting at corners are what the corner machinery
//! is built for.
//!
//! **What it deliberately does not do.** It does not merge groups. A group left holding NOTHING
//! is reported as such rather than silently doing nothing.

use std::collections::HashMap;

use crate::topology::BevelEdgeGroupResolution;

/// The same resolutions with every mesh edge named by more than one group left in only one of
/// them, and, per group, whether it resolved to edges and lost ALL of them to other groups (C#'s
/// `out bool[] gaveEveryEdgeAway`) - the case the caller has to say something about.
///
/// Takes the resolutions by value: where nothing is shared they come back untouched (C# returns
/// the same list instance).
pub fn award_once(
    resolutions: Vec<BevelEdgeGroupResolution>,
) -> (Vec<BevelEdgeGroupResolution>, Vec<bool>) {
    let mut gave_every_edge_away = vec![false; resolutions.len()];

    // Which groups name each mesh edge. Built first because the award needs to see all the
    // claimants at once, and because the common case - nobody sharing anything - is settled by
    // this pass alone. Only looked up, never iterated, so a HashMap's order cannot leak out.
    let mut claimants: HashMap<i32, Vec<usize>> = HashMap::new();
    let mut shared = false;

    for (i, resolution) in resolutions.iter().enumerate() {
        if !resolution.is_resolved() {
            continue;
        }

        for edge in &resolution.edges {
            let here = claimants.entry(edge.mesh_edge_index).or_default();

            if !here.contains(&i) {
                here.push(i);
                shared |= here.len() > 1;
            }
        }
    }

    if !shared {
        return (resolutions, gave_every_edge_away);
    }

    // `here` is in group order, because the scan above ran in it.
    let owner: HashMap<i32, usize> = claimants
        .iter()
        .map(|(mesh_edge, here)| (*mesh_edge, here[0]))
        .collect();

    let mut awarded = Vec::with_capacity(resolutions.len());

    for (i, resolution) in resolutions.into_iter().enumerate() {
        if !resolution.is_resolved() || resolution.edges.is_empty() {
            awarded.push(resolution);
            continue;
        }

        let kept: Vec<_> = resolution
            .edges
            .iter()
            .filter(|edge| owner[&edge.mesh_edge_index] == i)
            .copied()
            .collect();

        if kept.len() == resolution.edges.len() {
            awarded.push(resolution);
            continue;
        }

        gave_every_edge_away[i] = kept.is_empty();

        // The chain is dropped along with the edges it described, because it is the walk order of
        // a run that no longer exists in full and a caller reading it would sweep the edges this
        // group has just given up. A face group's own region is kept: it says which FACE was
        // picked, which is still true and is what the rim's convexity is read against.
        awarded.push(BevelEdgeGroupResolution::succeeded(
            resolution.group,
            resolution.features,
            resolution.seed_edge_index,
            kept,
            None,
            resolution.face_group,
        ));
    }

    (awarded, gave_every_edge_away)
}
