//! Which of the second pass's re-read creases are the SAME crease, and so must be beaded once
//! between them rather than once each. Ports `Bevel/BevelCreaseMerge.cs`.
//!
//! What identifies a crease is the MESH edges it runs over, on the cut part. Two groups whose
//! creases the round joined resolve onto the same run of mesh edges (each from its own end of
//! it), and a second bead along it would be a duplicate solid the union cannot fold. A
//! `ChainEdge.edge_index` looks like the same identity and is not: it indexes a `FeatureEdgeSet`,
//! and the second pass builds one set per distinct flat tolerance in the group list.
//!
//! And the merge is TRANSITIVE: a crease read from one group may share its first edges with a
//! second group's and its last with a third's, without the second and third sharing anything. So
//! the sets are unioned until nothing more joins, which is what union-find gives in one walk.

use std::collections::{BTreeMap, HashMap};

/// Groups the creases that share any mesh edge, transitively.
///
/// `mesh_edges_of_crease` holds one entry per crease, in the caller's own order, with the mesh
/// edges that crease runs over. A `None` (C# null) or empty entry is a crease the caller has
/// nothing to say about; it takes no part and is left out of the answer.
///
/// Answers one list per distinct crease, each holding the indices into `mesh_edges_of_crease`
/// that share it, in increasing order. The lists come back ordered by their first member, so the
/// answer does not depend on the order the merges happened to be found in.
pub fn merge(mesh_edges_of_crease: &[Option<Vec<i32>>]) -> Vec<Vec<usize>> {
    let mut parent: Vec<usize> = (0..mesh_edges_of_crease.len()).collect();

    fn find(parent: &mut [usize], mut of: usize) -> usize {
        while parent[of] != of {
            // Path halving: the walk is short here in any case (a handful of creases), and this
            // keeps it short however the merges came in.
            parent[of] = parent[parent[of]];
            of = parent[of];
        }

        of
    }

    // Lookup only, never iterated, so a HashMap cannot leak an order into the answer.
    let mut owner_of_edge: HashMap<i32, usize> = HashMap::new();

    for (i, crease) in mesh_edges_of_crease.iter().enumerate() {
        let Some(crease) = crease else {
            continue;
        };

        for &mesh_edge in crease {
            if let Some(&other) = owner_of_edge.get(&mesh_edge) {
                let a = find(&mut parent, i);
                let b = find(&mut parent, other);

                if a != b {
                    // The LOWER index wins, so a merged run is always named by the earliest
                    // crease in it - which is the one the caller reports the run against.
                    parent[a.max(b)] = a.min(b);
                }

                continue;
            }

            owner_of_edge.insert(mesh_edge, i);
        }
    }

    let mut members: BTreeMap<usize, Vec<usize>> = BTreeMap::new();

    for (i, crease) in mesh_edges_of_crease.iter().enumerate() {
        match crease {
            Some(edges) if !edges.is_empty() => {}
            _ => continue,
        }

        let root = find(&mut parent, i);
        members.entry(root).or_default().push(i);
    }

    members.into_values().collect()
}
