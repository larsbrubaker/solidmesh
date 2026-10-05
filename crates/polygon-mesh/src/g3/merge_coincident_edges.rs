//! geometry3Sharp `mesh_ops/MergeCoincidentEdges.cs`: closes cracks by welding pairs of
//! boundary edges whose ends coincide within `MergeDistance`. Edges are found through a
//! midpoint hash, grouped into equivalence sets, and merged fewest-candidates first through
//! [`DynamicPriorityQueue`]. Progress reporting is not ported: the repair's reporter has no
//! consumer in the combine.

use super::dmesh3::{DMesh3, G3Exception, MeshResult};
use super::dynamic_priority_queue::DynamicPriorityQueue;
use super::point_set_hashtable::{get_edge_point, BoundaryEdgeMidpointHashtable};

/// g3 `MathUtil.ZeroTolerancef`.
const ZERO_TOLERANCE_F: f64 = 1e-6f32 as f64;

/// g3 `MergeCoincidentEdges`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct MergeCoincidentEdges {
    pub(crate) merge_distance: f64,
    pub(crate) only_unique_pairs: bool,
}

impl Default for MergeCoincidentEdges {
    fn default() -> Self {
        MergeCoincidentEdges {
            merge_distance: ZERO_TOLERANCE_F,
            only_unique_pairs: false,
        }
    }
}

fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    dx * dx + dy * dy + dz * dz
}

impl MergeCoincidentEdges {
    /// C# `Apply`. `Err` is an exception C# throws out of it (a merge radius larger than a
    /// hash cell, an impossible lone hit, or a candidate with no set of its own), which the
    /// repair's `repair_cracks` catches.
    pub(crate) fn apply(&self, mesh: &mut DMesh3) -> Result<bool, G3Exception> {
        let merge_r2 = self.merge_distance * self.merge_distance;

        // construct hash table for edge midpoints
        let mut hash_n = 64;
        if mesh.triangle_count() > 100_000 {
            hash_n = 128;
        }
        if mesh.triangle_count() > 1_000_000 {
            hash_n = 256;
        }
        let hash = BoundaryEdgeMidpointHashtable::build(mesh, hash_n);

        // find "equivalence classes", ie sets of edges that are within merge distance
        let mut equiv_sets: Vec<Option<Vec<i32>>> = vec![None; mesh.max_edge_id() as usize];
        // C# `HashSet<int>`: additions in boundary-edge order, enumerated before any removal,
        // so a list plus a membership flag reproduces it.
        let mut remaining_order: Vec<i32> = Vec::new();
        let mut remaining = vec![false; mesh.max_edge_id() as usize];
        let boundary_edges: Vec<i32> = mesh.boundary_edge_indices().collect();
        for &eid in &boundary_edges {
            let midpt = get_edge_point(mesh, eid, 0.5);
            let buffer = hash.find_in_ball(mesh, midpt, self.merge_distance)?;
            let n = buffer.len();
            if n == 1 && buffer[0] != eid {
                return Err(G3Exception(
                    "MergeCoincidentEdges.Apply: how could this happen?!",
                ));
            }
            if n <= 1 {
                continue; // unique edge
            }

            let [ea, eb] = mesh.get_edge_v(eid);
            let (a, b) = (mesh.vertex(ea), mesh.vertex(eb));
            let mut equiv = Vec::with_capacity(n - 1);
            for &other in &buffer {
                if other != eid {
                    let [ec, ed] = mesh.get_edge_v(other);
                    let (c, d) = (mesh.vertex(ec), mesh.vertex(ed));
                    if is_same_edge(merge_r2, a, b, c, d) {
                        equiv.push(other);
                    }
                }
            }
            if !equiv.is_empty() {
                equiv_sets[eid as usize] = Some(equiv);
                remaining_order.push(eid);
                remaining[eid as usize] = true;
            }
        }

        // add potential duplicate edges to priority queue, sorted by number of possible matches.
        let mut q: DynamicPriorityQueue<i32> = DynamicPriorityQueue::new();
        for &i in &remaining_order {
            let set_len = equiv_sets[i as usize].as_ref().map_or(0, Vec::len);
            if self.only_unique_pairs {
                if set_len != 1 {
                    continue;
                }
                // C# checks each partner's set here but its `continue` only skips to the next
                // partner, so the check decides nothing; what it can do is throw, reading the
                // set of a partner that has none.
                if let Some(set) = &equiv_sets[i as usize] {
                    for &j in set {
                        if equiv_sets[j as usize].is_none() {
                            return Err(G3Exception(
                                "MergeCoincidentEdges.Apply: partner edge has no equivalence set",
                            ));
                        }
                    }
                }
            }
            q.enqueue(i, set_len as f32);
        }

        while let Some(eid) = q.dequeue() {
            if !mesh.is_edge(eid) || equiv_sets[eid as usize].is_none() || !remaining[eid as usize]
            {
                continue; // dealt with this edge already
            }
            if !mesh.is_boundary_edge(eid) {
                continue;
            }

            let mut equiv = equiv_sets[eid as usize].take().unwrap_or_default();

            // find viable match
            // [TODO] how to make good decisions here? prefer planarity?
            let mut merged = false;
            let mut i = 0;
            while i < equiv.len() && !merged {
                let other_eid = equiv[i];
                if !mesh.is_edge(other_eid) || !mesh.is_boundary_edge(other_eid) {
                    i += 1;
                    continue;
                }
                let (result, _) = mesh.merge_edges(eid, other_eid);
                if result != MeshResult::Ok {
                    equiv.remove(i);
                    match &mut equiv_sets[other_eid as usize] {
                        Some(other_set) => {
                            if let Some(pos) = other_set.iter().position(|&e| e == eid) {
                                other_set.remove(pos);
                            }
                        }
                        // C# calls `Remove` on the null set: a NullReferenceException
                        None => {
                            return Err(G3Exception(
                                "MergeCoincidentEdges.Apply: partner edge has no equivalence set",
                            ))
                        }
                    }
                } else {
                    merged = true;
                    equiv_sets[other_eid as usize] = None;
                    remaining[other_eid as usize] = false;
                    i += 1;
                }
            }

            // Whether or not it merged, this edge is done (C# nulls its set and drops it from
            // `remaining` on both branches).
            equiv_sets[eid as usize] = None;
            remaining[eid as usize] = false;
        }
        Ok(true)
    }
}

/// C# `is_same_edge`: the ends coincide pairwise, in either order.
fn is_same_edge(merge_r2: f64, a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> bool {
    (distance_squared(a, c) < merge_r2 && distance_squared(b, d) < merge_r2)
        || (distance_squared(a, d) < merge_r2 && distance_squared(b, c) < merge_r2)
}
