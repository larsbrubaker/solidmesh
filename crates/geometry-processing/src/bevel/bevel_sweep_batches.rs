//! Joins adjacent selections for sweeping without changing their persisted groups or row identities
//! ([`create`], [`BevelSweepBatch`]). It reads the resolver's rows
//! (`topology::bevel_edge_group_resolver`) and links each batch's edges with
//! [`super::bevel_chain_linker`]; the edge builder sweeps each batch as one tool.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::sync::Arc;

use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::math_helper;

use super::bevel_chain_linker::{self, BevelSweptChain};
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge};

/// A temporary shared sweep and the original selection rows that contribute to it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelSweepBatch {
    /// The contributing rows' indices in the resolutions, ascending.
    pub group_indices: Vec<i32>,
    /// The chains the batch sweeps (always exactly one: a continuous, uniform run).
    pub chains: Vec<BevelSweptChain>,
}

/// The key rows batch by: the feature set by IDENTITY (ResolveAll caches one per exact flat
/// tolerance on one working mesh) and the curve tolerance by value, under which `0.0` and `-0.0`
/// are one key and NaN equals NaN.
fn compatibility_key(resolution: &BevelEdgeGroupResolution) -> (usize, u64) {
    let tolerance = resolution.group.curve_tolerance_degrees;
    let bits = if tolerance == 0.0 {
        0.0f64.to_bits()
    } else if tolerance.is_nan() {
        f64::NAN.to_bits()
    } else {
        tolerance.to_bits()
    };
    (Arc::as_ptr(&resolution.features) as usize, bits)
}

/// The batches the eligible, compatible, unclipped rows join into. The caller has already checked
/// each group's fit and identified any clipping that owns its cutters. `eligible`,
/// `trimmed_by_a_wall` and `shadow_columns` run in step with `resolutions`.
pub fn create(
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
    trimmed_by_a_wall: &[bool],
    shadow_columns: &[Option<Vec<Mesh>>],
) -> Vec<BevelSweepBatch> {
    // Corner patches and mixed handovers see the whole eligible selection, including rows
    // excluded from batching below. A different tolerance or a shadow-owned cutter cannot
    // make a third selected edge disappear from the corner that it still owns.
    let mut selected_at_vertex: HashMap<i32, HashSet<i32>> = HashMap::new();
    for (i, resolution) in resolutions.iter().enumerate() {
        if !eligible[i] || !resolution.is_resolved() {
            continue;
        }
        for edge in &resolution.edges {
            for vertex in [edge.start_vertex, edge.end_vertex] {
                selected_at_vertex
                    .entry(vertex)
                    .or_default()
                    .insert(edge.mesh_edge_index);
            }
        }
    }

    // Insertion ordered: the batches come out in this order.
    let mut compatible: IndexMap<(usize, u64), (f64, Vec<usize>)> = IndexMap::new();
    for (i, resolution) in resolutions.iter().enumerate() {
        // Cap withdrawals and shadow/body fallbacks are owned by a single row. Leave those
        // rows on the existing path, as well as face rims with their own partial-rim warnings.
        //
        // And beads, whichever way their run turns: a bead ADDS, so where two of them meet at a
        // vertex the answer is not one mitred sweep but plan 5.3's - both overshoot into the corner
        // and their union fills it - and each one's overshoot is then held to that corner's own void
        // (BevelBeadClip, BevelCornerVoids), which is what keeps a bead inside a thin wall. A merged
        // row has neither: it mitres the corner instead of rounding it, which underfills every
        // outside turn, and it carries one wedge that knows nothing about the corner the mitre
        // swallowed, so the clip falls back or is refused outright. Merging a cut costs none of that
        // - removal is idempotent, so the merged sweep and the separate ones remove the same set -
        // which is why the rule is about beads rather than about adjacency.
        if !eligible[i]
            || !resolution.is_resolved()
            || trimmed_by_a_wall[i]
            || shadow_columns[i].as_ref().is_some_and(|c| !c.is_empty())
            || resolution.face_group.is_some()
            || resolution
                .edges
                .iter()
                .any(|edge| edge.convexity == EdgeConvexity::Concave)
        {
            continue;
        }

        // ResolveAll caches one feature set per exact flat tolerance on one working mesh.
        // Requiring that very instance guarantees shared vertex IDs and identical end-cap
        // context; equal angle values on unrelated meshes would not be enough.
        compatible
            .entry(compatibility_key(resolution))
            .or_insert_with(|| (resolution.group.curve_tolerance_degrees, Vec::new()))
            .1
            .push(i);
    }

    let mut batches = Vec::new();
    for (curve_tolerance, indices) in compatible.values() {
        let curve_tolerance = *curve_tolerance;

        let mut at_vertex: HashMap<i32, Vec<usize>> = HashMap::new();
        for &index in indices {
            for edge in &resolutions[index].edges {
                for vertex in [edge.start_vertex, edge.end_vertex] {
                    at_vertex.entry(vertex).or_default().push(index);
                }
            }
        }

        let mut visited: HashSet<usize> = HashSet::new();
        for &seed in indices {
            if !visited.insert(seed) {
                continue;
            }
            let mut pending = VecDeque::new();
            let mut contributors: Vec<usize> = Vec::new();
            // Read back sorted by mesh edge index; a repeated edge keeps its last copy.
            let mut edges: BTreeMap<i32, FeatureEdge> = BTreeMap::new();
            pending.push_back(seed);
            while let Some(index) = pending.pop_front() {
                contributors.push(index);
                for edge in &resolutions[index].edges {
                    edges.insert(edge.mesh_edge_index, *edge);
                    for vertex in [edge.start_vertex, edge.end_vertex] {
                        for &neighbour in &at_vertex[&vertex] {
                            if visited.insert(neighbour) {
                                pending.push_back(neighbour);
                            }
                        }
                    }
                }
            }

            let branched_at =
                |vertex: i32| selected_at_vertex.get(&vertex).map_or(0, |s| s.len()) > 2;
            if contributors.len() > 1
                && !edges
                    .values()
                    .any(|edge| branched_at(edge.start_vertex) || branched_at(edge.end_vertex))
            {
                contributors.sort();
                let ordered: Vec<FeatureEdge> = edges.values().copied().collect();
                let chains = bevel_chain_linker::link(&ordered);
                // A branched component, or one crossing from a cut to a bead, has independent
                // failure/patch behavior. Only make a continuous, uniform run atomic here.
                if chains.len() == 1
                    && bevel_chain_linker::split_by_convexity(chains[0].clone()).len() == 1
                    && every_cross_row_joint_continues(
                        &chains[0],
                        resolutions,
                        &contributors,
                        curve_tolerance,
                    )
                {
                    batches.push(BevelSweepBatch {
                        group_indices: contributors.iter().map(|&i| i as i32).collect(),
                        chains,
                    });
                }
            }
        }
    }
    batches
}

/// Whether every joint in a merged chain where two DIFFERENT rows meet is a CONTINUATION of one run
/// rather than a corner between two of them.
///
/// A merged row is swept as one tool, so every joint inside it is resolved by the sweep's mitre.
/// That is the right answer where the run simply carries on - the facets of a tessellated rim,
/// which is what this batching was written for - and the wrong one at a CORNER, where the two runs
/// meet their faces at different angles: a rolling ball leaves the higher of the two runs'
/// cylinders there and the mitre's bisector plane cuts below it, so the corner comes out over-cut
/// (`BevelUnequalDihedralCornerTests`). Left unmerged, the two rows meet as plan 5.3's pair corner
/// and each cutter is trimmed by the other's rolling-ball tube (`BevelPairCornerTrim`), which is
/// the treatment that corner has.
///
/// The threshold is the rows' own curve tolerance, which is the same angle the picker uses to
/// decide whether a neighbouring edge is more of this curve or the start of something else
/// (`FeatureEdgeChain`) - so what a merged sweep may swallow is exactly what one pick would have
/// walked through anyway. Joints INSIDE one row are left alone: those edges were picked together
/// and their mitre is the sweep this row already had, merged or not.
fn every_cross_row_joint_continues(
    chain: &BevelSweptChain,
    resolutions: &[BevelEdgeGroupResolution],
    contributors: &[usize],
    curve_tolerance_degrees: f64,
) -> bool {
    let mut rows_of_edge: HashMap<i32, HashSet<usize>> = HashMap::new();
    for &index in contributors {
        for edge in &resolutions[index].edges {
            rows_of_edge
                .entry(edge.mesh_edge_index)
                .or_default()
                .insert(index);
        }
    }

    let agreement = math_helper::degrees_to_radians(curve_tolerance_degrees).cos();
    let count = chain.edges.len();
    let joints = if chain.is_closed { count } else { count - 1 };
    for i in 0..joints {
        let before = &chain.edges[i];
        let after = &chain.edges[(i + 1) % count];
        if let (Some(before_rows), Some(after_rows)) = (
            rows_of_edge.get(&before.edge.mesh_edge_index),
            rows_of_edge.get(&after.edge.mesh_edge_index),
        ) {
            if !before_rows.is_disjoint(after_rows) {
                continue;
            }
        }

        if before.direction().dot(after.direction()) < agreement {
            return false;
        }
    }

    true
}
