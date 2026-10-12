//! The real mesh edges around a selected open run that its round tapers out along
//! (`Bevel/BevelRunTaper.cs`): the walk off each free end of the run onto the unselected edges
//! that carry on from it, as far as each end asked to reach. `BevelGroupTools` builds one and
//! `BevelTaperedSweep` sweeps it.

use std::collections::HashSet;

use super::bevel_chain_linker::BevelSweptChain;
use crate::topology::{ChainEdge, FeatureEdge, FeatureEdgeSet};

/// C# `BevelRunTaper`: real mesh edges surrounding a selected run, with physical runout extents.
/// A C# `record`, so a plain value; C#'s `with` expressions are struct updates on a clone.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelRunTaper {
    /// The selected run with the runout neighbours walked onto in front of and behind it.
    pub edges: Vec<ChainEdge>,
    /// Where in [`BevelRunTaper::edges`] the selected run starts.
    pub selected_index: i32,
    /// How many of [`BevelRunTaper::edges`] from there were actually picked.
    pub selected_count: i32,
    /// How far the start runs out; zero where it does not.
    pub start_length: f64,
    /// How far the end runs out; zero where it does not.
    pub end_length: f64,
}

impl BevelRunTaper {
    /// Walks off both free ends of an OPEN run onto the unselected edges that carry on from them,
    /// as far as each end asked to reach.
    ///
    /// A run of any length is taken, not just a lone edge: a run's end is the same situation a lone
    /// edge's end is - selected material meeting unselected material at a vertex nothing else owns -
    /// and the answer to it is the same. Only a CLOSED run has no free end to run out of. Each end
    /// is walked and bounded on its own, so a run may taper at one end, at both, or at neither -
    /// including where the caller offers only one of them, by asking for a length of zero at the
    /// other. `surfaces` is the per-face surface label (`None` is C#'s null, which tapers nothing).
    pub fn create(
        chain: &BevelSweptChain,
        features: &FeatureEdgeSet,
        surfaces: Option<&[i32]>,
        selected: &[FeatureEdge],
        tolerance: f64,
        start_length: f64,
        end_length: f64,
    ) -> Option<BevelRunTaper> {
        let surfaces = surfaces?;
        if chain.is_closed || chain.edges.is_empty() {
            return None;
        }
        // A length of zero is an END THE CALLER IS NOT OFFERING, not a bad number: an end another
        // pick, a corner patch or a blend already stands at keeps whatever that thing gave it, and
        // its partner at the other end of the same run is still free to run out. Only a run with
        // neither end on offer has nothing to do here.
        let walk_start = start_length.is_finite() && start_length > 0.0;
        let walk_end = end_length.is_finite() && end_length > 0.0;
        if !walk_start && !walk_end {
            return None;
        }
        let selected_ids: HashSet<i32> = selected.iter().map(|e| e.mesh_edge_index).collect();
        let mut visited: HashSet<i32> =
            chain.edges.iter().map(|e| e.edge.mesh_edge_index).collect();

        let mut walk = |mut previous: ChainEdge, length: f64| -> Option<Vec<ChainEdge>> {
            let convexity = previous.edge.convexity;
            let mut result = Vec::new();
            let mut travelled = 0.0;
            while travelled < length {
                // Read the complete selection, including rows excluded from shared sweeps.
                let to_vertex = previous.to_vertex();
                if selected.iter().any(|e| {
                    e.mesh_edge_index != previous.edge.mesh_edge_index
                        && (e.start_vertex == to_vertex || e.end_vertex == to_vertex)
                }) {
                    return None;
                }
                let candidates: Vec<ChainEdge> = features
                    .edges_at_vertex(to_vertex)
                    .iter()
                    .map(|&i| {
                        let edge = features.edges()[i as usize];
                        ChainEdge::new(i, edge, edge.end_vertex == to_vertex)
                    })
                    .filter(|e| {
                        e.edge.mesh_edge_index != previous.edge.mesh_edge_index
                            && e.edge.convexity == convexity
                            && surfaces[e.left_face() as usize]
                                == surfaces[previous.left_face() as usize]
                            && surfaces[e.right_face() as usize]
                                == surfaces[previous.right_face() as usize]
                            && e.direction().dot(previous.direction())
                                >= (tolerance * std::f64::consts::PI / 180.0).cos()
                    })
                    .collect();
                if candidates.len() != 1
                    || selected_ids.contains(&candidates[0].edge.mesh_edge_index)
                    || !visited.insert(candidates[0].edge.mesh_edge_index)
                {
                    return None;
                }
                previous = candidates[0];
                result.push(previous);
                travelled += (previous.to() - previous.from()).length();
            }
            Some(result)
        };

        let start = if walk_start {
            walk(chain.edges[0].reversed(), start_length)
        } else {
            None
        };
        let end = if walk_end {
            walk(chain.edges[chain.edges.len() - 1], end_length)
        } else {
            None
        };
        if start.is_none() && end.is_none() {
            return None;
        }
        let mut edges = Vec::new();
        if let Some(start) = &start {
            edges.extend(start.iter().rev().map(|e| e.reversed()));
        }
        let selected_index = edges.len() as i32;
        edges.extend_from_slice(&chain.edges);
        if let Some(end) = &end {
            edges.extend_from_slice(end);
        }
        Some(BevelRunTaper {
            edges,
            selected_index,
            selected_count: chain.edges.len() as i32,
            start_length: if start.is_none() { 0.0 } else { start_length },
            end_length: if end.is_none() { 0.0 } else { end_length },
        })
    }
}
