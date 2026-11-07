//! What one selection put at one mesh vertex: which feature edges end there and which rows of the
//! group list each of them was picked into.
//!
//! Read once, across every group at a time ([`BevelMixedCornerEnds::collect`]), because a vertex
//! does not care which group each edge came from - a cut and a bead meeting at one are usually two
//! rows of the panel. Split out of `BevelMixedBlends` when the corner grew a third question to
//! answer: not just what is here and whose it is, but whether two of these ends will be swept as
//! ONE run through the corner rather than two runs ending at it, which is what
//! [`BevelMixedCornerEnds::shares_a_group`] reads and what decides whether the corner can be
//! blended at all.

use std::collections::HashMap;

use vector_math::Vector3;

use crate::topology::{BevelEdgeGroupResolution, FeatureEdge};

/// One vertex where selected edges end, with the groups each edge there was picked into.
#[derive(Debug, Clone)]
pub struct BevelMixedCornerEnds {
    /// The mesh vertex this corner is, which is how a chain end recognizes it.
    pub vertex: i32,
    /// Where that vertex is.
    pub position: Vector3,
    edges: Vec<FeatureEdge>,
    groups: Vec<i32>,
    mesh_edges: Vec<i32>,
    groups_per_edge: Vec<Vec<i32>>,
}

impl BevelMixedCornerEnds {
    /// An empty corner at one vertex.
    pub fn new(vertex: i32, position: Vector3) -> Self {
        Self {
            vertex,
            position,
            edges: Vec::new(),
            groups: Vec::new(),
            mesh_edges: Vec::new(),
            groups_per_edge: Vec::new(),
        }
    }

    /// The selected edges ending here, one per mesh edge, in the order they were added.
    pub fn edges(&self) -> &[FeatureEdge] {
        &self.edges
    }

    /// Every group (row of the list) any of those edges was picked into, first seen first.
    pub fn groups(&self) -> &[i32] {
        &self.groups
    }

    /// Whether two of these ends were picked into one group, which is what makes them one RUN
    /// through this corner rather than two runs ending at it - see `BevelMixedBlends` for why a
    /// blend cannot be built over that.
    ///
    /// An edge that does not end here is in no group here, so it shares nothing (`false`) rather
    /// than reading a group list it does not have.
    pub fn shares_a_group(&self, first: &FeatureEdge, second: &FeatureEdge) -> bool {
        let Some(for_first) = self.groups_of(first.mesh_edge_index) else {
            return false;
        };
        let Some(for_second) = self.groups_of(second.mesh_edge_index) else {
            return false;
        };

        for group in for_second {
            if for_first.contains(group) {
                return true;
            }
        }

        false
    }

    fn groups_of(&self, mesh_edge: i32) -> Option<&Vec<i32>> {
        let at = self.mesh_edges.iter().position(|&e| e == mesh_edge)?;
        Some(&self.groups_per_edge[at])
    }

    /// Counts one selected end here, by MESH EDGE: an edge picked into two groups is one edge at
    /// this corner and asks for the same ball twice. Both of its groups are remembered even so -
    /// each is a row of the list that has to carry anything said about this corner, and which
    /// groups an end is in is what [`Self::shares_a_group`] reads.
    pub fn add(&mut self, edge: &FeatureEdge, group_index: i32) {
        if !self.groups.contains(&group_index) {
            self.groups.push(group_index);
        }

        match self
            .mesh_edges
            .iter()
            .position(|&e| e == edge.mesh_edge_index)
        {
            None => {
                self.edges.push(*edge);
                self.mesh_edges.push(edge.mesh_edge_index);
                self.groups_per_edge.push(vec![group_index]);
            }
            Some(at) => {
                if !self.groups_per_edge[at].contains(&group_index) {
                    self.groups_per_edge[at].push(group_index);
                }
            }
        }
    }

    /// Every vertex where selected edges end, first recorded first. An edge picked into two groups
    /// is counted once; both pickings ask for the same ball, because the operation has only one
    /// size.
    ///
    /// `eligible` is read at every resolution's index, so it must be at least as long.
    pub fn collect(
        resolutions: &[BevelEdgeGroupResolution],
        eligible: &[bool],
    ) -> Vec<BevelMixedCornerEnds> {
        let mut corners: Vec<BevelMixedCornerEnds> = Vec::new();
        let mut at_vertex: HashMap<i32, usize> = HashMap::new();

        for (group_index, resolution) in resolutions.iter().enumerate() {
            if !eligible[group_index] {
                continue;
            }

            for edge in &resolution.edges {
                let group_index = group_index as i32;
                Self::record(
                    &mut corners,
                    &mut at_vertex,
                    edge.start_vertex,
                    edge.start,
                    edge,
                    group_index,
                );
                Self::record(
                    &mut corners,
                    &mut at_vertex,
                    edge.end_vertex,
                    edge.end,
                    edge,
                    group_index,
                );
            }
        }

        corners
    }

    fn record(
        corners: &mut Vec<BevelMixedCornerEnds>,
        at_vertex: &mut HashMap<i32, usize>,
        vertex: i32,
        position: Vector3,
        edge: &FeatureEdge,
        group_index: i32,
    ) {
        let at = *at_vertex.entry(vertex).or_insert_with(|| {
            corners.push(BevelMixedCornerEnds::new(vertex, position));
            corners.len() - 1
        });

        corners[at].add(edge, group_index);
    }
}
