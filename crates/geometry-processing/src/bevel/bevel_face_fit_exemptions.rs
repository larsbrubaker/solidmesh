//! The face-width check's exemptions: the stops a walk can hit that are NOT a group running out of
//! face.

use super::BevelFaceFit;
use crate::topology::{EdgeConvexity, FeatureEdge};

/// A face narrower than this fraction of the setback is a sliver.
pub const NEGLIGIBLE_SLIVER_FRACTION: f64 = 0.02;

impl BevelFaceFit {
    /// A stop at an unselected convex edge with only a sliver of face before it.
    pub fn a_negligible_sliver_past_a_convex_edge(
        &self,
        blocking_edge: i32,
        width: f64,
        setback: f64,
    ) -> bool {
        blocking_edge >= 0
            && width < setback * NEGLIGIBLE_SLIVER_FRACTION
            && !self.concave_surface_edges.contains(&blocking_edge)
            && !self.setbacks.contains_key(&blocking_edge)
    }

    /// A concave edge's collision with another concave stop.
    pub fn two_beads_sharing_the_face(
        &self,
        edge: &FeatureEdge,
        blocking_edge: i32,
        collides: bool,
    ) -> bool {
        collides
            && edge.convexity == EdgeConvexity::Concave
            && self.concave_surface_edges.contains(&blocking_edge)
    }

    /// A concave edge's bead running out past an unselected convex edge, with full width elsewhere
    /// on the face.
    pub fn a_bead_running_out_past_a_convex_edge(
        &self,
        edge: &FeatureEdge,
        blocking_edge: i32,
        widest_on_this_face: f64,
        setback: f64,
    ) -> bool {
        blocking_edge >= 0
            && edge.convexity == EdgeConvexity::Concave
            && widest_on_this_face >= setback
            && !self.concave_surface_edges.contains(&blocking_edge)
            && !self.setbacks.contains_key(&blocking_edge)
    }

    /// A stop at an edge sharing a vertex with `edge`, where the face has full width elsewhere and
    /// the stop is selected or convex - the corner's to resolve.
    pub fn the_corners_business(
        &self,
        edge: &FeatureEdge,
        blocking_edge: i32,
        widest_on_this_face: f64,
        setback: f64,
    ) -> bool {
        if blocking_edge < 0 {
            return false;
        }

        let from = self.graph.get_vertex0(blocking_edge);
        let to = self.graph.get_vertex1(blocking_edge);

        if from != edge.start_vertex
            && from != edge.end_vertex
            && to != edge.start_vertex
            && to != edge.end_vertex
        {
            return false;
        }

        if widest_on_this_face < setback {
            return false;
        }

        self.setbacks.contains_key(&blocking_edge)
            || !self.concave_surface_edges.contains(&blocking_edge)
    }
}
