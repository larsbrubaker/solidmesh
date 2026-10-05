//! Building a run's sections chain vertex by chain vertex (`Bevel/Sections/SectionBuilder.cs`, the
//! un-dotted file of the C# `internal static partial class SectionBuilder`; a module of free
//! functions here, with the walked-side partial in [`super::section_builder_walked_side`]). What is
//! here are the readings the build starts from: the operation's own feature test
//! ([`the_feature_test`]), the feature edge between two chain vertices
//! ([`try_feature_edge_between`]) and the vertex-pair index of the mesh's edges
//! ([`edges_by_vertex_pair`]); the build itself (`TryBuild`) lands with a later package.

use std::collections::HashSet;
use std::rc::Rc;

use polygon_mesh::MeshEdgeGraph;

use super::section_fan_walk::{self, EdgeAtPair};
use super::IsAFeatureEdge;
use crate::topology::{ChainEdge, FeatureEdgeSet};

/// C# `TheFeatureTest`: the operation's OWN reading of which mesh edges are features - never a
/// coplanarity test, which on a tessellated wall calls every fold a feature and stops a surface walk
/// at the first one (see `SectionSurfaceWalk`). The set is only looked up, so its order cannot
/// reach an output.
pub fn the_feature_test(features: &FeatureEdgeSet) -> IsAFeatureEdge {
    let mesh_edges: HashSet<i32> = features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();

    Rc::new(move |edge| mesh_edges.contains(&edge))
}

/// C# `TryFeatureEdgeBetween`: the feature edge joining two mesh vertices, oriented to walk from
/// `from` to `to`.
pub fn try_feature_edge_between(
    features: &FeatureEdgeSet,
    from: i32,
    to: i32,
) -> Option<ChainEdge> {
    for &index in features.edges_at_vertex(from) {
        let edge = features.edges()[index as usize];

        if edge.start_vertex == from && edge.end_vertex == to {
            return Some(ChainEdge::new(index, edge, false));
        }

        if edge.end_vertex == from && edge.start_vertex == to {
            return Some(ChainEdge::new(index, edge, true));
        }
    }

    None
}

/// C# `EdgesByVertexPair`: every mesh edge, looked up by the pair of vertices it joins.
pub fn edges_by_vertex_pair(graph: &MeshEdgeGraph) -> EdgeAtPair {
    section_fan_walk::edges_by_vertex_pair(graph)
}
