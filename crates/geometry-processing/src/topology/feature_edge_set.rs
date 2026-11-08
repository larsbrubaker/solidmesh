//! The edges of a mesh that are edges of the shape. Built
//! from [`FeatureEdge::try_classify`]; walked by [`super::FeatureEdgeChain`] and
//! [`super::edge_picking`]. A set can hold its tangent chains walked once
//! ([`super::tangent_chain_cache`]), for the readings of a pick to read off.

use std::collections::HashMap;
use std::sync::OnceLock;

use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::feature_edge::{EdgeConvexity, FeatureEdge};
use super::tangent_chain_cache::TangentChainCache;

/// The edges of a mesh that are edges of the shape rather than of the triangle soup: manifold
/// edges where the surface actually turns. This is what a bevel or fillet can be applied to, and
/// what edge picking, chain propagation and face grouping all start from.
///
/// The set is a snapshot taken against one state of one mesh. It holds no reference to the mesh,
/// so nothing here goes stale-but-plausible after the mesh changes; rebuild it instead.
///
/// A set is safe to read from several threads at once: the only mutable things in here are the
/// vertex incidence cache and the tangent chain cache, each a `OnceLock` filled once.
#[derive(Debug)]
pub struct FeatureEdgeSet {
    edges: Vec<FeatureEdge>,
    flat_angle_degrees: f64,
    // Built on demand: walking from a vertex is what chain propagation needs, and the callers
    // that only want the edge list should not pay for it. Vertices with no feature edge are
    // simply absent. Only ever looked up, never iterated, so a HashMap's order cannot leak out.
    edges_by_vertex: OnceLock<HashMap<i32, Vec<i32>>>,
    // Filled only when asked for (`prepare_tangent_chains`), by a caller that will read many
    // chains of this set; a one-off reading walks just the chain it needs.
    tangent_chains: OnceLock<TangentChainCache>,
}

impl FeatureEdgeSet {
    /// Finds the feature edges of a mesh at the default flat tolerance
    /// ([`FeatureEdge::default_flat_angle_degrees`]).
    pub fn create(mesh: &Mesh) -> FeatureEdgeSet {
        Self::create_with_angle(mesh, FeatureEdge::default_flat_angle_degrees())
    }

    /// Finds the feature edges of a mesh: the manifold edges whose surface turn exceeds
    /// `flat_angle_degrees`. Boundary and non-manifold edges are never feature edges - there is
    /// no second face to bevel against.
    pub fn create_with_angle(mesh: &Mesh, flat_angle_degrees: f64) -> FeatureEdgeSet {
        Self::create_with_graph(mesh, &mesh.get_mesh_edge_graph(), flat_angle_degrees)
    }

    /// Finds the feature edges using an edge graph the caller already has. Building the graph is
    /// the expensive part of this scan, so anything that needs both (face grouping walks the same
    /// graph) should build it once and pass it here. That graph must be `mesh`'s own: nothing
    /// checks, and a graph from another mesh - or from this one before it changed - reads face
    /// and vertex indices that mean something else now, and answers with edges rather than an
    /// error.
    pub fn create_with_graph(
        mesh: &Mesh,
        graph: &MeshEdgeGraph,
        flat_angle_degrees: f64,
    ) -> FeatureEdgeSet {
        let mut found = Vec::new();

        for edge_index in 0..graph.edge_count() {
            if let Some(edge) =
                FeatureEdge::try_classify(mesh, graph, edge_index, flat_angle_degrees)
            {
                if edge.convexity != EdgeConvexity::Flat {
                    found.push(edge);
                }
            }
        }

        // What `index_of_mesh_edge`'s binary search stands on.
        debug_assert!(found
            .windows(2)
            .all(|pair| pair[0].mesh_edge_index < pair[1].mesh_edge_index));
        FeatureEdgeSet {
            edges: found,
            flat_angle_degrees,
            edges_by_vertex: OnceLock::new(),
            tangent_chains: OnceLock::new(),
        }
    }

    /// The feature edges, in mesh edge order.
    pub fn edges(&self) -> &[FeatureEdge] {
        &self.edges
    }

    /// How far the surface had to turn for an edge to be kept, in degrees.
    pub fn flat_angle_degrees(&self) -> f64 {
        self.flat_angle_degrees
    }

    /// The number of feature edges found.
    pub fn count(&self) -> usize {
        self.edges.len()
    }

    /// The index in [`Self::edges`] of the feature edge on mesh edge `mesh_edge_index`, or -1
    /// when that mesh edge is not a feature edge of this set (it is flat at this tolerance).
    ///
    /// A binary search: the set is filled by one pass over the mesh edges in index order, so its
    /// mesh edge indices are strictly ascending and each appears at most once.
    pub fn index_of_mesh_edge(&self, mesh_edge_index: i32) -> i32 {
        self.edges
            .binary_search_by_key(&mesh_edge_index, |edge| edge.mesh_edge_index)
            .map_or(-1, |i| i as i32)
    }

    /// Walks every tangent chain of the set at `tolerance_degrees` now, so that later readings
    /// of chains at that tolerance are lookups ([`Self::tangent_chains`]). Work for a background
    /// build: it costs one walk of every feature edge. Only the first call walks; a later call at
    /// another tolerance leaves the cache as it is.
    pub fn prepare_tangent_chains(&self, tolerance_degrees: f64) {
        self.tangent_chains
            .get_or_init(|| TangentChainCache::build(self, tolerance_degrees));
    }

    /// The tangent chains walked by [`Self::prepare_tangent_chains`], or `None` before it ran.
    pub fn tangent_chains(&self) -> Option<&TangentChainCache> {
        self.tangent_chains.get()
    }

    /// The chain a tangent walk from `seed_edge_index` at `tolerance_degrees` gives, and its
    /// sorted mesh edge indices, read off the prepared chains; `None` when they were not
    /// prepared, or were prepared at another tolerance.
    pub fn tangent_chain_through(
        &self,
        seed_edge_index: i32,
        tolerance_degrees: f64,
    ) -> Option<(super::FeatureEdgeChain, &[i32])> {
        self.tangent_chains()?
            .chain_through(self, seed_edge_index, tolerance_degrees)
    }

    /// The feature edges touching a mesh vertex, as indices into [`Self::edges`] (not mesh edge
    /// indices). Empty for a vertex no feature edge reaches. A vertex with three or more is a
    /// corner, where a chain has to stop.
    pub fn edges_at_vertex(&self, vertex_index: i32) -> &[i32] {
        self.edges_by_vertex
            .get_or_init(|| self.build_incidence())
            .get(&vertex_index)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    fn build_incidence(&self) -> HashMap<i32, Vec<i32>> {
        let mut incidence: HashMap<i32, Vec<i32>> = HashMap::new();

        for (i, edge) in self.edges.iter().enumerate() {
            incidence
                .entry(edge.start_vertex)
                .or_default()
                .push(i as i32);
            incidence.entry(edge.end_vertex).or_default().push(i as i32);
        }

        incidence
    }
}
