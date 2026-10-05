//! A box tree over one set of foreign edges ([`ForeignEdgeIndex`], with its node
//! [`ForeignEdgeIndexNode`]) and what a nearest-first search over it comes back with
//! ([`ForeignApproach`]). Ports the types of `Bevel/LevelLine/ForeignEdgeIndex.cs`; building and
//! searching the tree land with the room-reading method package.

use vector_math::Vector3;

/// C# `ForeignApproach`: what a nearest-first search over foreign edges came back with - the least
/// closest approach to the chain, and every edge whose approach is within the factor the search
/// was given of it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ForeignApproach {
    /// The least closest approach; positive infinity when no edge was searched at all.
    pub least: f64,
    /// Every edge the search actually read, as ((low, high), approach) - a superset of the edges
    /// within the factor, which the caller narrows with the exact test it means (the search's own
    /// prune is deliberately generous, so no rounding of the factor can drop an edge the exact
    /// test would keep).
    pub near: Vec<((i32, i32), f64)>,
    /// How many edges the search read.
    pub read: i32,
}

/// C# `ForeignEdgeIndex`: a box tree over ONE set of foreign edges - the edges bounding one pair of
/// region sides - so a chain can find the nearest of them without reading them all.
///
/// Built once per room reading and searched once per chain. The set is the thing that makes this
/// safe to share: an edge is foreign to chain k when k seeds the region on ONE side of it and not
/// the other, which depends only on the two TAGS across the edge, so edges carrying the same
/// unordered tag pair are foreign to exactly the same chains (see `ForeignEdgeRegions`).
/// Membership is therefore still per chain, and still the same test it always was; what is shared
/// is the geometry.
///
/// Why a tree and not a plain pass: on a face whose chains all bound ONE big region every chain's
/// foreign set is the whole face's boundary, so the plain pass is chains times boundary edges - 25
/// chains times about 1,718 edges on Many Holes' rim, which was 92% of what that pick's room
/// reading cost.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ForeignEdgeIndex {
    /// The edges, as (low, high) vertex indices, in the tree's order.
    pub edges: Vec<(i32, i32)>,
    /// Each edge's two end positions, edge `i` at `2 * i` and `2 * i + 1`.
    pub ends: Vec<Vector3>,
    /// The tree, root first.
    pub nodes: Vec<ForeignEdgeIndexNode>,
}

/// C# `ForeignEdgeIndex.Node`: one node of the tree - a run of edges and their box.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ForeignEdgeIndexNode {
    /// The first edge of the run.
    pub start: i32,
    /// How many edges the run holds.
    pub count: i32,
    /// The left child's node index, or -1 when this node is a leaf.
    pub left: i32,
    /// The right child's node index.
    pub right: i32,
    /// The box's least corner.
    pub min: Vector3,
    /// The box's greatest corner.
    pub max: Vector3,
}
