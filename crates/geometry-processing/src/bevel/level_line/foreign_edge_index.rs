//! A box tree over one set of foreign edges ([`ForeignEdgeIndex`], with its node
//! [`ForeignEdgeIndexNode`]) and what a nearest-first search over it comes back with
//! ([`ForeignApproach`]). Ports `Bevel/LevelLine/ForeignEdgeIndex.cs`: the types, their
//! constructors, the edge count and list, and the tree's object-median build step
//! ([`ForeignEdgeIndex::add`]); building over a mesh and searching land with the room-reading
//! method package.

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

impl ForeignApproach {
    /// C#'s constructor.
    pub fn new(least: f64, near: Vec<((i32, i32), f64)>, read: i32) -> Self {
        ForeignApproach { least, near, read }
    }
}

impl ForeignEdgeIndexNode {
    /// C#'s constructor.
    pub fn new(start: i32, count: i32, left: i32, right: i32, min: Vector3, max: Vector3) -> Self {
        ForeignEdgeIndexNode {
            start,
            count,
            left,
            right,
            min,
            max,
        }
    }
}

/// C# `Double.CompareTo`: NaN orders below every number and equal to itself.
fn compare_f64(a: f64, b: f64) -> std::cmp::Ordering {
    a.partial_cmp(&b)
        .unwrap_or_else(|| a.is_nan().cmp(&b.is_nan()).reverse())
}

impl ForeignEdgeIndex {
    /// C# `ForeignEdgeIndex.LeafEdges`: the most edges a leaf holds.
    pub const LEAF_EDGES: i32 = 4;

    /// C#'s private constructor.
    pub fn new(
        edges: Vec<(i32, i32)>,
        ends: Vec<Vector3>,
        nodes: Vec<ForeignEdgeIndexNode>,
    ) -> Self {
        ForeignEdgeIndex { edges, ends, nodes }
    }

    /// C# `ForeignEdgeIndex.Count`: how many edges the index holds.
    pub fn count(&self) -> i32 {
        self.edges.len() as i32
    }

    /// C# `ForeignEdgeIndex.Edges`: the edges, in the tree's order.
    pub fn edges(&self) -> &[(i32, i32)] {
        &self.edges
    }

    /// C# `ForeignEdgeIndex.Add`: builds the node over `order[start .. start + count]` (indices
    /// into the edges, whose ends are `ends[2 i]` and `ends[2 i + 1]`) into `built`, and returns
    /// its index. Splits the run at the median of the longest axis of its box - the ordinary
    /// object-median build, so a node's box shrinks with its depth even where the edges are
    /// unevenly spread.
    pub fn add(
        built: &mut Vec<ForeignEdgeIndexNode>,
        ends: &[Vector3],
        order: &mut [i32],
        start: i32,
        count: i32,
    ) -> i32 {
        let mut min = Vector3::POSITIVE_INFINITY;
        let mut max = Vector3::NEGATIVE_INFINITY;
        for i in start..start + count {
            let a = ends[(order[i as usize] * 2) as usize];
            let b = ends[((order[i as usize] * 2) + 1) as usize];
            min = Vector3::component_min(min, Vector3::component_min(a, b));
            max = Vector3::component_max(max, Vector3::component_max(a, b));
        }

        let index = built.len();
        built.push(ForeignEdgeIndexNode::default());

        let mut left = -1;
        let mut right = -1;
        if count > Self::LEAF_EDGES {
            let span = max - min;
            let axis = if span.x >= span.y && span.x >= span.z {
                0
            } else if span.y >= span.z {
                1
            } else {
                2
            };
            let centre = |i: i32| {
                let a = ends[(i * 2) as usize];
                let b = ends[((i * 2) + 1) as usize];
                match axis {
                    0 => a.x + b.x,
                    1 => a.y + b.y,
                    _ => a.z + b.z,
                }
            };

            // The index is the tie-break, so the split is the same one every time this set is
            // built (and the order is total, so any sort gives C#'s Array.Sort's answer).
            order[start as usize..(start + count) as usize]
                .sort_by(|&i, &j| compare_f64(centre(i), centre(j)).then(i.cmp(&j)));

            let half = count / 2;
            left = Self::add(built, ends, order, start, half);
            right = Self::add(built, ends, order, start + half, count - half);
        }

        built[index] = ForeignEdgeIndexNode::new(start, count, left, right, min, max);
        index as i32
    }
}
