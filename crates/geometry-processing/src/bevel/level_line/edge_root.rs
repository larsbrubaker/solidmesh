//! Where the level line crosses one mesh edge ([`EdgeRoot`]): found once, shared by both
//! triangles on that edge. Ports the record of `Bevel/LevelLine/EdgeRoot.cs`; the bisection that
//! finds the roots (`EdgeRoots`) lands with the core package that reads the field along edges.

use vector_math::Vector3;

/// C# `EdgeRoot`: where the level line crosses one mesh edge, keyed by the edge's two vertex
/// indices in ascending order.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EdgeRoot {
    /// The lower vertex index of the edge.
    pub a: i32,
    /// The higher vertex index of the edge.
    pub b: i32,
    /// Where on the edge the line crosses.
    pub point: Vector3,
}
