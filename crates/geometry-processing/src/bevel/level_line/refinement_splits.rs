//! The mesh edits the refinement makes and the bookkeeping that goes with them. Ports the enum of
//! `Bevel/LevelLine/RefinementSplits.cs` (where a point stands in a triangle, [`PointPlace`]); the
//! C# static class `RefinementSplits` lands with later packages.

/// C# `PointPlace`: where a point stands in a triangle - the three answers, and they are not
/// overlapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PointPlace {
    /// Off every edge: the triangle itself is divided about it.
    #[default]
    Interior = 0,

    /// On one edge, away from both its ends: that edge is split, and its neighbour with it.
    OnAnEdge,

    /// At a corner the mesh already carries: there is nothing to insert.
    AtACorner,
}
