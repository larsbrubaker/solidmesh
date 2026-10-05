//! Laying an OFFSET side's reach along the surface rather than straight off the chain
//! (`Bevel/Sections/SectionBuilderWalkedSide.cs`, part of the C# `internal static partial class
//! SectionBuilder`): where one such reach ended ([`WalkedEnd`]). The walk lands with the core's
//! section packages as free functions in this module.

use vector_math::Vector3;

/// C# `SectionBuilder.WalkedEnd`: where one OFFSET side's reach ended when it was laid along the
/// surface - what `BevelSection`'s `left_walk_*` / `right_walk_*` fields are filled from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WalkedEnd {
    /// The pool index of the boundary point, or -1 where this side was not walked at all.
    pub boundary: i32,
    /// The face that point lies in, or -1 where this side was not walked at all.
    pub face: i32,
    /// The direction the walk was travelling when it ended.
    pub direction: Vector3,
    /// Whether the point was placed along the JOIN between two neighbours' boundary points rather
    /// than walked from its own station.
    pub is_join_placed: bool,
    /// Whether the reach crossed a FOLD getting here.
    pub crossed_a_fold: bool,
    /// The mesh edge the walk ENDED on, or -1 where it ended inside its face.
    pub edge: i32,
    /// Where along that edge, 0 at vertex 0 and 1 at 1.
    pub parameter: f64,
    /// The mesh VERTEX the walk ended ON, or -1 where it ended anywhere but a corner. The edge can
    /// name only one of the folds meeting at such a corner, so the vertex is carried beside it.
    pub vertex: i32,
}
