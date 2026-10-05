//! The corner fill's loop reading. Ports the nested type of `Bevel/LevelLine/CornerFill.cs` (a C#
//! static partial class): one stretch of a group loop read by one chain ([`CornerFillPiece`]). The
//! fill's functions land with later packages.

/// C# `CornerFill.Piece`: one stretch of a group loop read by one chain - its vertices in the
/// loop's walk order, each with the chain parameter it reads.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CornerFillPiece {
    /// The chain that owns this stretch.
    pub owner: i32,
    /// The stretch's vertices, in the loop's walk order.
    pub vertices: Vec<i32>,
    /// The owner's chain parameter at each vertex.
    pub parameters: Vec<f64>,
}
