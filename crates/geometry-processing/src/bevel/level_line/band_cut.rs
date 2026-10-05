//! The band cut's records (`Bevel/LevelLine/BandCut.cs`): one run of the level line through the
//! cut mesh ([`LevelLineChain`]) and what the cut leaves ([`BandCutResult`]). The cut itself (C#'s
//! static `BandCut`) lands with its method package as functions of this module.

/// C# `LevelLineChain`: one run of the level line through the cut mesh, as mesh vertices in order.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LevelLineChain {
    /// The vertices of the run in order; a closed run does not repeat its first vertex at the end.
    pub vertices: Vec<i32>,
    /// True when the run comes back to where it started.
    pub closed: bool,
}

/// C# `BandCutResult`: what the cut leaves - the two halves of the mesh, and the line between
/// them.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BandCutResult {
    /// The triangles of the band that is to be deleted.
    pub inside_triangles: Vec<i32>,
    /// The triangles that stay.
    pub outside_triangles: Vec<i32>,
    /// The level line, chained into closed loops and open polylines. Empty when nothing is
    /// crossed.
    pub lines: Vec<LevelLineChain>,
}
