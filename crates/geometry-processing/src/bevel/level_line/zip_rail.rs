//! A loop re-read as a cycle running in increasing chain parameter ([`ZipRail`]). Ports the type of
//! `Bevel/LevelLine/ZipRail.cs`; how the merge reads it (canonically at tie vertices, across jumps,
//! from its start and past its wrap) lands with later packages.

/// C# `ZipRail`: a loop re-read as a cycle running in increasing chain parameter - one of the two
/// rails `GapFill`'s per-chain zip merges. Its init-only properties are public fields here.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ZipRail {
    /// The rail's mesh vertices, in increasing parameter.
    pub vertices: Vec<i32>,
    /// The chain parameter each vertex reads.
    pub parameters: Vec<f64>,
    /// True when the rail runs against its loop's walk - so its fill edges run WITH the rail.
    pub reversed: bool,
}
