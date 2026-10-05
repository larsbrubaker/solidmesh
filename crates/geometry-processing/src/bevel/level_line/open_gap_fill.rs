//! The gap fill for an OPEN run. Ports the nested type of `Bevel/LevelLine/OpenGapFill.cs` (the
//! primary file of the C# static partial class `OpenGapFill`): one open run read as a rail
//! ([`OpenGapFillOpenRail`]). The fill's functions and static state land with later packages.

/// C# `OpenGapFill.OpenRail`: one open run read as a rail running from the meeting point at end 0
/// to the one at end 1, every vertex carrying the extended parameter. Its init-only properties are
/// public fields here.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct OpenGapFillOpenRail {
    /// The rail's mesh vertices, end 0 to end 1.
    pub vertices: Vec<i32>,
    /// The EXTENDED parameter of each vertex.
    pub parameters: Vec<f64>,
}
