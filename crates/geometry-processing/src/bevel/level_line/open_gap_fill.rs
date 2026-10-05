//! The gap fill for an OPEN run. Ports the nested type of `Bevel/LevelLine/OpenGapFill.cs` (the
//! primary file of the C# static partial class `OpenGapFill`): one open run read as a rail
//! ([`OpenGapFillOpenRail`]), the class's constants and the straight-row test ([`is_straight_row`]). The fill's functions land with later
//! packages.

/// How close two extended parameters must be, relative to the chain's length, to count as one.
pub const PARAMETER_TIE_RELATIVE: f64 = 1e-9;

/// The fraction of a past-end row's reach to its control point below which the arcs lay its spoke
/// straight. A row that short spans under a quarter radian of the nose round the end vertex, where
/// the fade is all but gone (c563's folding rows: 0.006 and 0.0008 against 0.026).
pub const SHORT_PAST_END_SPOKE: f64 = 0.25;

/// A row the arcs laid as a straight edge onto a meeting point, or the degenerate spoke AT one:
/// one vertex repeated from the second (or to the second last) on.
pub fn is_straight_row(arc: &[i32]) -> bool {
    arc.len() > 2 && (arc[1] == arc[arc.len() - 1] || arc[arc.len() - 2] == arc[0])
}

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
