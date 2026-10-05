//! The open gap fill's cut sections. Ports the nested record of
//! `Bevel/LevelLine/OpenGapFill.CutSection.cs` (a partial of the C# static class `OpenGapFill`): a
//! run-out's flat region ([`OpenGapFillFlatRunOutRegion`]). The functions land with later packages.

use super::HoleFillTriangle;

/// C# `OpenGapFill.FlatRunOutRegion`: a run-out's flat region - its loop of mesh vertices, the
/// triangles closing it (loop indices), and the last cut row.
#[derive(Debug, Clone, PartialEq)]
pub struct OpenGapFillFlatRunOutRegion {
    /// The region's loop of mesh vertices.
    pub r#loop: Vec<i32>,
    /// The triangles closing the loop, as indices into it.
    pub triangles: Vec<HoleFillTriangle>,
    /// The last cut row.
    pub last_cut: i32,
}
