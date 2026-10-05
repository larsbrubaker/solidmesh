//! Which side of the level line a mesh vertex stands on ([`VertexSide`]). Ports the enum of
//! `Bevel/LevelLine/VertexSide.cs`; the per-vertex reading cache beside it
//! (`VertexFieldReading`) holds a `LevelLineMesh` and lands once that type does.

/// C# `VertexSide`: which side of the level line a mesh vertex stands on, decided once and read
/// by every triangle round it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum VertexSide {
    /// Field value below the snap band: in the material the cut deletes.
    #[default]
    Inside = 0,

    /// Within the snap band of the line: the line passes through this existing vertex.
    On,

    /// Field value above the snap band: in the material the cut keeps.
    Outside,
}
