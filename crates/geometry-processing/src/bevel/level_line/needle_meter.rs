//! A measurement only: what the per-split needle repair does on each call. Ports the nested enum of
//! `Bevel/LevelLine/NeedleMeter.cs` (a C# static class): what made a triangle
//! ([`NeedleMeterMadeBy`]). The meter's state and hooks land with later packages.

/// C# `NeedleMeter.MadeBy` (a `byte` enum): what put a triangle at an index of the mesh, as the
/// meter last saw it.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum NeedleMeterMadeBy {
    /// An input triangle no split or repair has rewritten.
    #[default]
    Input = 0,

    /// Added or rewritten by anything other than the needle repair - a refinement split.
    Split,

    /// One of the two triangles a needle repair left behind.
    Repair,
}
