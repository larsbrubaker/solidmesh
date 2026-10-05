//! How far a run may carry on PAST the end of its crease, and what stops it there
//! (`Bevel/BevelFaceRunOut.cs`): what stopped a run's tangent line ([`BevelRunOutBound`]), which
//! decides how the bead ends there. The walk itself (C# `BevelFaceRunOut`) lands with the core's
//! packages.

/// C# `BevelRunOutBound`: what stopped a run's tangent line, which is what decides how the bead
/// ends there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BevelRunOutBound {
    /// The mesh ran out - an open rim, or a non-manifold junction. Nothing to run out onto.
    #[default]
    OpenBoundary = 0,
    /// The face ENDS: past this edge the surface falls away, so the bead's fill tapers to nothing
    /// on the edge itself. The cube's front corner edge on "5mm Bevel Around Center.mcx", where
    /// both of the left beads' tangent lines converge onto the 45 degree knife tip.
    ConvexEdge,
    /// A WALL rises: the bead runs out flat against it, which is what the control does where the
    /// four beads eat the notch's unselected centre crease.
    ConcaveEdge,
}
