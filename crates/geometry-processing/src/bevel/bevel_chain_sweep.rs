//! One continuous tool swept along a whole chain of edges, mitred at every joint - the plan's 5.1
//! per-chain sweep (`Bevel/BevelChainSweep.cs`, the C# `internal static partial class
//! BevelChainSweep`): which section a sweep carries ([`BevelSweptSection`]) and its two rulers. The
//! sweep lands with later packages as free functions here and in its sibling files
//! ([`super::bevel_chain_sweep_mesh`], [`super::bevel_chain_sweep_rings`]).

/// C# `BevelSweptSection`: which section a sweep carries along the chain. All of them are swept
/// by exactly the same code - the same per-segment frames, mitre planes and end caps - which is
/// the point of asking for the wedge and the ball here rather than building a second solid some
/// other way.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BevelSweptSection {
    /// The bevel tool itself - the cutter on a convex run, the bead on a concave one.
    #[default]
    Tool = 0,
    /// The wedge the profile is drawn in rather than the profile
    /// (`BevelEdgeToolBuilder.TryBuildVoidWedgeCrossSection`), which on a concave run is the void
    /// between the two selected faces.
    VoidWedge,
    /// The same wedge with its two lateral walls left ON the crease's faces rather than buried a
    /// hair inside them - how the SQUARED sweep alone is drawn, because its wedge is the only one
    /// the clip unions with a 5.3 corner's void, whose own walls lie on those same faces.
    VoidWedgeOnTheFaces,
    /// The rolling BALL itself rather than the material it clears
    /// (`BevelEdgeToolBuilder.TryBuildBallCrossSection`) - swept, the tube whose surface is the
    /// fillet. `BevelPairCornerTrim` wants it, to take one chain's fillet out of a neighbouring
    /// chain's overshooting cutter.
    Ball,
}

/// The least cosine a joint's mitre may stand at against either segment before the joint is too
/// sharp to mitre.
pub const MINIMUM_MITER_COSINE: f64 = 0.1;

/// The fewest segments a closed run may have.
pub const MINIMUM_CLOSED_SEGMENTS: i32 = 3;
