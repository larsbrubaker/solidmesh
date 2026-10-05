//! How many of each query a level-line build asked, by the refinement rule that asked it. Ports
//! the enum of `Bevel/LevelLine/FieldCounters.cs` ([`LevelLineRule`]); the per-thread counters of
//! the C# static class `FieldCounters` land with later packages.

/// C# `LevelLineRule`: the refinement rules a search can be charged to, by the names the rules
/// carry in `BandRefinement`'s own remarks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LevelLineRule {
    /// Nothing named this search - it was not asked from inside a rule.
    #[default]
    None = 0,

    /// (i) an edge with both ends outside whose least value dips below the line.
    EdgeDip,

    /// (i, mirrored) an edge with both ends inside whose greatest value rises above it.
    EdgeBump,

    /// (i, third face) an edge whose two ends read DIFFERENT sides, asked whether it crosses the
    /// line more than the once its ends admit to - the rule that makes "one root per crossed edge"
    /// true.
    MixedEdge,

    /// (ii) a triangle wholly outside hiding a closed loop in its middle.
    HiddenLoop,

    /// A triangle wholly inside hiding an island of kept surface.
    Island,

    /// A ridge of kept surface crossing a triangle whose corners are all outside.
    Ridge,

    /// (iii) the chord of an already-crossed triangle, held to the chord tolerance.
    ChordAccuracy,

    /// The sliver guard: a split some other rule asked for that would have thinned a child past
    /// `RefinementSplits.SliverAspectLimit`, taken instead as a longest-edge bisection. Last, so
    /// adding it moved no other rule's row.
    SliverGuard,
}
