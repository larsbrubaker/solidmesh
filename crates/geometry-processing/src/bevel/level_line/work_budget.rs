//! The level-line build's work accounting: which phase was running ([`LevelLineWorkPhase`]) and
//! how far a build has got ([`LevelLineProgress`]). Ports the record types of
//! `Bevel/LevelLine/WorkBudget.cs`; the budget itself (`WorkBudget`, with its cancellation and
//! progress callback, and the ambient `LevelLineWork.Current`) lands with the core's work-budget
//! package, whose Rust shape is that package's decision.

/// C# `LevelLineWorkPhase`: which part of the construction was running when the work budget ran
/// out.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LevelLineWorkPhase {
    /// Nothing is running.
    #[default]
    None = 0,

    /// Flooding the chain's two sides, or the band, out over the mesh.
    Sides,

    /// The refinement's worklist: splitting until the mesh can carry the line.
    Refinement,

    /// Rooting the crossed mesh edges.
    EdgeRoots,

    /// Walking the band in from the line to the chain, to say what comes out.
    BandDelete,

    /// Reading the room off the region's boundary.
    Room,

    /// Removing degenerate input strips before the refinement, and repairing the needles left.
    InputStrips,

    /// Walking the gap's boundary loops.
    GapLoops,

    /// Zipping the two rails together.
    Fill,

    /// Reading the room from INSIDE a cut - a build handed no room, or the pinch search after its
    /// probes. Named apart from [`LevelLineWorkPhase::Room`] because it runs after the cut has
    /// begun: reported as Room, the words under the bar jumped back to the up-front reading
    /// mid-cut. It carries no weight of its own, so the bar holds where the cut had got to.
    CutRoom,
}

/// C# `LevelLineProgress`: how far a level line build has got - what is running, the work it has
/// spent, and a 0..1 fraction of the whole call. Reported for the bar in front of the user;
/// nothing in the construction reads it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LevelLineProgress {
    /// What was running when this was reported.
    pub phase: LevelLineWorkPhase,
    /// Units of work spent by then.
    pub work_spent: i64,
    /// How far through the call this is, 0..1, never going backwards within one call.
    pub fraction: f64,
}
