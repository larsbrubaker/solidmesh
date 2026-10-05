//! What a run does at each of its two ends (`Bevel/BevelChainEndCap.cs`): [`BevelChainEnds`],
//! the decision `BevelEndCap` records one half of. Resolving it - the cap plane scan, the 5.3
//! corner exemption, the bead's square cap and the overshoot bound - lands with the core's
//! packages as an `impl` block.
//!
//! A cutter that finds a face bounding its end vertex stops on that face's plane; one that finds
//! none overshoots, which costs nothing because what it reaches is material a neighbouring tool
//! takes anyway, or air. A BEAD that finds none squares off instead, because every bit of tool
//! hanging past the last vertex is material ADDED where the user selected nothing - except at a
//! 5.3 corner, where the overshoot is the mechanism: the tools overlap and their union resolves
//! the corner. Only ONE plane caps an end; where several faces bound a corner the most square to
//! the run is used.

use super::BevelEndCap;

/// C# `BevelChainEnds`: the end treatment for one run of a group's selection.
///
/// `Default` is C#'s `BevelChainEnds.Overshoot`: both ends run past the last vertex, unbounded -
/// what a closed loop and a run with nothing to cap against get.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelChainEnds {
    /// The faces the run leaves the part through at a START that had to square off, as planes
    /// the bead may not reach past (`BevelBeadExit`). `None` at every end that capped, overshot,
    /// or is a cutter's.
    pub start_exit_planes: Option<Vec<BevelEndCap>>,
    /// As [`BevelChainEnds::start_exit_planes`], at the run's end.
    pub end_exit_planes: Option<Vec<BevelEndCap>>,
    /// What happens where the run begins.
    pub start: BevelEndCap,
    /// What happens where the run ends.
    pub end: BevelEndCap,
    /// The vertex where this run's START deliberately overshoots into another selected run of
    /// its own convexity across an unselected CONCAVE crease between them
    /// (`BevelPairCornerCrease`) - 5.3's PAIR CORNER - or `None` where it does not. Decided here
    /// and nowhere else; `BevelPairCornerTrim` builds the solid that follows from it.
    pub start_pair_corner: Option<i32>,
    /// As [`BevelChainEnds::start_pair_corner`], at the run's end.
    pub end_pair_corner: Option<i32>,
    /// The vertex this run's START runs PAST because another selected run of its own convexity
    /// departs from it - 5.3's corner read at its widest, recognised crease or not - or `None`.
    /// Only an UNCAPPED end fills it in. What reads it is the bead's void wedge
    /// (`BevelGroupTools`), which is squared off there while the vertex's own void cone carries
    /// the corner.
    pub start_overshoot_corner: Option<i32>,
    /// As [`BevelChainEnds::start_overshoot_corner`], at the run's end.
    pub end_overshoot_corner: Option<i32>,
    /// How far past its own last vertex this run's START may reach when it is uncapped: half the
    /// segment adjoining the end, so the spill can never be more than half of what the user
    /// picked there - infinity (the overshoot unbounded) at a 5.3 corner, which is exempt and
    /// must be, and wherever no bound applies.
    pub start_maximum_overshoot: f64,
    /// As [`BevelChainEnds::start_maximum_overshoot`], at the run's end.
    pub end_maximum_overshoot: f64,
}

impl Default for BevelChainEnds {
    fn default() -> Self {
        // C#'s constructor defaults: no pair or overshoot corner, no exit planes, and both
        // overshoots unbounded.
        Self {
            start_exit_planes: None,
            end_exit_planes: None,
            start: BevelEndCap::default(),
            end: BevelEndCap::default(),
            start_pair_corner: None,
            end_pair_corner: None,
            start_overshoot_corner: None,
            end_overshoot_corner: None,
            start_maximum_overshoot: f64::INFINITY,
            end_maximum_overshoot: f64::INFINITY,
        }
    }
}
