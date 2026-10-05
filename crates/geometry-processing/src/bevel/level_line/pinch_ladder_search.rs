//! The pinch ladder: halving the size down from one that pinched until a probe builds. Ports the
//! nested record struct of `Bevel/LevelLine/PinchLadderSearch.cs` (a C# static class): what a ladder
//! found ([`PinchLadderSearchPinchLadder`]). The search lands with a later package.

use std::rc::Rc;

use super::WorkBudget;

/// C# `PinchLadderSearch.PinchLadder`: what a pinch ladder found - the room, how many probes it
/// cost, whether every size it tried pinched, and - when the search ran out of work before ANY size
/// was proven - the allowance it ran out of.
///
/// Three outcomes, and they are three different statements. A `room` above zero is a size a probe
/// BUILT at. `pinches_everywhere` says every rung was tried and every one pinched - a statement
/// about the PART. `ran_out_budget` not `None` says the search ran out of work with nothing proven
/// at all: a statement about this machine's allowance, which must leave as a work-budget refusal
/// and never as a size (see `LadderRanOut`).
#[derive(Debug, Clone, Default)]
pub struct PinchLadderSearchPinchLadder {
    /// The room found: a size a probe built at, or zero.
    pub room: f64,
    /// How many probes the ladder cost.
    pub probes: i32,
    /// True when every rung was tried and every one pinched.
    pub pinches_everywhere: bool,
    /// The budget the search ran out of before proving anything, or `None`.
    pub ran_out_budget: Option<Rc<WorkBudget>>,
}
