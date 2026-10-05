//! The pinch ladder: halving the size down from one that pinched until a probe builds. Ports the
//! nested record struct of `Bevel/LevelLine/PinchLadderSearch.cs` (a C# static class): what a ladder
//! found ([`PinchLadderSearchPinchLadder`]), with the answer for a ladder that ran out
//! ([`ladder_ran_out`]) and who a pinch is charged to ([`who_pinched`]). The search lands with a
//! later package.

use std::rc::Rc;

use super::{BandGapRefusal, BandGapResult, ChainSideLoops, GapLoop, RoomSource, WorkBudget};

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

/// How many times the ladder halves the size down from one that pinched.
pub const PINCH_HALVINGS: i32 = 6;

/// How many bisections it then spends between the last pinch and the first build.
pub const PINCH_BISECTIONS: i32 = 3;

/// The answer for a ladder that ran out of work before ANY size was proven: refused for the
/// budget, with the phase and totals of the allowance it ran out of, the loops and side loops so
/// far (C#'s nulls are `None`, read as none), and how many probes it got through - so a caller
/// looking at why has the count without running the search again.
///
/// Only asked of a ladder that did run out, which always carries its budget (C# reads it
/// unguarded; a ladder without one here is a defect and panics as C#'s null dereference would).
pub fn ladder_ran_out(
    ladder: &PinchLadderSearchPinchLadder,
    loops: Option<Vec<GapLoop>>,
    side_loops: Option<Vec<ChainSideLoops>>,
) -> BandGapResult {
    let budget = ladder
        .ran_out_budget
        .as_ref()
        .expect("a ladder that ran out carries the budget it ran out of");

    BandGapResult {
        refusal: BandGapRefusal::WorkBudgetExceeded,
        room: 0.0,
        phase: budget.exhausted_in.get(),
        work_spent: budget.spent.get(),
        work_limit: budget.limit,
        loops: loops.unwrap_or_default(),
        side_loops: side_loops.unwrap_or_default(),
        pinch_probes: ladder.probes,
        ..BandGapResult::default()
    }
}

/// Who a pinch is charged to: the first side that is not merely owed - the chain it met first,
/// or the chain's own room where it met none - and nothing where every side is owed.
pub fn who_pinched(side_loops: Option<&[ChainSideLoops]>) -> RoomSource {
    for row in side_loops.unwrap_or_default() {
        if row.owed {
            continue;
        }

        return match row.met_chains.first() {
            Some(&met) => RoomSource::chain_to_chain(row.chain, met),
            None => RoomSource::self_room(row.chain),
        };
    }

    RoomSource::nothing()
}
