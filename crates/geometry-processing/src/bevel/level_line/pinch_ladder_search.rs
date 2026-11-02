//! The pinch ladder: halving the size down from one that pinched until a probe builds. What a
//! ladder found ([`PinchLadderSearchPinchLadder`]), with the answer for a ladder that ran out
//! ([`ladder_ran_out`]), who a pinch is charged to ([`who_pinched`]), and the search itself
//! ([`pinch_room_below`], each probe [`leaves_one_loop_per_side`]).

use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap::build_unvalidated;
use super::level_line_routing;
use super::seam_edges::FeatureEdgeTest;
use super::{
    BandGapRefusal, BandGapResult, BandProfile, ChainSideLoops, GapLoop, LevelLineWork,
    LevelLineWorkPhase, RoomSource, WorkBudget, WorkError,
};

/// What a pinch ladder found - the room, how many probes it
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
/// far (`None` read as none), and how many probes it got through - so a caller
/// looking at why has the count without running the search again.
///
/// Only asked of a ladder that did run out, which always carries its budget; a ladder without one
/// here is a defect and panics.
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

/// The largest size below `pinching_size` (a size that
/// pinched) at which the pick's sides each leave their one loop - halving down until a probe
/// builds, then bisecting the bracket - on ONE allowance for the whole search carrying the
/// caller's cancellation. A probe that cannot be built at all (a fault out of the build) reads as
/// one that pinched: it left no loop. The caller's cancel leaves as [`WorkError::Cancelled`].
#[allow(clippy::too_many_arguments)]
pub fn pinch_room_below(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    pinching_size: f64,
    sharp_turn_degrees: f64,
) -> Result<PinchLadderSearchPinchLadder, WorkError> {
    if !pinching_size.is_finite() || pinching_size <= 0.0 {
        // There is no positive size below this one to try, so "every size tried pinched" is
        // vacuously the whole truth.
        return Ok(PinchLadderSearchPinchLadder {
            room: 0.0,
            probes: 0,
            pinches_everywhere: true,
            ran_out_budget: None,
        });
    }

    let outer = LevelLineWork::current();

    // ONE allowance for the WHOLE search, and the caller's own cancellation on it.
    let mut search_budget = WorkBudget::with_cancellation(
        outer.as_ref().map_or_else(
            || WorkBudget::default_limit(indices.len() as i32),
            |o| o.limit,
        ),
        outer
            .as_ref()
            .map(|o| o.cancellation_token.clone())
            .unwrap_or_default(),
        outer.as_ref().and_then(|o| o.a_slice_for_a_nested_search()),
    )
    .expect("a budget's limit is a positive count of work");
    // A probe reads the same input the build does, so it reads it at the same precision.
    search_budget.input_is_float_stored = outer.as_ref().is_some_and(|o| o.input_is_float_stored);
    let search_budget = Rc::new(search_budget);

    // Watched like every other allowance.
    if let Some(watch) = level_line_routing::watch_budget() {
        watch(&search_budget);
    }

    let mut probes = 0;
    let mut one_loop_each_side = |size: f64| {
        probes += 1;
        leaves_one_loop_per_side(
            positions,
            indices,
            chains,
            closed,
            is_feature_edge.clone(),
            size,
            profile,
            snap,
            &search_budget,
            sharp_turn_degrees,
        )
    };

    let mut high = pinching_size;
    let mut low = 0.0;
    let mut ran_out = false;
    for _ in 0..PINCH_HALVINGS {
        let next = high * 0.5;
        if next.is_nan() || next <= 0.0 || next >= high {
            break;
        }

        match one_loop_each_side(next)? {
            // A probe that ran out of work says nothing about the part.
            None => {
                ran_out = true;
                break;
            }
            Some(true) => {
                low = next;
                break;
            }
            Some(false) => high = next,
        }
    }

    if low.is_nan() || low <= 0.0 {
        drop(one_loop_each_side);
        return Ok(if ran_out {
            PinchLadderSearchPinchLadder {
                room: 0.0,
                probes,
                pinches_everywhere: false,
                ran_out_budget: Some(search_budget),
            }
        } else {
            PinchLadderSearchPinchLadder {
                room: 0.0,
                probes,
                pinches_everywhere: true,
                ran_out_budget: None,
            }
        });
    }

    // Three bisections of the bracket [low, high], where high is exactly twice low.
    for _ in 0..PINCH_BISECTIONS {
        let mid = 0.5 * (low + high);
        if mid.is_nan() || mid <= low || mid >= high {
            break;
        }

        match one_loop_each_side(mid)? {
            None => break,
            Some(true) => low = mid,
            Some(false) => high = mid,
        }
    }

    drop(one_loop_each_side);
    Ok(PinchLadderSearchPinchLadder {
        room: low,
        probes,
        pinches_everywhere: false,
        ran_out_budget: None,
    })
}

/// One probe - a topology-only build at `size` on the
/// search's allowance. `None` where the probe ran out of work (it says nothing about the part),
/// `Some(false)` where it refused or (one chain) a side left other than its one loop.
///
/// A cancel leaves the probe as [`WorkError::Cancelled`], ending the whole build.
#[allow(clippy::too_many_arguments)]
pub fn leaves_one_loop_per_side(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    snap: f64,
    search_budget: &Rc<WorkBudget>,
    sharp_turn_degrees: f64,
) -> Result<Option<bool>, WorkError> {
    let cut = build_unvalidated(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        size,
        profile,
        0.0,
        snap,
        Some(f64::INFINITY),
        Some(Rc::clone(search_budget)),
        true,
        0.0,
        sharp_turn_degrees,
        0,
    );
    LevelLineWork::enter(LevelLineWorkPhase::CutRoom);

    let cut = match cut {
        Ok(cut) => cut,
        Err(super::band_gap::BandGapBuildError::Cancelled) => return Err(WorkError::Cancelled),
        Err(_) => return Ok(Some(false)),
    };

    if cut.refusal == BandGapRefusal::WorkBudgetExceeded {
        return Ok(None);
    }

    if cut.refusal != BandGapRefusal::None {
        return Ok(Some(false));
    }

    if chains.len() > 1 {
        // The build's OWN several-chain rule answered: no refusal means every (chain, side) left
        // its one owned loop.
        return Ok(Some(true));
    }

    for side in [1, 2] {
        let mine: Vec<&GapLoop> = cut.loops.iter().filter(|l| l.side == side).collect();
        if mine.len() != 1 || (closed[0] && !mine[0].is_closed) {
            return Ok(Some(false));
        }
    }

    Ok(Some(true))
}
