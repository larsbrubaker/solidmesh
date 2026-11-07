//! How deep a swept tool may reach BACK into the part before its turn folds it over itself
//!. A chain's tightest turn
//! names a radius the tool's back reach has to stay inside; where it does not, the sweep is
//! retried at the depth the turn allows.

use vector_math::ieee_math;

use super::bevel_chain_sweep_mesh::SweptSegment;
use crate::topology::ChainEdge;

/// The share of the turn radius the back reach may use - a margin so the fold is never met
/// exactly.
pub const BACK_DEPTH_MARGIN_OF_THE_TURN: f64 = 0.8;

/// The least retry depth worth trying, as a share of the size: shallower than this the retried
/// tool takes almost nothing off and the bevel is better refused.
pub const MINIMUM_RETRY_FRACTION_OF_SIZE: f64 = 0.1;

/// The size the tool's back reach scales down to so that it stays inside the chain's tightest
/// turn (the lesser of the chain's own and a `measured_turn_radius`, infinity where none was
/// measured) by the margin, at `size` with `back_reach` at full size; zero where it already fits
/// or the chain does not turn.
pub fn back_depth_the_turn_allows(
    chain: &[ChainEdge],
    is_closed: bool,
    size: f64,
    back_reach: f64,
    measured_turn_radius: f64,
) -> f64 {
    let allowed = ieee_math::min_f64(turn_radius(chain, is_closed), measured_turn_radius)
        * BACK_DEPTH_MARGIN_OF_THE_TURN;

    if allowed > 0.0 && back_reach > allowed {
        size * allowed / back_reach
    } else {
        0.0
    }
}

/// The depth to retry the sweep at: [`back_depth_the_turn_allows`], or zero where that is
/// shallower than [`MINIMUM_RETRY_FRACTION_OF_SIZE`] of the size.
pub fn retry_depth(
    chain: &[ChainEdge],
    is_closed: bool,
    size: f64,
    back_reach: f64,
    measured_turn_radius: f64,
) -> f64 {
    let allowed =
        back_depth_the_turn_allows(chain, is_closed, size, back_reach, measured_turn_radius);

    if allowed >= size * MINIMUM_RETRY_FRACTION_OF_SIZE {
        allowed
    } else {
        0.0
    }
}

/// How far the swept profile reaches back from its edge: the greatest profile `y` of any segment.
pub fn measure_back_reach(swept: &[SweptSegment]) -> f64 {
    let mut reach = 0.0;

    for segment in swept {
        for point in &segment.profile {
            reach = ieee_math::max_f64(reach, point.y);
        }
    }

    reach
}

/// The chain's tightest turn as a radius: at each joint, the shorter of the two edges over the
/// angle turned; infinity where the chain never turns.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` refuses a NaN as well.
pub fn turn_radius(chain: &[ChainEdge], is_closed: bool) -> f64 {
    let mut tightest = f64::INFINITY;
    let count = chain.len();

    for i in (if is_closed { 0 } else { 1 })..count {
        let before = &chain[(i + count - 1) % count];
        let after = &chain[i];
        let turn = before
            .direction()
            .dot(after.direction())
            .clamp(-1.0, 1.0)
            .acos();

        if !(turn > 1e-9) {
            continue;
        }

        let span = ieee_math::min_f64(
            (before.to() - before.from()).length(),
            (after.to() - after.from()).length(),
        );

        tightest = ieee_math::min_f64(tightest, span / turn);
    }

    tightest
}

/// The same reading over the swept segments: each segment's own length and direction.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` refuses a NaN as well.
pub fn turn_radius_of_the_segments(swept: &[SweptSegment], is_closed: bool) -> f64 {
    let mut tightest = f64::INFINITY;
    let count = swept.len();

    for i in (if is_closed { 0 } else { 1 })..count {
        let before = &swept[(i + count - 1) % count];
        let after = &swept[i];
        let turn = before
            .direction
            .dot(after.direction)
            .clamp(-1.0, 1.0)
            .acos();

        if !(turn > 1e-9) {
            continue;
        }

        tightest = ieee_math::min_f64(
            tightest,
            ieee_math::min_f64(before.length, after.length) / turn,
        );
    }

    tightest
}
