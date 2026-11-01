//! The room's bracket over the foreign edges where some chain point's setback per unit of size is
//! zero (`foreign_edge_room_readings::room_for`'s caller of [`bracket_past_a_zero_setback`]): the
//! rolling ball's round closing onto a free end's vertex sets that vertex's setback to nothing,
//! and the bare pass's bracket ([`bracket_from_one_bare_pass`](super::foreign_edge_room_readings::bracket_from_one_bare_pass))
//! needs every setback positive for its upper end. Without a bracket the reading doubles from the
//! size asked and bisects every edge unpruned, which on the T Handle cost a chain's room reading
//! about thirteen times the angle's.
//!
//! **The upper end.** The field at size s is no more, at a point x, than x's distance from a chain
//! point p_k less that point's setback s u_k, so an edge reaches the band (value at or below its
//! extra share s e) by the size d(edge, p_k) / (u_k + e). Taking p_k the point with the largest
//! unit setback (the first such, in chain order) gives every edge a finite upper end; the room lies
//! below the least of them, and an edge whose lower end (approach / (largest + e), as the bare pass
//! reads it) stands above that can never set it. The bound can sit exactly on the room, where
//! rounding leaves the value a hair above zero, and a field bounded past an end plane can stand
//! above that distance, so the upper end is checked (some edge's value at or below zero there),
//! doubled a few times where it does not hold, and the bracket dropped where it still does not.
//!
//! **Only with the rolling ball's switch on.** A pick's setbacks from the angle can be zero too (a
//! chain vertex with no turn in its crease), and the reading there stays what it was with the
//! switch off, bit for bit, until the switch goes.

use super::foreign_edge_room_readings::{
    every_value_is_above_zero, BarePassBracket, FieldAt, ForeignEdge,
};
use super::{foreign_room_counters, ChainField, LevelLineMesh, WorkError};

/// The bracket read off one bare pass with its upper end through the chain point of the largest
/// unit setback (see the file header); unbracketed (`bracketed` false) where the unit setbacks do
/// not run one per point of the field's chain, the largest is not positive, an extra share is
/// negative, or the upper end does not hold. An edge whose approach is at or below zero touches
/// the chain, as in the bare pass.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn bracket_past_a_zero_setback(
    mesh: &LevelLineMesh,
    foreign: &[ForeignEdge],
    field_at: FieldAt,
    unit_setback: &[f64],
    highest_unit: f64,
) -> Result<BarePassBracket, WorkError> {
    let mut answer = BarePassBracket::default();
    let bare = field_at(0.0);
    if !(highest_unit > 0.0) || bare.points().len() != unit_setback.len() {
        return Ok(answer);
    }

    // The first point carrying the largest unit setback: ties go to the earlier point.
    let Some(k) = unit_setback.iter().position(|&u| u == highest_unit) else {
        return Ok(answer);
    };
    let through = bare.points()[k];
    let mut low_ends = vec![0.0; foreign.len()];
    let mut bracket_low = f64::INFINITY;
    let mut bracket_high = f64::INFINITY;
    for (i, &(low, high, extra_unit)) in foreign.iter().enumerate() {
        foreign_room_counters::read();
        let (a, b) = (mesh.position(low), mesh.position(high));
        let approach = bare.min_along_segment(a, b)?.value;
        if !(approach > 0.0) {
            answer.touches_the_chain = approach <= 0.0 && extra_unit >= 0.0;
            return Ok(answer);
        }

        if !(extra_unit >= 0.0) {
            return Ok(answer);
        }

        low_ends[i] = approach / (highest_unit + extra_unit);
        bracket_low = vector_math::ieee_math::min_f64(bracket_low, low_ends[i]);
        let reach = (through - ChainField::closest_on_segment(through, a, b)).length();
        bracket_high =
            vector_math::ieee_math::min_f64(bracket_high, reach / (highest_unit + extra_unit));
    }

    if !(bracket_low > 0.0) || !bracket_high.is_finite() || bracket_high < bracket_low {
        return Ok(answer);
    }

    // Checked, and doubled a few times where it does not hold: the bound can sit exactly on the
    // room (the nearest edge seen from the point of the largest setback), where rounding leaves the
    // value a hair above zero, and a field bounded past an end plane can stand above it.
    for _ in 0..UPPER_END_TRIES {
        let searched: Vec<ForeignEdge> = foreign
            .iter()
            .zip(&low_ends)
            .filter(|(_, &low_end)| low_end <= bracket_high)
            .map(|(&edge, _)| edge)
            .collect();
        if !every_value_is_above_zero(mesh, &searched, field_at, bracket_high)? {
            answer.searched = Some(searched);
            answer.bracketed = true;
            answer.lo = bracket_low;
            answer.hi = bracket_high;
            return Ok(answer);
        }
        bracket_high *= 2.0;
    }
    Ok(answer)
}

/// How many upper ends are checked (the first, then each doubled) before the bracket is dropped.
const UPPER_END_TRIES: i32 = 4;
