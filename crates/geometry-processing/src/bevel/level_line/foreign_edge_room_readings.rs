//! The room foreign features leave a chain: the readings over the foreign edges. Ports the room
//! readings of `Bevel/LevelLine/ForeignEdgeRoom.cs` (a C# static class; its constants, knife test and
//! edge lists are in `foreign_edge_room.rs`): whether every edge's value stands above zero at a
//! trial size ([`every_value_is_above_zero`]), the bisection every room reading ends in
//! ([`bisect`]) and where the edge that sets the room comes nearest the chain
//! ([`where_the_room_is_set`]). The readings that bracket and index the room land with their
//! method package.

use vector_math::Vector3;

use super::foreign_edge_room::{ROOM_BISECTION_STEPS, UNIFORM_SETBACK_TOLERANCE};
use super::{foreign_room_counters, ChainField, LevelLineMesh, LevelLineWork, WorkError};

/// A foreign edge as (low, high, the other chain's extra unit setback taken off it).
pub type ForeignEdge = (i32, i32, f64);

/// The field at a trial size (C#'s `Func<double, ChainField>`).
pub type FieldAt<'a> = &'a dyn Fn(f64) -> ChainField;

/// C#'s `Enumerable.Min` / `Max` over doubles: (least, NaN if any is; greatest, NaN only if all).
fn least_and_greatest(values: &[f64]) -> (f64, f64) {
    let mut least = values[0];
    let mut most = values[0];
    for &v in values {
        if least.is_nan() || v.is_nan() {
            least = f64::NAN;
        } else if v < least {
            least = v;
        }

        if v > most || most.is_nan() {
            most = v;
        }
    }

    (least, most)
}

/// C# `ForeignEdgeRoom.WhereTheRoomIsSet`: where the edge that sets `room` comes nearest the
/// chain: ranked as the room reading ranks, by closest approach over its setback where the setback
/// is one number, and otherwise by the least field value at the room itself, where that edge is the
/// one the band has just reached. Read only once the room is known, so it costs a room nothing.
pub fn where_the_room_is_set(
    mesh: &LevelLineMesh,
    foreign: &[ForeignEdge],
    field_at: FieldAt,
    unit_setback: &[f64],
    room: f64,
) -> Result<Vector3, WorkError> {
    let (lowest_unit, highest_unit) = least_and_greatest(unit_setback);
    let uniform =
        lowest_unit > 0.0 && highest_unit - lowest_unit <= UNIFORM_SETBACK_TOLERANCE * highest_unit;
    let field = field_at(if uniform { 0.0 } else { room });
    let mut best = f64::MAX;
    let mut at = Vector3::ZERO;
    for &(low, high, extra_unit) in foreign {
        let least = field.min_along_segment(mesh.position(low), mesh.position(high))?;
        let rank = if uniform {
            least.value / (highest_unit + extra_unit)
        } else {
            least.value
        };
        if rank < best {
            best = rank;
            at = least.point;
        }
    }

    Ok(at)
}

/// C# `ForeignEdgeRoom.EveryValueIsAboveZero`: whether EVERY foreign edge's field value stands
/// above zero at one trial size - the only thing a bisection or doubling step asks of the least
/// value, so the reading stops at the first edge that answers it. The other chain's own setback is
/// taken off at the largest it reads over the edge. "Not above" rather than "at or below", so a
/// NaN value counts as not above.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn every_value_is_above_zero(
    mesh: &LevelLineMesh,
    searched: &[ForeignEdge],
    field_at: FieldAt,
    size: f64,
) -> Result<bool, WorkError> {
    let field = field_at(size);
    for &(low, high, extra_unit) in searched {
        foreign_room_counters::read();
        let value = field
            .min_along_segment(mesh.position(low), mesh.position(high))?
            .value
            - (size * extra_unit);
        if !(value > 0.0) {
            return Ok(false);
        }
    }

    Ok(true)
}

/// C# `ForeignEdgeRoom.Bisect`: the fixed bisection every room reading ends in - the size between
/// `lo` and `hi` at which the least value is zero.
pub fn bisect(
    mesh: &LevelLineMesh,
    searched: &[ForeignEdge],
    field_at: FieldAt,
    mut lo: f64,
    mut hi: f64,
) -> Result<f64, WorkError> {
    for _ in 0..ROOM_BISECTION_STEPS {
        LevelLineWork::charge(1)?;
        let mid = 0.5 * (lo + hi);
        if every_value_is_above_zero(mesh, searched, field_at, mid)? {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    Ok(0.5 * (lo + hi))
}
