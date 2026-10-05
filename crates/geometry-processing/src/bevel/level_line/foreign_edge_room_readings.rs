//! The room foreign features leave a chain: the readings over the foreign edges. Ports the room
//! readings of `Bevel/LevelLine/ForeignEdgeRoom.cs` (a C# static class; its constants, knife test and
//! edge lists are in `foreign_edge_room.rs`): whether every edge's value stands above zero at a
//! trial size ([`every_value_is_above_zero`]), the bisection every room reading ends in
//! ([`bisect`]) and where the edge that sets the room comes nearest the chain
//! ([`where_the_room_is_set`]), and the bracket read off one pass of the bare field
//! ([`bracket_from_one_bare_pass`]) and the room itself ([`room_for`]). The indexed reading lands
//! with its method package.

use vector_math::Vector3;

use super::foreign_edge_room::{
    ROOM_BISECTION_STEPS, ROOM_BRACKET_DOUBLINGS, UNIFORM_SETBACK_TOLERANCE,
};
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

/// What [`bracket_from_one_bare_pass`] reads.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BarePassBracket {
    /// True where the bracket holds; false sends the room reading down its doubling path.
    pub bracketed: bool,
    /// The edges the bisection still has to read (those whose lower end stands at or below the
    /// bracket's upper end); `None` where nothing was bracketed (C# leaves its `ref` untouched).
    pub searched: Option<Vec<ForeignEdge>>,
    /// The bracket's low end: a size at which every value is still at or above zero.
    pub lo: f64,
    /// The bracket's high end.
    pub hi: f64,
    /// True where an edge already stands on the chain: the room is zero outright.
    pub touches_the_chain: bool,
}

/// C# `ForeignEdgeRoom.BracketFromOneBarePass`: brackets the room from ONE pass of the BARE field
/// (setback zero) over the foreign edges: with every unit setback in [`lowest_unit`,
/// `highest_unit`] and positive, each edge's critical size lies in
/// [approach / (highest + extra), approach / (lowest + extra)], so the room lies between the least
/// lower end and the least upper end, and the edges whose lower end stands above that least upper
/// end can never matter (pruned once, against the initial upper end). Not bracketed where the
/// derivation does not hold (a unit setback not positive, a negative extra); an edge whose
/// approach is at or below zero touches the chain.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn bracket_from_one_bare_pass(
    mesh: &LevelLineMesh,
    foreign: &[ForeignEdge],
    field_at: FieldAt,
    lowest_unit: f64,
    highest_unit: f64,
) -> Result<BarePassBracket, WorkError> {
    let mut answer = BarePassBracket::default();
    if !(lowest_unit > 0.0) {
        return Ok(answer);
    }

    let bare = field_at(0.0);
    let mut low_ends = vec![0.0; foreign.len()];
    let mut bracket_low = f64::INFINITY;
    let mut bracket_high = f64::INFINITY;
    for (i, &(low, high, extra_unit)) in foreign.iter().enumerate() {
        foreign_room_counters::read();
        let approach = bare
            .min_along_segment(mesh.position(low), mesh.position(high))?
            .value;
        if !(approach > 0.0) {
            // Nothing can be built between the chain and something already standing on it.
            answer.touches_the_chain = approach <= 0.0 && extra_unit >= 0.0;
            return Ok(answer);
        }

        if !(extra_unit >= 0.0) {
            return Ok(answer);
        }

        low_ends[i] = approach / (highest_unit + extra_unit);
        bracket_low = vector_math::cs_math::min_f64(bracket_low, low_ends[i]);
        bracket_high =
            vector_math::cs_math::min_f64(bracket_high, approach / (lowest_unit + extra_unit));
    }

    if !(bracket_low > 0.0) || bracket_high.is_infinite() {
        return Ok(answer);
    }

    answer.searched = Some(
        foreign
            .iter()
            .zip(&low_ends)
            .filter(|(_, &low_end)| low_end <= bracket_high)
            .map(|(&edge, _)| edge)
            .collect(),
    );
    answer.bracketed = true;
    answer.lo = bracket_low;
    answer.hi = bracket_high;
    Ok(answer)
}

/// C# `ForeignEdgeRoom.RoomFor`: the room the `foreign` edges leave the chain - the size at which
/// the band first reaches one of them; +infinity with none. Where the chain asks ONE setback per
/// unit of size the room is read in one pass of the bare field (each edge's closest approach over
/// its share of the setback - the two bands close on each other from both ends of the approach).
/// Otherwise it is bracketed from one bare pass and bisected; where the bracket does not hold, an
/// edge already touching the chain is room zero, and the room is otherwise bracketed by doubling
/// from the `asked` size (+infinity where it never closes).
pub fn room_for(
    mesh: &LevelLineMesh,
    foreign: &[ForeignEdge],
    field_at: FieldAt,
    unit_setback: &[f64],
    asked: f64,
) -> Result<f64, WorkError> {
    if foreign.is_empty() {
        return Ok(f64::INFINITY);
    }

    foreign_room_counters::with(|t| {
        t.calls += 1;
        t.edges_offered += foreign.len() as i64;
        if t.in_a_pair {
            t.pair_calls += 1;
        } else if let Some(per_chain) = &mut t.edges_per_chain {
            per_chain.push(foreign.len() as i32);
        }
    });

    let (lowest_unit, highest_unit) = least_and_greatest(unit_setback);
    if lowest_unit > 0.0 && highest_unit - lowest_unit <= UNIFORM_SETBACK_TOLERANCE * highest_unit {
        // The bare distance field - no setback taken off - so each edge's approach is read once.
        let bare = field_at(0.0);
        let mut best = f64::MAX;
        for &(low, high, extra_unit) in foreign {
            foreign_room_counters::read();
            let approach = bare
                .min_along_segment(mesh.position(low), mesh.position(high))?
                .value;
            best = vector_math::cs_math::min_f64(best, approach / (highest_unit + extra_unit));
        }

        return Ok(best);
    }

    let bracket = bracket_from_one_bare_pass(mesh, foreign, field_at, lowest_unit, highest_unit)?;
    let searched: Vec<ForeignEdge> = bracket.searched.unwrap_or_else(|| foreign.to_vec());
    let (mut lo, mut hi) = (bracket.lo, bracket.hi);
    if !bracket.bracketed {
        foreign_room_counters::with(|t| {
            t.bracket_fallbacks += 1;
            if bracket.touches_the_chain {
                t.touching_fallbacks += 1;
            }
        });
        if bracket.touches_the_chain {
            // An edge ALREADY standing on the chain leaves nothing to build in: the room is zero.
            return Ok(0.0);
        }

        lo = 0.0;
        hi = asked;
        let mut doubling = 0;
        while doubling < ROOM_BRACKET_DOUBLINGS
            && every_value_is_above_zero(mesh, &searched, field_at, hi)?
        {
            LevelLineWork::charge(1)?;
            lo = hi;
            hi *= 2.0;
            doubling += 1;
        }

        if every_value_is_above_zero(mesh, &searched, field_at, hi)? {
            return Ok(f64::INFINITY);
        }
    }

    bisect(mesh, &searched, field_at, lo, hi)
}
