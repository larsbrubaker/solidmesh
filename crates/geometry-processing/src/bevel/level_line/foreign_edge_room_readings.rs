//! The room foreign features leave a chain: the readings over the foreign edges (the constants,
//! knife test and edge lists are in `foreign_edge_room.rs`): whether every edge's value stands
//! above zero at a
//! trial size ([`every_value_is_above_zero`]), the bisection every room reading ends in
//! ([`bisect`]) and where the edge that sets the room comes nearest the chain
//! ([`where_the_room_is_set`]), and the bracket read off one pass of the bare field
//! ([`bracket_from_one_bare_pass`]), the room itself ([`room_for`]) and the room
//! read off a spatial index of the foreign edges ([`room_from_index`]). A field a stopped end clips
//! (`stopped_end_clip.rs`, [`ChainField::is_clipped_by_a_stop`]) is not its bare field less the
//! setback: its room starts from the bare field's reading as a lower bound and is bracketed by
//! doubling and bisected on the field itself, over every edge (`room_on_a_stopped_field`).

use vector_math::Vector3;

use std::rc::Rc;

use super::foreign_edge_room::{
    every_edge, read_every_foreign_edge, PRUNE_SLACK, ROOM_BISECTION_STEPS, ROOM_BRACKET_DOUBLINGS,
    UNIFORM_SETBACK_TOLERANCE,
};
use super::ForeignEdgeIndex;
use super::{foreign_room_counters, ChainField, LevelLineMesh, LevelLineWork, WorkError};

/// A foreign edge as (low, high, the other chain's extra unit setback taken off it).
pub type ForeignEdge = (i32, i32, f64);

/// The field at a trial size.
pub type FieldAt<'a> = &'a dyn Fn(f64) -> ChainField;

/// The least and greatest of `values`: the least is NaN if any value is (a broken reading is
/// never hidden), the greatest only if all are.
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

/// Where the edge that sets `room` comes nearest the
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

/// Whether EVERY foreign edge's field value stands
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

/// The fixed bisection every room reading ends in - the size between
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
    /// bracket's upper end); `None` where nothing was bracketed.
    pub searched: Option<Vec<ForeignEdge>>,
    /// The bracket's low end: a size at which every value is still at or above zero.
    pub lo: f64,
    /// The bracket's high end.
    pub hi: f64,
    /// True where an edge already stands on the chain: the room is zero outright.
    pub touches_the_chain: bool,
}

/// Brackets the room from ONE pass of the BARE field
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
        bracket_low = vector_math::ieee_math::min_f64(bracket_low, low_ends[i]);
        bracket_high =
            vector_math::ieee_math::min_f64(bracket_high, approach / (lowest_unit + extra_unit));
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

/// The room the `foreign` edges leave the chain - the size at which
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
    if field_at(0.0).is_clipped_by_a_stop() {
        return room_on_a_stopped_field(mesh, foreign, field_at, highest_unit, asked);
    }

    if lowest_unit > 0.0 && highest_unit - lowest_unit <= UNIFORM_SETBACK_TOLERANCE * highest_unit {
        // The bare distance field - no setback taken off - so each edge's approach is read once.
        let bare = field_at(0.0);
        let mut best = f64::MAX;
        for &(low, high, extra_unit) in foreign {
            foreign_room_counters::read();
            let approach = bare
                .min_along_segment(mesh.position(low), mesh.position(high))?
                .value;
            best = vector_math::ieee_math::min_f64(best, approach / (highest_unit + extra_unit));
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

/// The room the `foreign` edges leave a field a stopped end clips (`stopped_end_clip.rs`). The
/// clip's plane does not scale with the size, so the field is not its bare field less the setback;
/// but the clip only raises the field, so the bare field's approach over the greatest setback is
/// still a lower bound. From there the room is bracketed by doubling and bisected on the field
/// itself, every edge read at every trial size and charged to the budget. An edge already standing
/// on the chain leaves room zero; +infinity where the bracket never closes.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn room_on_a_stopped_field(
    mesh: &LevelLineMesh,
    foreign: &[ForeignEdge],
    field_at: FieldAt,
    highest_unit: f64,
    asked: f64,
) -> Result<f64, WorkError> {
    let bare = field_at(0.0);
    let mut lo = f64::INFINITY;
    for &(low, high, extra_unit) in foreign {
        foreign_room_counters::read();
        LevelLineWork::charge(1)?;
        let approach = bare
            .min_along_segment(mesh.position(low), mesh.position(high))?
            .value;
        if approach <= 0.0 && extra_unit >= 0.0 {
            return Ok(0.0);
        }

        lo = vector_math::ieee_math::min_f64(lo, approach / (highest_unit + extra_unit));
    }

    // A lower bound that cannot be read (no positive setback, a NaN) starts from nothing.
    if !(lo > 0.0) || lo.is_infinite() {
        lo = 0.0;
    }

    let charge = foreign.len() as i64;
    let mut hi = if lo > 0.0 { 2.0 * lo } else { asked };
    let mut doubling = 0;
    loop {
        LevelLineWork::charge(charge)?;
        if !every_value_is_above_zero(mesh, foreign, field_at, hi)? {
            break;
        }

        if doubling == ROOM_BRACKET_DOUBLINGS {
            return Ok(f64::INFINITY);
        }

        lo = hi;
        hi *= 2.0;
        doubling += 1;
    }

    // The bisection charges a unit a step; every step reads every edge.
    LevelLineWork::charge((charge - 1).max(0) * i64::from(ROOM_BISECTION_STEPS))?;
    bisect(mesh, foreign, field_at, lo, hi)
}

/// The same room as [`room_for`], read off a SPATIAL INDEX of
/// the pick's foreign edges rather than a pass over all of them: the chain's own edges (`exempt`)
/// aside, the answer is the same number. The nearest-first search reads each edge's bare approach
/// once, the uniform setback answers outright, and otherwise the bracket [least / highest,
/// least / lowest] is bisected over the edges near enough to set it. Every edge read the plain way
/// where the switch asks for it, a unit setback is not positive, or an approach is NaN.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn room_from_index(
    mesh: &LevelLineMesh,
    indexes: &[Rc<ForeignEdgeIndex>],
    field_at: FieldAt,
    unit_setback: &[f64],
    asked: f64,
    exempt: Option<&dyn Fn((i32, i32)) -> bool>,
) -> Result<f64, WorkError> {
    let mut offered: i32 = indexes.iter().map(|index| index.count()).sum();
    if offered == 0 {
        return Ok(f64::INFINITY);
    }

    // A stopped end's clip does not scale with the size, so the index's bare-field bracket does
    // not hold: a stopped field is read over every edge by `room_for`.
    let (lowest_unit, highest_unit) = least_and_greatest(unit_setback);
    if read_every_foreign_edge() || !(lowest_unit > 0.0) || field_at(0.0).is_clipped_by_a_stop() {
        return room_for(
            mesh,
            &every_edge(indexes, exempt),
            field_at,
            unit_setback,
            asked,
        );
    }

    // The meters mean what they meant before the index: OFFERED is after the exemption, counted
    // only when a measurement is running.
    let measuring = foreign_room_counters::with(|t| t.edges_per_chain.is_some());
    if let (Some(exempt), true) = (exempt, measuring) {
        for index in indexes {
            offered -= index.edges().iter().filter(|&&edge| exempt(edge)).count() as i32;
        }
    }

    foreign_room_counters::with(|t| {
        t.calls += 1;
        t.edges_offered += i64::from(offered);
        if t.in_a_pair {
            t.pair_calls += 1;
        } else if let Some(per_chain) = &mut t.edges_per_chain {
            per_chain.push(offered);
        }
    });

    let found = ForeignEdgeIndex::least_approach(
        indexes,
        &field_at(0.0),
        (highest_unit / lowest_unit) * (1.0 + PRUNE_SLACK),
        exempt,
    )?;
    foreign_room_counters::with(|t| t.search_edges_read += i64::from(found.read));
    if found.least == f64::INFINITY {
        // Every edge the indexes hold was exempt: nothing foreign bounds this chain at all.
        return Ok(f64::INFINITY);
    }

    if highest_unit - lowest_unit <= UNIFORM_SETBACK_TOLERANCE * highest_unit {
        return Ok(found.least / highest_unit);
    }

    if found.least.is_nan() {
        // A NaN approach is a NaN room on the unindexed path; the whole reading goes back there.
        return room_for(
            mesh,
            &every_edge(indexes, exempt),
            field_at,
            unit_setback,
            asked,
        );
    }

    if !(found.least > 0.0) {
        // An edge already standing on the chain leaves nothing to build in.
        foreign_room_counters::with(|t| {
            t.bracket_fallbacks += 1;
            t.touching_fallbacks += 1;
        });
        return Ok(0.0);
    }

    let lo = found.least / highest_unit;
    let hi = found.least / lowest_unit;
    // Unreachable with a positive least approach and a positive unit setback.
    assert!(
        lo > 0.0 && !hi.is_infinite(),
        "a positive approach and unit setback did not bracket"
    );

    let kept: Vec<ForeignEdge> = found
        .near
        .iter()
        .filter(|(_, approach)| approach / highest_unit <= hi)
        .map(|&(edge, _)| (edge.0, edge.1, 0.0))
        .collect();
    foreign_room_counters::with(|t| t.kept_edges += kept.len() as i64);
    bisect(mesh, &kept, field_at, lo, hi)
}
