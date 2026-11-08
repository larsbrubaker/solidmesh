//! The refinement's MIXED edge rule, rule (i)'s third face: an edge whose ends read DIFFERENT
//! sides, which neither the dip nor the bump rule asks about and which can still hide a PAIR of
//! crossings past its first root. The refinement that asks it is `band_refinement_asking`.

use std::collections::{HashMap, VecDeque};

use solidmesh_math::Vector3;

use super::band_refinement::{farthest_from_the_line, nearest_to_the_line};
use super::edge_bump_rule::BumpAnswer;
use super::edge_root::root_on;
use super::field_bounds;
use super::refinement_splits::{cannot_be_divided_further, meter_would_thin_on_edge};
use super::refinement_splits_enqueue::{
    bisect_instead_of_thinning_on_edge, split_edge_and_enqueue,
};
use super::{
    ChainDistanceField, FieldCounters, LevelLineMeshError, LevelLineRule, LevelLineTriangle,
    LevelLineWork, VertexFieldReading, VertexSide, WorkError,
};

/// The mixed edge rule, with the work units it spends charged
/// to it on the meter (and nothing else done) when the meter is on. Answers as the bump rule does.
#[allow(clippy::too_many_arguments)]
pub fn ask_mixed_edges(
    reading: &VertexFieldReading,
    field: &dyn ChainDistanceField,
    triangle: i32,
    t: LevelLineTriangle,
    chord_tolerance: f64,
    floor: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<BumpAnswer, LevelLineMeshError> {
    if !FieldCounters::meter_on() {
        return ask_mixed_edges_body(
            reading,
            field,
            triangle,
            t,
            chord_tolerance,
            floor,
            pending,
            depth,
        );
    }

    let spent = || LevelLineWork::current().map_or(0, |b| b.spent.get());
    let before = spent();
    let answer = ask_mixed_edges_body(
        reading,
        field,
        triangle,
        t,
        chord_tolerance,
        floor,
        pending,
        depth,
    );
    // Noted on an error too.
    FieldCounters::note_units(LevelLineRule::MixedEdge, spent() - before);
    answer
}

/// For each edge with one end Inside and the other Outside
/// not already answered clean, splits it ONCE where a hidden crossing pair stands past its first
/// root - or answers it clean on the mesh where the edge is too short to hide a pair or none was
/// found.
#[allow(clippy::too_many_arguments)]
pub fn ask_mixed_edges_body(
    reading: &VertexFieldReading,
    field: &dyn ChainDistanceField,
    triangle: i32,
    t: LevelLineTriangle,
    chord_tolerance: f64,
    floor: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<BumpAnswer, LevelLineMeshError> {
    for corner in 0..3 {
        let v = t.corner(corner);
        let w = t.corner((corner + 1) % 3);
        let side_v = reading.side_of(v)?;
        let side_w = reading.side_of(w)?;

        // Only an edge running from Inside to Outside; an On end is the bump rule's.
        if (side_v != VertexSide::Inside || side_w != VertexSide::Outside)
            && (side_v != VertexSide::Outside || side_w != VertexSide::Inside)
        {
            continue;
        }

        if reading.mesh.borrow().is_answered_clean(v, w) {
            continue;
        }

        let (from, to) = {
            let mesh = reading.mesh.borrow();
            (mesh.position(v), mesh.position(w))
        };
        let v_inside = side_v == VertexSide::Inside;
        let inside = if v_inside { from } else { to };
        let outside = if v_inside { to } else { from };
        let inside_value = if v_inside {
            reading.value_at(v)?
        } else {
            reading.value_at(w)?
        };
        let outside_value = if v_inside {
            reading.value_at(w)?
        } else {
            reading.value_at(v)?
        };

        // Too short to hide a pair from what its ends read: answered clean for good.
        if (from - to).length()
            <= field_bounds::length_to_hide_a_pair(inside_value, outside_value, chord_tolerance)
        {
            reading.mesh.borrow_mut().answer_clean(v, w);
            continue;
        }

        let root = root_on(from, to, reading)?;

        if FieldCounters::meter_on() {
            FieldCounters::asked_by_reaching(
                LevelLineRule::MixedEdge,
                (from - to).length() / (20.0 * chord_tolerance),
                nearest_to_the_line(chord_tolerance, inside_value, outside_value),
                farthest_from_the_line(chord_tolerance, inside_value, outside_value),
            );
        }

        let Some(place) = try_hidden_crossing(
            reading,
            field,
            (inside, outside, root),
            inside_value,
            outside_value,
            chord_tolerance,
        )?
        else {
            reading.mesh.borrow_mut().answer_clean(v, w);
            continue;
        };

        if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
            return Ok(BumpAnswer::Refused);
        }

        FieldCounters::fired(LevelLineRule::MixedEdge);

        let mut mesh = reading.mesh.borrow_mut();
        meter_would_thin_on_edge(
            LevelLineRule::MixedEdge,
            &mesh,
            v,
            w,
            place,
            chord_tolerance,
        );
        if !bisect_instead_of_thinning_on_edge(
            &mut mesh,
            triangle,
            v,
            w,
            place,
            chord_tolerance,
            pending,
            depth,
        )? {
            split_edge_and_enqueue(&mut mesh, v, w, place, pending, depth)?;
        }

        return Ok(BumpAnswer::Split);
    }

    Ok(BumpAnswer::Nothing)
}

/// Where past the first root of a mixed edge a hidden pair
/// of crossings stands - a bump above the line between the inside end and the root, or a dip below
/// it between the root and the outside end - read exactly at the point before it is answered;
/// `None` where there is neither. `(inside, outside, root)` are the edge's inside end, its outside
/// end and its root.
pub fn try_hidden_crossing(
    reading: &VertexFieldReading,
    field: &dyn ChainDistanceField,
    (inside, outside, root): (Vector3, Vector3, Vector3),
    inside_value: f64,
    outside_value: f64,
    chord_tolerance: f64,
) -> Result<Option<Vector3>, WorkError> {
    if field_bounds::greatest_along_edge(inside_value, 0.0, (inside - root).length())
        > chord_tolerance
    {
        let greatest = field.max_along_segment_at_least(inside, root, chord_tolerance, 0.0)?;
        if greatest.value > chord_tolerance
            && reading.value_at_point(greatest.point)? > reading.snap
        {
            return Ok(Some(greatest.point));
        }
    }

    if field_bounds::least_along_edge(0.0, outside_value, (outside - root).length())
        < -chord_tolerance
    {
        let least = field.min_along_segment(root, outside)?;
        if least.value < -chord_tolerance && reading.value_at_point(least.point)? < -reading.snap {
            return Ok(Some(least.point));
        }
    }

    Ok(None)
}
