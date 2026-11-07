//! The refinement's BUMP rule, rule (i) mirrored: an edge of a band triangle whose ends read inside
//! (or one inside and one on the line) while the edge between them stands outside hides a pair of
//! crossings no end reading shows, and is split at its peak. The refinement that asks it is
//! `band_refinement_asking`.

use std::collections::{HashMap, VecDeque};

use super::band_refinement::{farthest_from_the_line, nearest_to_the_line};
use super::field_bounds;
use super::refinement_splits::{cannot_be_divided_further, meter_would_thin_on_edge};
use super::refinement_splits_enqueue::{
    bisect_instead_of_thinning_on_edge, split_edge_and_enqueue,
};
use super::{
    ChainDistanceField, FieldCounters, LevelLineMeshError, LevelLineRule, LevelLineTriangle,
    VertexFieldReading, VertexSide,
};

/// What [`ask`] did: split an edge at its peak, refused because the triangle is too small to
/// divide further, or found nothing to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BumpAnswer {
    /// No edge of the triangle bumps across the line.
    Nothing,
    /// An edge was split at its peak; its children are queued.
    Split,
    /// The triangle is under the floor and cannot be divided further.
    Refused,
}

/// For each edge of `triangle` whose ends read inside (or inside and on)
/// while the edge rises above the line between them, splits it ONCE at its peak - which reads
/// Outside, leaving two plainly crossed halves - or bisects the longest-edge path where that split
/// would thin a child. Read on the reading's own mesh.
///
/// The search is seeded one whole tolerance BELOW the firing threshold: a pruned piece then has a
/// bound at or under the snap and so holds nothing that could fire, while the searches that find
/// nothing get cheap. When nothing beats the seed the search hands the seed back at the segment's
/// start, which the strict `>` below rejects.
#[allow(clippy::too_many_arguments)]
pub fn ask(
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
        // An end ON the line hides the same crossing: an inside end beside an on end, with the edge
        // standing outside between them, crosses near the inside end and nowhere the ends can show.
        let side_v = reading.side_of(v)?;
        let side_w = reading.side_of(w)?;
        if side_v == VertexSide::Outside
            || side_w == VertexSide::Outside
            || (side_v != VertexSide::Inside && side_w != VertexSide::Inside)
        {
            continue;
        }

        // The edge cannot rise to the line from where its ends read.
        let (from, to) = {
            let mesh = reading.mesh.borrow();
            (mesh.position(v), mesh.position(w))
        };
        let value_v = reading.value_at(v)?;
        let value_w = reading.value_at(w)?;
        if field_bounds::greatest_along_edge(value_v, value_w, (to - from).length()) <= reading.snap
        {
            continue;
        }

        // Filed by how many SIZES the edge spans: the chord tolerance is the size over twenty.
        if FieldCounters::meter_on() {
            FieldCounters::asked_by_reaching(
                LevelLineRule::EdgeBump,
                (from - to).length() / (20.0 * chord_tolerance),
                nearest_to_the_line(chord_tolerance, value_v, value_w),
                farthest_from_the_line(chord_tolerance, value_v, value_w),
            );
        }

        let greatest = field.max_along_segment_at_least(
            from,
            to,
            chord_tolerance,
            reading.snap - chord_tolerance,
        )?;
        if greatest.value <= reading.snap {
            continue;
        }

        if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
            return Ok(BumpAnswer::Refused);
        }

        FieldCounters::fired(LevelLineRule::EdgeBump);

        // As with the dip: a peak within a snap of an end is refused by the mesh, nothing is split,
        // and the triangle settles with that pair of crossings unresolved - below the accuracy.
        let mut mesh = reading.mesh.borrow_mut();
        meter_would_thin_on_edge(
            LevelLineRule::EdgeBump,
            &mesh,
            v,
            w,
            greatest.point,
            chord_tolerance,
        );
        if !bisect_instead_of_thinning_on_edge(
            &mut mesh,
            triangle,
            v,
            w,
            greatest.point,
            chord_tolerance,
            pending,
            depth,
        )? {
            split_edge_and_enqueue(&mut mesh, v, w, greatest.point, pending, depth)?;
        }

        return Ok(BumpAnswer::Split);
    }

    Ok(BumpAnswer::Nothing)
}
