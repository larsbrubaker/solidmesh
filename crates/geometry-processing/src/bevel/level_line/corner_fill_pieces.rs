//! The corner fill's loop pieces. Ports `Bevel/LevelLine/CornerFill.Pieces.cs` (a partial of the C#
//! static class `CornerFill`; the piece type lives in `corner_fill.rs`): so far the trim of a
//! piece's turn-back at an end ([`trim_the_turn_back`]); the rest of the partial lands with later
//! packages.

use vector_math::cs_math;

use super::{ChainField, CornerFillPiece};

/// Trims a piece's turn-back at an end: only a turn-back that comes off the chain's END vertex is
/// the wrap - a rail that turns back in the middle of its run is a strip that cannot be zipped,
/// and trimmed it would hand a whole round to the hole filler, so that one stays refused by name.
/// A single stop on an end plane after a run past that same end is dropped too.
pub fn trim_the_turn_back(field: &ChainField, piece: &mut CornerFillPiece) {
    let tie = 1e-9 * cs_math::max_f64(1.0, field.length);
    let length = field.length;
    let climbs = piece.parameters[piece.parameters.len() - 1] > piece.parameters[0];
    let back = |from: f64, to: f64| {
        if climbs {
            to < from - tie
        } else {
            to > from + tie
        }
    };
    // AT an end, not past one: a reading carried out beyond an end (negative, or over the length)
    // is a rail already running round it, not the clamped run the wrap comes off.
    let at_an_end = |t: f64| t.abs() <= tie || (t - length).abs() <= tie;
    // The stop on an end plane after a run past that same end: the end reading at one end of the
    // piece, the reading beside it carried out beyond the end it names.
    let stops_past_the_end = |stop: f64, beside: f64| {
        (stop.abs() <= tie && beside < -tie)
            || ((stop - length).abs() <= tie && beside > length + tie)
    };

    let p = &mut piece.parameters;
    let mut head = 0;
    while p.len() - head > 2 && back(p[head], p[head + 1]) {
        head += 1;
    }

    if head == 1 && stops_past_the_end(p[0], p[1]) {
        piece.vertices.remove(0);
        p.remove(0);
    } else if head > 0 && at_an_end(p[head]) {
        piece.vertices.drain(0..head);
        p.drain(0..head);
    }

    let mut tail = 0;
    while p.len() - tail > 2 && back(p[p.len() - 2 - tail], p[p.len() - 1 - tail]) {
        tail += 1;
    }

    let count = p.len();
    if tail == 1 && stops_past_the_end(p[count - 1], p[count - 2]) {
        piece.vertices.pop();
        p.pop();
    } else if tail > 0 && at_an_end(p[count - 1 - tail]) {
        let vertices = piece.vertices.len();
        piece.vertices.truncate(vertices - tail);
        p.truncate(count - tail);
    }
}
