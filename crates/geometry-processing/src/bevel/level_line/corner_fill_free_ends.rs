//! A corner loop's pieces split round a chain's FREE end. Ports the piece helpers of
//! `Bevel/LevelLine/CornerFill.FreeEnds.cs` (a partial of the C# static class `CornerFill`, whose
//! piece type is in `corner_fill.rs`): the rails kept past an end ([`rails_past_an_end`]), a closed
//! loop's piece turned to start inside an end arc ([`start_inside_an_end_arc`]) and a piece's slice
//! ([`slice`]); the split over a group lands with later packages.

use vector_math::cs_math;

use super::{ChainField, CornerFillPiece};

/// C# `CornerFill.RailsPastAnEnd`: the rails of `piece` with its run(s) at one free end's
/// `parameter` taken off. A piece wholly at the end (or not at it at all) has no rail to keep and
/// comes back as it was.
pub fn rails_past_an_end(
    piece: &CornerFillPiece,
    parameter: f64,
    tolerance: f64,
) -> Vec<CornerFillPiece> {
    let n = piece.vertices.len();
    let at_end: Vec<usize> = (0..n)
        .filter(|&i| (piece.parameters[i] - parameter).abs() <= tolerance)
        .collect();
    if at_end.is_empty() || at_end.len() == n {
        return vec![piece.clone()];
    }

    // A run at the piece's start or end: keep only its inner vertex, where the rail reaches the
    // end.
    let mut from = 0;
    while from + 1 < n && at_end.contains(&from) && at_end.contains(&(from + 1)) {
        from += 1;
    }

    let mut to = n - 1;
    while to > from && at_end.contains(&to) && at_end.contains(&(to - 1)) {
        to -= 1;
    }

    let inner: Vec<usize> = at_end
        .iter()
        .copied()
        .filter(|&i| i > from && i < to)
        .collect();
    if inner.is_empty() {
        if from == 0 && to == n - 1 {
            return vec![piece.clone()];
        }

        return vec![slice(piece, from, to)];
    }

    // A run inside: the piece goes down one rail, round the end, and back up the other.
    vec![
        slice(piece, from, inner[0]),
        slice(piece, inner[inner.len() - 1], to),
    ]
}

/// C# `CornerFill.StartInsideAnEndArc`: turns a one-owner closed loop's piece to start INSIDE an
/// arc round one of its open chain's ends (a vertex at an end's parameter whose predecessor is
/// too), so no rail runs across the loop's seam and the free-end split reads the arcs as runs at
/// the piece's ends or inside it. False, the piece untouched, where no such vertex exists.
pub fn start_inside_an_end_arc(field: &ChainField, piece: &mut CornerFillPiece) -> bool {
    let n = piece.vertices.len();
    let tolerance = 1e-9 * cs_math::max_f64(1.0, field.length);
    let at_end = |i: usize, parameter: f64| (piece.parameters[i] - parameter).abs() <= tolerance;
    let start = (0..n).find(|&i| {
        (at_end(i, 0.0) && at_end((i + n - 1) % n, 0.0))
            || (at_end(i, field.length) && at_end((i + n - 1) % n, field.length))
    });
    let Some(start) = start else {
        return false;
    };

    piece.vertices.rotate_left(start);
    piece.parameters.rotate_left(start);
    true
}

/// C# `CornerFill.Slice`: the vertices of `piece` from index `first` to `last`, inclusive, with
/// the same owner.
pub fn slice(piece: &CornerFillPiece, first: usize, last: usize) -> CornerFillPiece {
    CornerFillPiece {
        owner: piece.owner,
        vertices: piece.vertices[first..=last].to_vec(),
        parameters: piece.parameters[first..=last].to_vec(),
    }
}
