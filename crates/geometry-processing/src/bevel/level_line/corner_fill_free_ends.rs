//! A corner loop's pieces split round a chain's FREE end (the piece type is in `corner_fill.rs`):
//! the rails kept past an end ([`rails_past_an_end`]), a closed
//! loop's piece turned to start inside an end arc ([`start_inside_an_end_arc`]), a piece's slice
//! ([`slice`]), and the split round a free end ([`split_round_a_free_end`]), which at a STOPPED
//! end reads the stretch round it by the end plane and ends each rail at its corner in that plane.

use vector_math::ieee_math;

use std::rc::Rc;

use super::corner_fill::same;
use super::stopped_end_fill::stopped_plane;
use super::{ChainField, CornerFillPiece, LevelLineMesh};

/// The rails of `piece` with its run(s) at one free end's
/// `parameter` taken off. A piece wholly at the end (or not at it at all) has no rail to keep and
/// comes back as it was.
pub fn rails_past_an_end(
    piece: &CornerFillPiece,
    parameter: f64,
    tolerance: f64,
) -> Vec<CornerFillPiece> {
    rails_past_an_end_where(piece, |i| {
        (piece.parameters[i] - parameter).abs() <= tolerance
    })
}

/// The rails of `piece` with its run(s) round one free end taken off, where `round_the_end` names
/// the piece's vertices (by index) that stand round that end; [`rails_past_an_end`] reads them by
/// the end's parameter, a STOPPED end also by its plane ([`split_round_a_free_end`]).
pub fn rails_past_an_end_where(
    piece: &CornerFillPiece,
    round_the_end: impl Fn(usize) -> bool,
) -> Vec<CornerFillPiece> {
    let n = piece.vertices.len();
    let at_end: Vec<usize> = (0..n).filter(|&i| round_the_end(i)).collect();
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

/// Turns a one-owner closed loop's piece to start INSIDE an
/// arc round one of its open chain's ends (a vertex at an end's parameter whose predecessor is
/// too), so no rail runs across the loop's seam and the free-end split reads the arcs as runs at
/// the piece's ends or inside it. False, the piece untouched, where no such vertex exists.
pub fn start_inside_an_end_arc(field: &ChainField, piece: &mut CornerFillPiece) -> bool {
    let n = piece.vertices.len();
    let tolerance = 1e-9 * ieee_math::max_f64(1.0, field.length);
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

/// The vertices of `piece` from index `first` to `last`, inclusive, with
/// the same owner.
pub fn slice(piece: &CornerFillPiece, first: usize, last: usize) -> CornerFillPiece {
    CornerFillPiece {
        owner: piece.owner,
        vertices: piece.vertices[first..=last].to_vec(),
        parameters: piece.parameters[first..=last].to_vec(),
    }
}

/// A piece's stretches ROUND a free end of its open chain taken
/// off, leaving only its rails ([`rails_past_an_end`] at each end no other chain's point stands
/// on). They happen where no end region of the chain's own was carved: the end face is a side
/// region of the pick. Each rail keeps one vertex at the end's parameter; the arc beyond it is left
/// open for the cap, as a carved end region's run is. Every other piece - a closed chain's, or one
/// with no free end - comes back as it was.
///
/// At a STOPPED free end (`stopped_end_clip.rs`) the stretch round the end is the end plane's
/// trace: every vertex standing in the plane by [`ChainField::on_a_stopped_plane`] is round the
/// end, whatever its parameter. Where the plane leans off square to the chain, the trace's
/// vertices read parameters short of the end (the rail's corner in the plane reads where its
/// square section crosses the chain), and only the stretch nearest the end vertex reads the end's
/// own; read by parameter alone, the rail ran on along the plane to that stretch and its last row
/// was drawn from there, a section narrower than the round's (the rim facet leaning 4.5 degrees:
/// 0.73 of the profile's reach on the top face). The rail keeps its first vertex in the plane, its
/// corner, which reads the end's own parameter: the strip runs the other rail on to the end
/// against it, and the row between the two corners is the round's section in the plane
/// (`fillet_arc_rows::build_in_a_stopped_plane`).
pub fn split_round_a_free_end(
    mesh: &LevelLineMesh,
    chains: &[Rc<ChainField>],
    piece: &CornerFillPiece,
) -> Vec<CornerFillPiece> {
    let field = &chains[piece.owner as usize];
    if field.is_closed {
        return vec![piece.clone()];
    }

    let tolerance = 1e-9 * ieee_math::max_f64(1.0, field.length);
    let mut result = vec![piece.clone()];
    let ends = [
        (field.points[0], 0.0, stopped_plane(field, false)),
        (
            field.points[field.points.len() - 1],
            field.length,
            stopped_plane(field, true),
        ),
    ];
    for (point, parameter, plane) in ends {
        let free = !chains
            .iter()
            .filter(|c| !Rc::ptr_eq(c, field))
            .any(|c| c.points.iter().any(|&p| same(p, point)));
        if !free {
            continue;
        }

        let in_the_plane = |vertex: i32| {
            plane.is_some_and(|(origin, normal, tilt)| {
                ChainField::on_a_stopped_plane(origin, normal, tilt, mesh.position(vertex))
            })
        };
        result = result
            .iter()
            .flat_map(|p| {
                rails_past_an_end_where(p, |i| {
                    (p.parameters[i] - parameter).abs() <= tolerance || in_the_plane(p.vertices[i])
                })
            })
            .collect();
        if plane.is_none() {
            continue;
        }

        // The corner stands for the end: its row is the section in the plane, drawn about the end
        // vertex, so it reads the end's own parameter and the strip runs the other rail on to the
        // end against it.
        for rail in &mut result {
            let last = rail.vertices.len() - 1;
            for k in [0, last] {
                if in_the_plane(rail.vertices[k]) {
                    rail.parameters[k] = parameter;
                }
            }
        }
    }

    result
}
