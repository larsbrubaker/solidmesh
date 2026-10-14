//! The corner fill's loop pieces (the piece type lives in `corner_fill.rs`): the trim of a
//! piece's turn-back at an end ([`trim_the_turn_back`]) and of its ends off the line
//! ([`trim_off_the_line`]) and a one-owner loop split at another chain's tie run
//! ([`tie_split`]).

use vector_math::cs_math;

use std::rc::Rc;

use super::{ChainField, CornerFillPiece, GapLoop, LevelLineMesh, WorkError};

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

/// Drops a piece's end vertices that stand OFF its chain's line
/// (the chain's own field reads above a tie there): the face piece where two chains of different
/// setback meet at a setback step, or a run-out's run, which belongs to no rail - kept boundary the
/// one corner hole runs along. A rail keeps at least two vertices.
pub fn trim_off_the_line(
    mesh: &LevelLineMesh,
    field: &ChainField,
    piece: &mut CornerFillPiece,
) -> Result<(), WorkError> {
    let tie = 1e-9 * cs_math::max_f64(1.0, field.length);
    let off = |vertex: i32| -> Result<bool, WorkError> {
        Ok(field.evaluate(mesh.position(vertex))?.value > tie)
    };
    while piece.vertices.len() > 2 && off(piece.vertices[0])? {
        piece.vertices.remove(0);
        piece.parameters.remove(0);
    }

    while piece.vertices.len() > 2 && off(piece.vertices[piece.vertices.len() - 1])? {
        piece.vertices.pop();
        piece.parameters.pop();
    }

    Ok(())
}

/// A closed loop the walk handed wholly to one OPEN chain, where one other
/// chain's rail is nothing but ties (rim 0-1-2-3 with edges 1-5 and 2-6: the bottom face's line is
/// four corner-tie vertices, and the walk gives all four to the lower index). Split into the
/// owner's piece - from the end of the other chain's tie run round to its start - and the other
/// chain's piece, its tie run; the two share their end vertices, as tie vertices belong to both
/// rails. Left whole, the other chain had no rail on this side and its whole band fell to one hole
/// across both corners. `None` where the ties name no other chain, or more than one, or it has more
/// than one run.
pub fn tie_split(
    mesh: &LevelLineMesh,
    chains: &[Rc<ChainField>],
    gap_loop: &GapLoop,
    owner: i32,
) -> Result<Option<[CornerFillPiece; 2]>, WorkError> {
    let Some(ties) = &gap_loop.vertex_ties else {
        return Ok(None);
    };

    // Every other tie once, in first-met order, so the result is the same on every run.
    let mut others: Vec<i32> = Vec::new();
    for &k in ties.iter().flatten() {
        if k != owner && !others.contains(&k) {
            others.push(k);
        }
    }

    if others.len() != 1 {
        return Ok(None);
    }

    let k = others[0];
    let n = gap_loop.vertices.len();
    let starts: Vec<usize> = (0..n)
        .filter(|&i| ties[i].contains(&k) && !ties[(i + n - 1) % n].contains(&k))
        .collect();
    if starts.len() != 1 {
        return Ok(None);
    }

    let mut tied = CornerFillPiece {
        owner: k,
        ..CornerFillPiece::default()
    };
    let mut end = starts[0];
    let mut i = starts[0];
    while ties[i % n].contains(&k) && tied.vertices.len() < n {
        let vertex = gap_loop.vertices[i % n];
        tied.vertices.push(vertex);
        tied.parameters.push(
            chains[k as usize]
                .evaluate(mesh.position(vertex))?
                .parameter,
        );
        end = i % n;
        i += 1;
    }

    if tied.vertices.len() < 2 || tied.vertices.len() >= n {
        return Ok(None);
    }

    let mut own = CornerFillPiece {
        owner,
        ..CornerFillPiece::default()
    };
    for step in 0..=(n - tied.vertices.len() + 1) {
        let i = (end + step) % n;
        own.vertices.push(gap_loop.vertices[i]);
        own.parameters.push(gap_loop.parameters[i]);
    }

    Ok(Some([own, tied]))
}
