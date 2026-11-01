//! The cap the rolling ball cuts on an open chain's end face where the chain's two sides stand
//! apart there ([`CapShift`], read by `band_gap_setback_fields::cap_fields_at_the_size`).
//! `rolling_ball_setbacks` decides when an end needs one; this file finds where each side's rail
//! ends and the cap that meets both, for an end face square to the chain or leaning off it.
//!
//! **Where each rail ends.** Each side's band is cut by that side's own field (the band's own
//! field from the chain's points with that side's setbacks, a union of balls), so its rail meets
//! the end face exactly where that field's zero crosses the side's edge with the end face: walked
//! out from the end vertex along the edges between the side's triangles and the end face's own
//! ([`end_face_triangles`]: the triangles of neither side round the end vertex, flooded across
//! the face and no further, so the walk never runs on onto a top face or a third face), the first
//! point the field is no longer inside ([`rail_end`]). On an end face square to the chain that is
//! the point the side's distance out in the plane square to the chain; on one that leans, the
//! rail on the side the face leans away from runs on along the chain to the face (`A'` stands the
//! side's distance from the chain's line), and on the other side it meets the face before the
//! chain's end, where its field curls round the end vertex's ball (`A'` stands the side's
//! distance from the end vertex). The walk reads each side as extended past the end face
//! (`rolling_ball::side_extension`), so on a leaning face its end station's setbacks are the
//! rails' too.
//!
//! **The cap.** The cap is the chain's own field (the larger side's setbacks) moved by `offset`
//! along `B' - A'` with every setback grown by `grow`, so it is still a union of balls: 1-Lipschitz,
//! so every bounded search, sample and chain parameter its group reads works as before (a move
//! changes no parameter). Read at a moved point the cap is `own(p - offset) - grow`, so it is zero
//! at both rails' ends where `own(A' - mu d) = own(B' - mu d)` (`d = B' - A'`, `offset = mu d`) and
//! `grow` is that common value. The difference runs from below zero to above it as `mu` runs from
//! far below to far above (it tends to `-|d|` and `+|d|`: far along `d`, the two points' distances
//! from the chain differ by the gap between them), so `mu` is bracketed and halved to
//! the last bit ([`cap_through`]). The cut on the end face is then the moved field's own trace on
//! it: smooth between the walls (the end ball's circle running on, tangent where they meet, into
//! the moved chain's trace where the face leans, the same shape the level line's own cap has on a
//! leaning face), through `A'` and `B'`, so no notch opens along either wall. On a square end with
//! the setbacks even along the end it is the circle through `A'` and `B'` centred on their
//! bisector nearest the end vertex. Both points lie on the end face's edges, so on a flat end face
//! the move runs in the face's plane. Where the sides agree there is no cap of its own and the end
//! reads the chain's own field as the level line does.
//!
//! An end face that ends before a side's rail does (narrower than that side's distance, or one the
//! band flows over) has no such point on its edge, and the ball refuses the pass.

use std::collections::HashMap;

use vector_math::Vector3;

use super::rolling_ball_setbacks::CapShift;
use super::triangle_edges::{self, StandingMap};
use super::ChainField;

/// One side of an open chain at one end: its triangles and its field at the size.
pub(super) struct CapSide<'a> {
    /// The side's triangles.
    pub side: &'a [bool],
    /// The side's own field at the size.
    pub field: &'a ChainField,
}

/// How many halvings [`cap_through`] and [`rail_end`] run at most; each stops sooner once its
/// interval no longer shrinks in floating point.
const HALVINGS: usize = 200;

/// How many times [`cap_through`] doubles its bracket at most before it gives up.
const DOUBLINGS: usize = 64;

/// The end face at chain end vertex `end`: the triangles of neither side round the vertex, flooded
/// on over every edge `joins(from, to, a, b)` lets the face cross (triangle `from` to triangle `to`
/// over edge `a`-`b`), never into either side. A rail's end is looked for only along this face's
/// own edges with a side, so a walk along the side's edge never runs on onto a top face or a third
/// face the end face meets.
pub(super) fn end_face_triangles(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    end: i32,
    (in_a, in_b): (&[bool], &[bool]),
    joins: &dyn Fn(usize, usize, i32, i32) -> bool,
) -> Vec<bool> {
    let neither = |t: usize| !in_a[t] && !in_b[t];
    let mut on_face = vec![false; indices.len()];
    let mut stack: Vec<usize> = Vec::new();
    for (t, &(a, b, c)) in indices.iter().enumerate() {
        if (a == end || b == end || c == end) && neither(t) {
            on_face[t] = true;
            stack.push(t);
        }
    }
    while let Some(t) = stack.pop() {
        for (a, b) in triangle_edges::corners(indices[t]) {
            let across = standing
                .get(&triangle_edges::key(a, b))
                .map_or(&[][..], Vec::as_slice);
            for &u in across {
                // Mesh triangle indices are non-negative.
                let u = u as usize;
                if u != t && !on_face[u] && neither(u) && joins(t, u, a, b) {
                    on_face[u] = true;
                    stack.push(u);
                }
            }
        }
    }

    on_face
}

/// Where `side`'s rail meets the end face, walked out from the end vertex `end` along the side's
/// edges with the end face (`end_face`, [`end_face_triangles`]: an edge of a triangle of the side
/// whose other triangle is the end face's): the first point the side's field is no longer inside,
/// found by halving the edge it crosses on. `Err` with why where those edges run out first or a
/// reading fails.
pub(super) fn rail_end(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    end: i32,
    end_face: &[bool],
    side: &CapSide<'_>,
) -> Result<Vector3, String> {
    let value = |p: Vector3| -> Result<f64, String> {
        side.field
            .evaluate(p)
            .map(|sample| sample.value)
            .map_err(|error| format!("its rail's end could not be read: {error:?}"))
    };
    // The side's edges with the end face, as each vertex's neighbours along them, lowest first so
    // the walk is the same on every run; gathered once from the end face's own triangles.
    let mut along: HashMap<i32, Vec<i32>> = HashMap::new();
    for (t, &corners) in indices.iter().enumerate() {
        if !end_face[t] {
            continue;
        }
        for (a, b) in triangle_edges::corners(corners) {
            let on_the_side = standing
                .get(&triangle_edges::key(a, b))
                .map_or(&[][..], Vec::as_slice)
                .iter()
                .any(|&u| side.side[u as usize]);
            if on_the_side {
                along.entry(a).or_default().push(b);
                along.entry(b).or_default().push(a);
            }
        }
    }
    for next in along.values_mut() {
        next.sort_unstable();
        next.dedup();
    }
    let next = |at: i32, from: i32| -> Option<i32> {
        along
            .get(&at)
            .and_then(|next| next.iter().copied().find(|&to| to != from))
    };

    let (mut from, mut at) = (-1, end);
    let mut inside = value(positions[end as usize])?;
    for _ in 0..=along.len() {
        let Some(to) = next(at, from) else { break };
        let (start, stop) = (positions[at as usize], positions[to as usize]);
        let reached = value(stop)?;
        if inside < 0.0 && reached >= 0.0 {
            let (mut lo, mut hi) = (0.0f64, 1.0f64);
            for _ in 0..HALVINGS {
                let mid = 0.5 * (lo + hi);
                if mid <= lo || mid >= hi {
                    break;
                }
                if value(start + ((stop - start) * mid))? < 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            return Ok(start + ((stop - start) * hi));
        }
        inside = reached;
        (from, at) = (at, to);
        if at == end {
            break;
        }
    }
    Err(
        "its rail does not meet the end face along the side's own edge with it (the end face \
         is narrower than the rail's distance, or the band flows over it)"
            .to_string(),
    )
}

/// The cap through the left rail's end `a` and the right's `b` (see the file header), from the
/// chain's own field `own` at the size: the move and the growth, at the size. `Err` where `a` and
/// `b` are one point, no bracket is found or a reading fails.
pub(super) fn cap_through(own: &ChainField, a: Vector3, b: Vector3) -> Result<CapShift, String> {
    let d = b - a;
    let across = d.length_squared();
    if across.is_nan() || across <= 0.0 {
        return Err("both rails end at one point".to_string());
    }
    let value = |p: Vector3| -> Result<f64, String> {
        own.evaluate(p)
            .map(|sample| sample.value)
            .map_err(|error| format!("its cap could not be read: {error:?}"))
    };
    let apart =
        |mu: f64| -> Result<f64, String> { Ok(value(a - (d * mu))? - value(b - (d * mu))?) };

    // Bracket the root about the circle's answer on a square end with even setbacks: the point of
    // the bisector of a and b nearest the end vertex, read off the field's nearest chain point.
    let e = own
        .evaluate(a)
        .map_err(|error| format!("{error:?}"))?
        .nearest;
    let guess = ((b - e).length_squared() - (a - e).length_squared()) / (2.0 * across);
    let (mut lo, mut hi, mut step) = (guess - 0.5, guess + 0.5, 0.5);
    let mut bracketed = false;
    for _ in 0..DOUBLINGS {
        if apart(lo)? <= 0.0 && apart(hi)? >= 0.0 {
            bracketed = true;
            break;
        }
        step *= 2.0;
        (lo, hi) = (guess - step, guess + step);
    }
    if !bracketed {
        return Err("its cap has no circle through both rails' ends".to_string());
    }
    for _ in 0..HALVINGS {
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if apart(mid)? < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let mu = 0.5 * (lo + hi);
    let offset = d * mu;
    let grow = value(a - offset)?;
    if !grow.is_finite() {
        return Err("its cap's growth is not a number".to_string());
    }
    Ok(CapShift { offset, grow })
}
