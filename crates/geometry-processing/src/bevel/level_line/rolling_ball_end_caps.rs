//! The cap the rolling ball cuts on an open chain's end face where the chain's two sides stand
//! apart there ([`CapShift`], read by `band_gap_setback_fields::cap_fields_at_the_size`).
//! `rolling_ball_setbacks` decides when an end needs one (today only an end face square to the
//! chain: on a leaning one the walk's end station reads its sides only on the chain's side of the
//! face, so the setbacks there are not the rails'); this file finds where each side's rail ends
//! and the cap that meets both, for a square end face or a leaning one.
//!
//! **Where each rail ends.** Each side's band is cut by that side's own field (the chain's points
//! with that side's setbacks, a union of balls), so its rail meets the end face exactly where that
//! field's zero crosses the side's edge with the end face: walked out from the end vertex along the
//! edges between the side's triangles and the end face's, the first point the field is no longer
//! inside ([`rail_end`]). On an end face square to the chain that is the point the side's distance
//! out in the plane square to the chain; on one that leans, the rail on the side the face leans
//! away from runs on along the chain to the face (`A'` stands the side's distance from the chain's
//! line), and on the other side it meets the face before the chain's end, where its field curls
//! round the end vertex's ball (`A'` stands the side's distance from the end vertex). The walk's
//! `EndFace` station is no stand-in: its contacts lie in the plane square to the chain at the end
//! vertex, or in the end face's plane only where the free centre would cross it (the T Handle's
//! web holds its centre 1.56 short of the plane, so the sphere side's contact lies that far from
//! the face's edge).
//!
//! **The cap.** The cap is the chain's own field (the larger side's setbacks) moved by `offset`
//! along `B' - A'` with every setback grown by `grow`, so it is still a union of balls: 1-Lipschitz,
//! so every bounded search, sample and chain parameter its group reads works as before (a move
//! changes no parameter). Read at a moved point the cap is `own(p - offset) - grow`, so it is zero
//! at both rails' ends where `own(A' - mu d) = own(B' - mu d)` (`d = B' - A'`, `offset = mu d`) and
//! `grow` is that common value. The difference runs from below zero to above it as `mu` runs from
//! far below to far above (each term grows like `|mu| |d|`), so `mu` is bracketed and halved to
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
//! band flows over) has no such point on its edge, and the pass keeps the level line.

use vector_math::Vector3;

use super::rolling_ball_setbacks::CapShift;
use super::triangle_edges::{self, StandingMap};
use super::ChainField;

/// One side of an open chain at one end: its triangles, the other side's, and its field at the
/// size.
pub(super) struct CapSide<'a> {
    /// The side's triangles.
    pub side: &'a [bool],
    /// The other side's triangles.
    pub other: &'a [bool],
    /// The side's own field at the size.
    pub field: &'a ChainField,
}

/// How many halvings [`cap_through`] and [`rail_end`] run at most; each stops sooner once its
/// interval no longer shrinks in floating point.
const HALVINGS: usize = 200;

/// How many times [`cap_through`] doubles its bracket at most before it gives up.
const DOUBLINGS: usize = 64;

/// Where `side`'s rail meets the end face, walked out from the end vertex `end` along the side's
/// edges with the end face (an edge of a triangle of the side whose other triangle is of neither
/// side): the first point the side's field is no longer inside, found by halving the edge it
/// crosses on. `Err` with why where the edges run out first or a reading fails.
pub(super) fn rail_end(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    end: i32,
    side: &CapSide<'_>,
) -> Result<Vector3, String> {
    let value = |p: Vector3| -> Result<f64, String> {
        side.field
            .evaluate(p)
            .map(|sample| sample.value)
            .map_err(|error| format!("its rail's end could not be read: {error:?}"))
    };
    let on_the_end_face = |a: i32, b: i32| {
        standing
            .get(&triangle_edges::key(a, b))
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .any(|&u| !side.side[u as usize] && !side.other[u as usize])
    };
    // The next vertex along the side's edge with the end face from `at`, never back to `from`;
    // the lowest triangle first, so the walk is the same on every run.
    let next = |at: i32, from: i32| -> Option<i32> {
        indices.iter().enumerate().find_map(|(t, &corners)| {
            if !side.side[t] {
                return None;
            }
            triangle_edges::corners(corners)
                .into_iter()
                .find_map(|(a, b)| {
                    let to = if a == at {
                        b
                    } else if b == at {
                        a
                    } else {
                        return None;
                    };
                    (to != from && on_the_end_face(a, b)).then_some(to)
                })
        })
    };

    let (mut from, mut at) = (-1, end);
    let mut inside = value(positions[end as usize])?;
    for _ in 0..indices.len() {
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
