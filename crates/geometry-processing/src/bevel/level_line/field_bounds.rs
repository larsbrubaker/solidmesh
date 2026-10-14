//! What the values already in hand say a piece of the mesh CANNOT do, so an exact question is never
//! asked where its answer could not have mattered.
//!
//! The field is exactly 1-Lipschitz - it is the signed distance to the union of the chain's balls -
//! so from a point p reading v, any point q has `v - |q - p| <= value(q) <= v + |q - p|`, whatever
//! the setback does along the chain. It is the same argument the chain field's own triangle bound
//! is built on, hoisted out of the search so the CALL is skipped rather than merely cut short: the
//! cached per-vertex values cost nothing to look at, while one bounded search costs about a hundred
//! evaluations.
//!
//! Every use is one-sided. A refinement rule fires only when the exact answer crosses a threshold,
//! and the bound is consulted only to prove it cannot, so the questions skipped are exactly the ones
//! whose answers could not have changed anything and the refinement's result is identical.

use vector_math::{cs_math, Vector3};

/// The greatest value the field can take anywhere on an edge of length L whose ends read va, vb.
/// From either end the value rises at most one per unit of length, so along the edge it lies under
/// both cones; they cross at `(va + vb + L) / 2`.
pub fn greatest_along_edge(va: f64, vb: f64, length: f64) -> f64 {
    0.5 * (va + vb + length)
}

/// The least value the field can take anywhere on an edge of length L whose ends read va, vb.
pub fn least_along_edge(va: f64, vb: f64, length: f64) -> f64 {
    0.5 * (va + vb - length)
}

/// The length a CROSSED edge must exceed before it can hide a second pair of crossings past its
/// first root - below it the mixed edge rule's root search and two bounded searches are not worth
/// asking for.
///
/// An edge running from an inside end reading `inside` to an outside end reading `outside` spends
/// at least `|inside|` of its length reaching the root and at least `|outside|` leaving it, so
/// neither piece the root leaves is longer than the whole length less the other end's value. Put
/// that into [`greatest_along_edge`] against a root reading zero: a peak over `+tolerance` on the
/// inside piece needs `L - |outside| > |inside| + 2*tolerance`, and the mirror gives the same
/// number. An ordinary crossed edge of a fine mesh is barely longer than `|inside| + |outside|` and
/// is skipped outright.
pub fn length_to_hide_a_pair(inside: f64, outside: f64, tolerance: f64) -> f64 {
    inside.abs() + outside.abs() + (2.0 * tolerance)
}

/// The greatest value the field can take anywhere on a triangle whose corners `a`, `b`, `c` read
/// `va`, `vb`, `vc`. The farthest point of a triangle from one of its corners is another of its
/// corners, so a corner's cone covers the whole triangle once given that reach; the tightest of the
/// three corners is the bound.
pub fn greatest_over_triangle(
    va: f64,
    vb: f64,
    vc: f64,
    a: Vector3,
    b: Vector3,
    c: Vector3,
) -> f64 {
    let mut bound = va + reach(a, b, c);
    bound = cs_math::min_f64(bound, vb + reach(b, a, c));
    bound = cs_math::min_f64(bound, vc + reach(c, a, b));
    bound
}

/// The least value the field can take anywhere on a triangle whose corners read va, vb, vc.
pub fn least_over_triangle(va: f64, vb: f64, vc: f64, a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let mut bound = va - reach(a, b, c);
    bound = cs_math::max_f64(bound, vb - reach(b, a, c));
    bound = cs_math::max_f64(bound, vc - reach(c, a, b));
    bound
}

/// How far a triangle reaches from one of its corners: the longer of the two edges leaving it.
fn reach(from: Vector3, other: Vector3, third: Vector3) -> f64 {
    cs_math::max_f64((other - from).length(), (third - from).length())
}
