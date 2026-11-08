//! Closest approach between one chain segment and a piece of the mesh, in closed form. The closest
//! points it evaluates are [`ChainField::closest_on_segment`] and
//! [`ChainField::closest_on_triangle`].
//!
//! Where a chain segment asks the SAME setback at both its ends - every chamfer, and every fillet
//! between two flat faces - the least of `|P - C(u)| - setback` over a piece is just the closest
//! approach between the piece and the chain segment, less that one number. No search is needed,
//! and the convex search in the chain field is left for the only case that still needs it: a
//! segment whose two ends ask different setbacks.
//!
//! **How the exact answer is reached.** `f(u) = dist(C(u), piece)` is convex (a distance to a
//! convex set, composed with an affine map), so its minimum over `[0, 1]` sits at an end, or at an
//! interior point whose attaining point of the piece is on the piece's own boundary - one of the
//! triangle's edges - or, for a triangle, where the chain crosses its plane inside it. Every one of
//! those is a closed form, so a handful of candidate parameters is certain to contain the
//! minimiser; each is then EVALUATED exactly and the least taken. Extra candidates are harmless,
//! which is why the tests for "is the crossing inside the triangle" and "is this the near edge"
//! are not written: they would only remove work that costs one closest-point evaluation.
//!
//! **Ties.** Candidates are taken in ascending chain parameter and only a strictly smaller
//! distance displaces the best, so a minimum attained over a whole interval (two parallel
//! segments) answers at its LOWEST parameter - the same rule the rest of the field reads ties by.

use solidmesh_math::Vector3;

use super::ChainField;

/// A tie between two candidate parameters is the same number reached by different arithmetic, so
/// "the same" has to allow a few bits of drift, scaled to the distance being compared.
const TIE_RELATIVE: f64 = 1e-12;

/// The closest approach between the mesh segment `pa`-`pb` and the chain segment `ca`-`cb`, with
/// the attaining point on the MESH segment (returned beside the distance). Parallel, touching,
/// crossing and zero-length segments all fall out of the same arithmetic.
pub fn segment_to_segment(pa: Vector3, pb: Vector3, ca: Vector3, cb: Vector3) -> (f64, Vector3) {
    let mut candidates = [0.0, 1.0, chain_parameter_of_approach(pa, pb, ca, cb)];
    least_over_candidates(&mut candidates, ca, cb, pa, pb, Vector3::ZERO, false)
}

/// The closest approach between the triangle `a b c` - interior included - and the chain segment,
/// with the attaining point on the TRIANGLE (returned beside the distance). A chain segment that
/// pierces the triangle answers zero, through the plane-crossing candidate.
pub fn triangle_to_segment(
    a: Vector3,
    b: Vector3,
    c: Vector3,
    ca: Vector3,
    cb: Vector3,
) -> (f64, Vector3) {
    let mut candidates = [
        0.0,
        1.0,
        chain_parameter_of_approach(a, b, ca, cb),
        chain_parameter_of_approach(b, c, ca, cb),
        chain_parameter_of_approach(c, a, ca, cb),
        plane_crossing(a, b, c, ca, cb),
    ];
    least_over_candidates(&mut candidates, ca, cb, a, b, c, true)
}

/// The parameter on the CHAIN segment of the closest approach between two segments - the standard
/// clamped solution, with the parallel and zero-length cases named.
fn chain_parameter_of_approach(pa: Vector3, pb: Vector3, ca: Vector3, cb: Vector3) -> f64 {
    let d1 = pb - pa;
    let d2 = cb - ca;
    let r = pa - ca;
    let a = d1.length_squared();
    let e = d2.length_squared();
    let f = d2.dot(r);

    if e <= 0.0 {
        // The chain segment is a point: every parameter names the same point, so the lowest wins.
        return 0.0;
    }

    if a <= 0.0 {
        // The mesh piece is a point: project it onto the chain segment.
        return clamp01(f / e);
    }

    let c = d1.dot(r);
    let b = d1.dot(d2);
    let denominator = (a * e) - (b * b);

    // Parallel (the determinant vanishing): every s does as well as any other, and taking s = 0
    // leaves the candidate sweep to settle the tie at the lowest chain parameter.
    let s = if denominator > 0.0 {
        clamp01(((b * f) - (c * e)) / denominator)
    } else {
        0.0
    };

    clamp01(((b * s) + f) / e)
}

/// Where the chain segment crosses the triangle's plane, or 0 when it is parallel to it - a
/// candidate that costs one evaluation to discard, rather than an inside test that could get the
/// question wrong.
fn plane_crossing(a: Vector3, b: Vector3, c: Vector3, ca: Vector3, cb: Vector3) -> f64 {
    let normal = (b - a).cross(c - a);
    if normal.length_squared() <= 0.0 {
        return 0.0;
    }

    let from = (ca - a).dot(normal);
    let to = (cb - a).dot(normal);
    let span = from - to;
    if span == 0.0 {
        return 0.0;
    }

    clamp01(from / span)
}

/// Evaluates every candidate parameter exactly, in ascending order, and keeps the least - a tie
/// going to the lower parameter - with the point of the piece attaining it. At most six numbers,
/// so the ordering is an insertion pass.
fn least_over_candidates(
    candidates: &mut [f64],
    ca: Vector3,
    cb: Vector3,
    piece_a: Vector3,
    piece_b: Vector3,
    piece_c: Vector3,
    is_triangle: bool,
) -> (f64, Vector3) {
    for i in 1..candidates.len() {
        let value = candidates[i];
        let mut j = i as i32 - 1;
        while j >= 0 && candidates[j as usize] > value {
            candidates[(j + 1) as usize] = candidates[j as usize];
            j -= 1;
        }

        candidates[(j + 1) as usize] = value;
    }

    let mut best = f64::INFINITY;
    let mut on_piece = piece_a;
    for &candidate in candidates.iter() {
        let chain_point = ca + ((cb - ca) * candidate);
        let here = if is_triangle {
            ChainField::closest_on_triangle(chain_point, piece_a, piece_b, piece_c)
        } else {
            ChainField::closest_on_segment(chain_point, piece_a, piece_b)
        };
        let distance = (here - chain_point).length();
        if best == f64::INFINITY || distance < best - (TIE_RELATIVE * (1.0 + best)) {
            best = distance;
            on_piece = here;
        }
    }

    (best, on_piece)
}

fn clamp01(value: f64) -> f64 {
    // NaN passes through, so a broken reading stays visible downstream.
    value.clamp(0.0, 1.0)
}
