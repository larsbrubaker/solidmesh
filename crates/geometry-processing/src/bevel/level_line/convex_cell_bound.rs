//! An upper bound on a 1-Lipschitz union-of-balls field over one cell of a bounded search, read
//! from the CHAIN POINTS the cell's own corner samples already named rather than from the
//! Lipschitz reach. Reads [`FieldSample`]s; the chain and pick fields' ridge searches call it.
//!
//! The field is `f(q) = min over chain points c of (|q - c| - s(c))` (see `ChainField`). Fix ONE
//! chain point c. Then `g_c(q) = |q - c| - s(c)` is CONVEX in q (a norm of an affine map less a
//! constant) and `f <= g_c` everywhere, since f is a minimum over a set c belongs to. A convex
//! function on a simplex attains its greatest value at a vertex, so for a cell with corners v:
//!
//! ```text
//! max over the cell of f <= max over the cell of g_c = max over the corners v of (|v - c| - s(c))
//! ```
//!
//! and therefore, for ANY finite set P of chain points,
//! `max over the cell of f <= min over c in P of ( max over the corners v of (|v - c| - s(c)) )`.
//! Each extra c only sharpens it. P here is the minimising chain point of each corner - already in
//! hand from the corner's own sample, which carries both that point ([`FieldSample::nearest`]) and
//! the setback AT it (`distance - value`, exact whether the setback is uniform or varies along a
//! segment).
//!
//! It is never looser than the Lipschitz bound the search already had. From corner i, with c_i the
//! sample's own chain point, `|v_j - c_i| <= |v_j - v_i| + |v_i - c_i|`, so corner i's term here is
//! at most `value_i + max_j |v_j - v_i|`, which is exactly corner i's Lipschitz term; the minimum
//! over corners of the one is therefore at most the minimum over corners of the other. So
//! `min(Lipschitz, convex)` IS the convex bound, and the search is free to use it alone.
//!
//! Soundness does not care what else the module does with the field. A pick's field (`PickField`)
//! is the minimum over several chains, so its sample's chain point is a c in the union the minimum
//! is taken over and the inequality holds unchanged. The band restricts WHERE the field is read,
//! never how it is computed, and the bound is pointwise. A cell is a sub-triangle or sub-segment of
//! a mesh triangle, and the corners used are the CELL's corners, not the triangle's.
//!
//! **Refuted, do not rebuild:** a convex-cell "no search needed" test in front of the ridge
//! search, skipping a triangle whose convex bound shows it cannot beat the seed, retires
//! NOTHING. The ridge search is seeded at `max(Snap, highestCorner + chordTolerance)`, and the
//! convex bound over a cell whose own corners read at or under that already retires the root cell
//! inside the search at its first step. Measured 2026-09-22: 0 of 15,715 empty searches retired on the Many Holes
//! witness, and 0 on the grid plate at 5, 17 and 37 loops.
//!
//! The end-plane reading (chain index, point) is optional: `None` where no end is clipped.

use solidmesh_math::{ieee_math, Vector3};

use super::FieldSample;

/// A chain's greatest end plane read at a point: `(chain, point) -> value`.
pub type EndPlaneAt<'a> = &'a dyn Fn(i32, Vector3) -> f64;

/// The bound over the segment v0 v1, from the two corners' own chain points.
pub fn over_segment(
    s0: &FieldSample,
    s1: &FieldSample,
    v0: Vector3,
    v1: Vector3,
    end_plane_at: Option<EndPlaneAt>,
) -> f64 {
    // The two ends stand in for the third corner - a segment's farthest point from c is one of
    // its ends.
    let bound = from_one(s0, v0, v1, v1, end_plane_at);
    ieee_math::min_f64(bound, from_one(s1, v0, v1, v1, end_plane_at))
}

/// The bound over the triangle v0 v1 v2, from the three corners' own chain points.
#[allow(clippy::too_many_arguments)]
pub fn over_triangle(
    s0: &FieldSample,
    s1: &FieldSample,
    s2: &FieldSample,
    v0: Vector3,
    v1: Vector3,
    v2: Vector3,
    end_plane_at: Option<EndPlaneAt>,
) -> f64 {
    let mut bound = from_one(s0, v0, v1, v2, end_plane_at);
    bound = ieee_math::min_f64(bound, from_one(s1, v0, v1, v2, end_plane_at));
    bound = ieee_math::min_f64(bound, from_one(s2, v0, v1, v2, end_plane_at));
    bound
}

/// One chain point's term: the farthest corner from it, less the setback it asks for - and, where
/// the field has end clips, no less than the sample's chain's end planes read at the corners.
///
/// A clipped chain's field is `max(g, P)`: g the union of balls, P the greatest end plane. The
/// farthest-corner term bounds g only. P is a max of linear functions, so its greatest value over
/// the cell stands at a corner, and `max(term, P at the corners)` bounds the chain's field; a
/// pick's field is at most any one chain's, so the term of the chain the sample came from bounds
/// the pick. A CLIPPED sample names the end vertex with the plane's value, so `distance - value`
/// there is not a setback - it can stand above the real one and understate the term - and such a
/// sample gives no term.
fn from_one(
    sample: &FieldSample,
    v0: Vector3,
    v1: Vector3,
    v2: Vector3,
    end_plane_at: Option<EndPlaneAt>,
) -> f64 {
    if sample.clipped {
        return f64::INFINITY;
    }

    let c = sample.nearest;
    let farthest = ieee_math::max_f64(
        (v0 - c).length(),
        ieee_math::max_f64((v1 - c).length(), (v2 - c).length()),
    );
    let mut term = farthest - setback_at(sample);
    if let Some(end_plane_at) = end_plane_at {
        term = ieee_math::max_f64(
            term,
            ieee_math::max_f64(
                end_plane_at(sample.chain, v0),
                ieee_math::max_f64(
                    end_plane_at(sample.chain, v1),
                    end_plane_at(sample.chain, v2),
                ),
            ),
        );
    }

    term
}

/// The setback at the chain point this sample minimised at. The sample carries the distance to
/// that point and the value `distance - setback` there, so the setback falls out of the two -
/// which is what makes the bound exact for a setback that VARIES along a chain segment, where the
/// segment's end setbacks would not be.
fn setback_at(sample: &FieldSample) -> f64 {
    sample.distance - sample.value
}
