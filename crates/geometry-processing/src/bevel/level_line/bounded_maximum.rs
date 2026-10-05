//! The GREATEST value a 1-Lipschitz field takes over a segment or a triangle, found by bounded
//! search over ANY evaluator - one chain's field or a whole pick's. Ports the nested types of
//! `Bevel/LevelLine/BoundedMaximum.cs` (a C# static class): the further cell bound a search may be
//! handed ([`BoundedMaximumCellBound`]) and the tolerant best-so-far
//! ([`BoundedMaximumCanonicalMaximum`], with its tolerant offer), the depth that ends a search and
//! the tie-tolerant longest-edge test ([`not_shorter`]). The searches themselves land with a later
//! package.

use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::FieldSample;

/// C# `BoundedMaximum.CellBound`: a further SOUND upper bound on a triangle cell's greatest value,
/// read from the cell's three corner samples and corners - taken with the search's own bounds by
/// the lesser, so it can only retire cells sooner.
pub type BoundedMaximumCellBound =
    Rc<dyn Fn(&FieldSample, &FieldSample, &FieldSample, Vector3, Vector3, Vector3) -> f64>;

/// C# `BoundedMaximum.CanonicalMaximum`: the greatest value a bounded search has found, kept by a
/// TOLERANT update so that a tie is settled by the search's canonical order rather than by the last
/// bit of a float.
///
/// A distance field over a symmetric part is full of plateaux and ridges: a strip's spine is a
/// whole LINE of equally greatest value. Taking "the first sample that is strictly greater" then
/// returns whichever maximiser the recursion reached first, and the last bit of a float decides it -
/// the same pick on the same part gave the peak at (0, -7.5, 2) standing still and (0, -6.875, 2)
/// turned, and the refinement that followed cut the band differently in the two placements.
///
/// Searching HARDER for a geometrically canonical maximiser - the tying point nearest the piece's
/// own centroid - did settle it, and cost more than twice the whole drag budget of a
/// twenty-thousand triangle rim. What settles it for nothing instead is making the SEARCH
/// deterministic: it subdivides at midpoints, so two placements visit the same sub-pieces as long
/// as its own three choices do not hang on noise - which edge is longest (`NotShorter`), the order
/// the two children are explored (fixed), and this update, where a sample replaces the best only by
/// beating it by MORE than a relative tie tolerance. On a plateau the FIRST maximiser reached in
/// that canonical order therefore stands, and it is the same point in every placement.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BoundedMaximumCanonicalMaximum {
    /// The greatest value found.
    pub best: f64,
    /// Where that value stands - the first maximiser in the search's canonical order.
    pub best_point: Vector3,
}

/// Depth rather than a size test ends the recursion, so the cost is bounded whatever the geometry;
/// at depth 60 a segment has been halved past the precision of its own endpoints, and a
/// longest-edge bisection has halved the triangle 30 times in each direction.
pub const MAX_DEPTH: i32 = 60;

/// True when `length` is not shorter than `other`, with lengths within a RELATIVE 1e-12 counted
/// equal - so a tie between two edges of the same piece is settled by the order they are written
/// in and not by which one rounding made a bit longer.
pub fn not_shorter(length: f64, other: f64) -> bool {
    length >= other - (1e-12 * cs_math::max_f64(length.abs(), other.abs()))
}

impl BoundedMaximumCanonicalMaximum {
    /// A tie of values is a tie of ROUNDING, so the tolerance is relative to the value.
    pub const TIE_TOLERANCE: f64 = 1e-9;

    /// A best-so-far starting at `starting_at`, standing at `fallback_point` until a sample beats it.
    pub fn new(starting_at: f64, fallback_point: Vector3) -> Self {
        Self {
            best: starting_at,
            best_point: fallback_point,
        }
    }

    /// Offers one evaluated sample; keeps it only when it beats the best by more than a tie.
    pub fn offer(&mut self, value: f64, point: Vector3) {
        // Nothing has been evaluated yet when the best still stands at negative infinity, and the
        // slack there is infinite - so the first real sample is taken outright rather than
        // compared.
        if self.best == f64::NEG_INFINITY
            || value > self.best + (Self::TIE_TOLERANCE * (1.0 + self.best.abs()))
        {
            // The value reported is the value AT the point reported - a caller reads the two
            // together.
            self.best = value;
            self.best_point = point;
        }
    }
}
