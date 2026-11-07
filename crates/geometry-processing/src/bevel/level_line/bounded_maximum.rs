//! The GREATEST value a 1-Lipschitz field takes over a segment or a triangle, found by bounded
//! search over ANY evaluator - one chain's field or a whole pick's: the further cell bound a search may be
//! handed ([`BoundedMaximumCellBound`]) and the tolerant best-so-far
//! ([`BoundedMaximumCanonicalMaximum`], with its tolerant offer), the depth that ends a search and
//! the tie-tolerant longest-edge test ([`not_shorter`]), and the searches: over a segment
//! ([`over_segment`]), over a triangle ([`over_triangle`]), the island rule's peak
//! ([`island_peak`]) and the bound a cell is judged by ([`sharpen`]).
//!
//! The search evaluates corners and bounds above: from a corner p any point q of the piece has
//! `value(q) <= value(p) + |q - p|`, and the farthest point of a segment or triangle from p is one
//! of its corners. A piece whose bound cannot beat the best found so far by more than the tolerance
//! is discarded; otherwise it is halved and both halves recurse. Nothing is sampled, so a bump as
//! narrow as the tolerance is still found, and the same piece always gives the same answer. The
//! Lipschitz bound is joined by the convex cell bound (`convex_cell_bound.rs`); both are SOUND and
//! the lesser is taken. Turning this into an EXISTENCE search was tried twice and got the
//! wrong answer both times on a peak that barely clears the threshold; the one early-out that
//! holds is `stop_above`, which stops on a value already FOUND, never on a bound.

use std::rc::Rc;

use vector_math::{ieee_math, Vector3};

use super::convex_cell_bound::{self, EndPlaneAt};
use super::{FieldCounters, FieldMaximum, FieldSample, LevelLineWork, WorkError};

/// A point query a search runs over; it may charge the budget.
pub type Evaluate<'a> = &'a dyn Fn(Vector3) -> Result<FieldSample, WorkError>;

/// A further cell bound as the searches take it.
pub type CellBound<'a> = &'a dyn Fn(
    &FieldSample,
    &FieldSample,
    &FieldSample,
    Vector3,
    Vector3,
    Vector3,
) -> Result<f64, WorkError>;

/// Panics on a non-positive tolerance: a search with no tolerance
/// would never retire a cell and halve to `MAX_DEPTH` everywhere, so it is a caller's bug, not an
/// input to refuse.
fn require_positive(tolerance: f64) {
    assert!(
        tolerance > 0.0,
        "A bounded search needs a positive tolerance."
    );
}

/// The greatest value along the segment ab, to within
/// `tolerance`, pruning from its first step against a value `at_least` the answer must beat to be
/// of any use - a search nothing beats returns `at_least` itself at a MEANINGLESS point (`a`).
pub fn over_segment(
    evaluate: Evaluate,
    a: Vector3,
    b: Vector3,
    tolerance: f64,
    at_least: f64,
    end_plane_at: Option<EndPlaneAt>,
) -> Result<FieldMaximum, WorkError> {
    require_positive(tolerance);

    // A segment lying ALONG a ridge attains its greatest value at every point of it, so which
    // maximiser is returned is settled by the search's own canonical order, not by float noise.
    let mut found = BoundedMaximumCanonicalMaximum::new(at_least, a);
    let mut stack = vec![(a, b, 0)];
    while let Some((p, q, depth)) = stack.pop() {
        LevelLineWork::charge(1)?;
        LevelLineWork::poll_cancel()?;

        let at_p = evaluate(p)?;
        let at_q = evaluate(q)?;
        let span = (q - p).length();
        found.offer(at_p.value, p);
        found.offer(at_q.value, q);

        // The field itself is 1-Lipschitz, so a sample's own VALUE plus the reach to the farthest
        // point of the piece bounds it - no setback term of any kind.
        let mut bound = ieee_math::min_f64(at_p.value, at_q.value) + span;
        bound = sharpen(
            bound,
            convex_cell_bound::over_segment(&at_p, &at_q, p, q, end_plane_at),
            found.best + tolerance,
            depth,
        )?;
        FieldCounters::note_step(depth);
        if depth >= MAX_DEPTH || bound <= found.best + tolerance {
            continue;
        }

        let m = (p + q) * 0.5;
        stack.push((p, m, depth + 1));
        stack.push((m, q, depth + 1));
    }

    Ok(FieldMaximum::new(found.best, found.best_point))
}

/// The greatest value over the triangle abc, interior included,
/// to within `tolerance`, giving up once it is known to stand above `stop_above` and pruning from
/// its first step against a value `at_least` already attained elsewhere.
#[allow(clippy::too_many_arguments)]
pub fn over_triangle(
    evaluate: Evaluate,
    a: Vector3,
    b: Vector3,
    c: Vector3,
    tolerance: f64,
    stop_above: f64,
    at_least: f64,
    extra_bound: Option<CellBound>,
    end_plane_at: Option<EndPlaneAt>,
) -> Result<FieldMaximum, WorkError> {
    require_positive(tolerance);

    // A plateau or a ridge has a whole LINE - or a region - of equally greatest value, so "the
    // first maximiser the search reaches" must not be decided by the last bit of a float. The
    // search's own choices are made canonical instead (longest edge by a relative tie, fixed child
    // order, a tolerant update), so the first maximiser reached is the same in every placement.
    let mut found = BoundedMaximumCanonicalMaximum::new(at_least, a);
    let mut stack = vec![(a, b, c, 0)];
    while let Some((p, q, r, depth)) = stack.pop() {
        LevelLineWork::charge(1)?;
        LevelLineWork::poll_cancel()?;

        let samples = [evaluate(p)?, evaluate(q)?, evaluate(r)?];
        let corners = [p, q, r];
        let mut bound = f64::INFINITY;
        for i in 0..3 {
            found.offer(samples[i].value, corners[i]);

            let mut farthest = 0.0;
            for corner in corners {
                farthest = ieee_math::max_f64(farthest, (corner - corners[i]).length());
            }

            bound = ieee_math::min_f64(bound, samples[i].value + farthest);
        }

        if found.best > stop_above {
            // Known to stand above the threshold, and the caller asked for nothing finer.
            break;
        }

        let convex = convex_cell_bound::over_triangle(
            &samples[0],
            &samples[1],
            &samples[2],
            p,
            q,
            r,
            end_plane_at,
        );
        bound = sharpen(bound, convex, found.best + tolerance, depth)?;
        if let Some(extra_bound) = extra_bound {
            bound = ieee_math::min_f64(
                bound,
                extra_bound(&samples[0], &samples[1], &samples[2], p, q, r)?,
            );
        }

        FieldCounters::note_step(depth);
        if depth == 0 && FieldCounters::meter_on() {
            // The "no search needed" question, asked of the WHOLE triangle: three corners reading
            // one chain, and a convex bound that cannot rise above the corners, would together say
            // the greatest value stands at a corner and this search need never have been made.
            let corner_max = ieee_math::max_f64(
                samples[0].value,
                ieee_math::max_f64(samples[1].value, samples[2].value),
            );
            FieldCounters::note_root_cell(
                samples[0].chain == samples[1].chain && samples[1].chain == samples[2].chain,
                convex <= corner_max,
                bound <= found.best + tolerance,
            );
        }

        if depth >= MAX_DEPTH || bound <= found.best + tolerance {
            continue;
        }

        // Split at the longest edge's midpoint - the halves shrink in every direction, so the
        // bound closes on the value however thin the triangle starts. Lengths within a relative
        // 1e-12 count EQUAL and the tie goes to the first in the piece's own corner order, which
        // comes from vertex indices, so two placements split the same edge.
        let ab = (q - p).length();
        let bc = (r - q).length();
        let ca = (p - r).length();
        if not_shorter(ab, bc) && not_shorter(ab, ca) {
            let m = (p + q) * 0.5;
            stack.push((p, m, r, depth + 1));
            stack.push((m, q, r, depth + 1));
        } else if not_shorter(bc, ca) {
            let m = (q + r) * 0.5;
            stack.push((q, m, p, depth + 1));
            stack.push((m, r, p, depth + 1));
        } else {
            let m = (r + p) * 0.5;
            stack.push((r, m, q, depth + 1));
            stack.push((m, p, q, depth + 1));
        }
    }

    Ok(FieldMaximum::new(found.best, found.best_point))
}

/// The island rule's question and answer over the triangle abc:
/// does the field rise above `snap` anywhere inside it, and if so, where the split vertex goes.
///
/// The answer is a peak only to within the tolerance, and WHICH sample within it the search holds
/// as its best depends on which cells it retired on the way: a sharper sound bound retires cells
/// sooner and the best moves (by 0.0077 at a tolerance of 0.01 in `IslandSplitOrderTests`), and a
/// vertex moved by less than the tolerance was measured to flip a rim fill from closed into a
/// pinch. So the placement is taken from ONE fixed search - the plain one - and `extra_bound` is
/// only a cheaper way to say NO: when the pruned search says yes, the plain search is run to place
/// the vertex and its answer is the rule's, decision included.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn island_peak(
    evaluate: Evaluate,
    a: Vector3,
    b: Vector3,
    c: Vector3,
    tolerance: f64,
    snap: f64,
    extra_bound: Option<CellBound>,
    end_plane_at: Option<EndPlaneAt>,
) -> Result<FieldMaximum, WorkError> {
    let decision = over_triangle(
        evaluate,
        a,
        b,
        c,
        tolerance,
        f64::INFINITY,
        snap - tolerance,
        extra_bound,
        end_plane_at,
    )?;
    if extra_bound.is_none() || !(decision.value > snap) {
        return Ok(decision);
    }

    over_triangle(
        evaluate,
        a,
        b,
        c,
        tolerance,
        f64::INFINITY,
        snap - tolerance,
        None,
        end_plane_at,
    )
}

/// The bound this cell is judged by - the LESSER of the Lipschitz
/// reach bound and the convex union-of-balls bound, with the measurement of what the sharper one
/// buys taken on the way past. Both are upper bounds, so taking the lesser keeps the search sound.
/// The convex bound is charged one work unit - the same charge one `Evaluate` takes for a walk of
/// the whole segment tree, and so a conservative one.
pub fn sharpen(lipschitz: f64, convex: f64, retire_at: f64, depth: i32) -> Result<f64, WorkError> {
    if FieldCounters::meter_on() {
        FieldCounters::note_convex_bound(lipschitz > retire_at && convex <= retire_at, depth);
    }

    LevelLineWork::charge(1)?;
    Ok(ieee_math::min_f64(lipschitz, convex))
}

/// A further SOUND upper bound on a triangle cell's greatest value,
/// read from the cell's three corner samples and corners - taken with the search's own bounds by
/// the lesser, so it can only retire cells sooner.
/// The bound may charge the work budget (a chain's segment bound does), so it answers a `Result`.
pub type BoundedMaximumCellBound = Rc<
    dyn Fn(
        &FieldSample,
        &FieldSample,
        &FieldSample,
        Vector3,
        Vector3,
        Vector3,
    ) -> Result<f64, WorkError>,
>;

/// The greatest value a bounded search has found, kept by a
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
    length >= other - (1e-12 * ieee_math::max_f64(length.abs(), other.abs()))
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
