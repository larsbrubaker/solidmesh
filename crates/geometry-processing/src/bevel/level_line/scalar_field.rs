//! The only thing the level-line cut knows about the bevel: one number per point in space, and the
//! chain field that can be ASKED where it runs. Ports `Bevel/LevelLine/ScalarField.cs`
//! (`IScalarField`, `FieldMinimum`, `FieldMaximum`, `IChainDistanceField`).
//!
//! C#'s overloads take distinct names here: `MaxAlongSegment(a, b, tolerance, atLeast)` is
//! [`ChainDistanceField::max_along_segment_at_least`] and the six-argument `MaxOverTriangle` is
//! [`ChainDistanceField::max_over_triangle_bounded`] (its `atLeast` default of negative infinity
//! passed explicitly).
//!
//! Every query may charge the work budget in force, so each answers `Result<_, WorkError>` and a
//! caller passes a cancel or an exhausted budget up with `?` (C# throws through them).

use vector_math::Vector3;

use super::WorkError;

/// One number per point in space, negative in the band that is to be deleted, positive outside
/// it, zero on the level line the cut follows (C# `IScalarField`).
///
/// Deliberately this narrow so the cut can be built and proven against fields with an exact
/// answer (distance to a plane, a point, a segment) before the chain's own field exists.
/// Implementations are expected to be 1-Lipschitz - a true distance, not a scaled or squared one -
/// because the refinement leans on that to rule out an edge crossing the line twice.
pub trait ScalarField {
    /// The field value at a point: negative inside the band, positive outside, zero on the line.
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError>;
}

/// The least value a field takes over a piece of the mesh, and the point where it takes it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FieldMinimum {
    /// The least field value over the segment or triangle asked about.
    pub value: f64,
    /// The point where that least value is attained.
    pub point: Vector3,
}

impl FieldMinimum {
    /// C#'s constructor.
    pub fn new(value: f64, point: Vector3) -> Self {
        FieldMinimum { value, point }
    }
}

/// The greatest value a field takes over a piece of the mesh, and the point where it takes it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FieldMaximum {
    /// The greatest field value over the triangle asked about, to the tolerance asked for.
    pub value: f64,
    /// The point where that greatest value is attained.
    pub point: Vector3,
}

impl FieldMaximum {
    /// C#'s constructor.
    pub fn new(value: f64, point: Vector3) -> Self {
        FieldMaximum { value, point }
    }
}

/// A field that can be ASKED where it runs, rather than hunted by halving: as well as a value at a
/// point it answers its least value over a whole segment or triangle, and where that least value
/// stands (C# `IChainDistanceField`).
///
/// The field is convex over each chain segment (a norm of an affine map less a linear term), so
/// these minima are found deterministically. That is what lets the refinement split an edge ONCE
/// at the point where it dips across the line - instead of halving it down to the chord tolerance
/// merely to prove, from a Lipschitz bound, that it does not cross twice.
///
/// **The minima are exact.** The field is `min over chain points c of (|q - c| - setback(c))`, so
/// the least over a piece of the mesh is a minimum over the piece of a minimum over the chain - and
/// the two commute. Searching each chain segment independently and taking the best therefore
/// returns a value the field ATTAINS, at the point returned, whether or not the setback varies.
pub trait ChainDistanceField: ScalarField {
    /// The least field value along the segment from `a` to `b`, and where it stands - exact,
    /// whatever the setback does.
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError>;

    /// The least field value over the triangle abc, its interior included.
    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError>;

    /// A cheap "no" to `is the least value over this box at or below the margin`, for a caller
    /// that only asks whether a piece dips across the line. The default answers "cannot say", so a
    /// field that has no such bound to give simply pays for the exact search.
    fn nothing_reaches_below(&self, _min: Vector3, _max: Vector3, _margin: f64) -> bool {
        false
    }

    /// The GREATEST field value along the segment from `a` to `b`, to within `tolerance` - the
    /// mirror of [`ChainDistanceField::min_along_segment`]. An edge with both ends INSIDE whose
    /// middle rises above the line crosses it twice, exactly as an edge with both ends outside that
    /// dips below it does, and neither pair of crossings can be seen from the ends.
    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError>;

    /// The same search, given a floor `at_least` to prune against from its first step, so every
    /// sub-segment whose bound cannot beat it is discarded - the one-dimensional twin of the seeded
    /// triangle search, with the same sentinel answer when nothing beats it. A caller testing the
    /// answer against a threshold T passes `T - tolerance`, the floor that can never by itself drop
    /// a piece the test would have passed.
    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError>;

    /// The GREATEST field value over the triangle abc, its interior included, to within
    /// `tolerance` - the island of kept material a cut would otherwise swallow whole, and the room
    /// a loop closing on itself has. Found by bounded search, not by sampling.
    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError>;

    /// The same search, told a value `at_least` the answer must beat to be of any use - so every
    /// part of the triangle whose bound cannot beat it is discarded from the first step - and a
    /// `stop_above` past which it may stop early. C# defaults `at_least` to negative infinity.
    ///
    /// The threshold is ABSOLUTE - fixed before the search and never raised by it. With `M` the
    /// true peak: when `M > at_least + tolerance` the answer beats the threshold exactly as it
    /// would without one; when `M <= at_least` neither search beats it; between the two the answer
    /// is indeterminate, which is no loss since the plain search is itself only accurate to the
    /// tolerance. When nothing beats it the answer is `at_least` itself, standing at a MEANINGLESS
    /// point, so a caller must compare with a strict `>` against the same threshold it passed in
    /// and never read the point of an answer that did not beat it.
    #[allow(clippy::too_many_arguments)]
    fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError>;

    /// The island rule's search: whether the field rises above `snap` inside the triangle abc, and
    /// where the split vertex goes when it does - a point the SAME whatever a search pruning harder
    /// would have held. Read the point only when the value beats the snap.
    fn island_peak(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        snap: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_over_triangle_bounded(a, b, c, tolerance, f64::INFINITY, snap - tolerance)
    }
}
