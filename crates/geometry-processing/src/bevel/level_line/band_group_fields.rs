//! The ONE field each side group of a band cut reads: the pick field of the chains that seed it -
//! and, where a `CreaseRunOut` leaves one of its corners across that group's faces, the least of
//! that and the run-out's own field (`RunOutField`). Ports the nested types of
//! `Bevel/LevelLine/BandGroupFields.cs` (a C# static class): the run-out gate ([`run_outs_on`],
//! [`turn_run_outs_on`] and its scope [`BandGroupFieldsRunOutsScope`], kept on the build's
//! [`WorkBudget`](super::WorkBudget) as C#'s `AsyncLocal`) and the wrapped field
//! ([`BandGroupFieldsWithRunOuts`], with its constructor, inner field, point query, bounded maxima
//! and field seam) and the pick under it
//! ([`unwrapped`]). The static class's other functions and the wrapped field's
//! queries land with later packages.

use std::rc::Rc;

use vector_math::Vector3;

use super::bounded_maximum;
use super::{
    ChainDistanceField, FieldCounters, FieldKind, FieldMaximum, FieldSample, LevelLineWork,
    ScalarField, WorkError,
};

/// C# `BandGroupFields.RunOutsOn`: whether the side groups read their crease run-outs' fields for
/// the build in force (off wherever no budget is, as C#'s flow default is).
///
/// C# keeps the gate per ASYNC FLOW rather than per thread: no options object reaches from the
/// app's bevel build down to here, and the app builds on the thread pool, so a thread-static gate
/// could not be turned on by an app-level test. Here it travels with the build, on its
/// [`WorkBudget`](super::WorkBudget).
pub fn run_outs_on() -> bool {
    LevelLineWork::current().is_some_and(|budget| budget.run_outs_on.get())
}

/// C# `BandGroupFields.TurnRunOutsOn`: turns run-outs on for the build in force until the returned
/// scope is dropped, which puts back what was there before.
pub fn turn_run_outs_on() -> BandGroupFieldsRunOutsScope {
    BandGroupFieldsRunOutsScope::new()
}

/// C# `BandGroupFields.RunOutsScope`: turns run-outs on for the build in force until it is
/// dropped, which puts back what was there before. With no budget in force there is no build to
/// switch and the scope does nothing.
#[derive(Debug)]
#[must_use = "run-outs stay on only while the scope is alive"]
pub struct BandGroupFieldsRunOutsScope {
    /// Whether run-outs were on when the scope was opened - restored when it closes.
    pub was: bool,
}

impl BandGroupFieldsRunOutsScope {
    /// C#'s `RunOutsScope()`: remembers the setting and turns run-outs on.
    pub fn new() -> Self {
        let was = LevelLineWork::current().is_some_and(|budget| budget.run_outs_on.replace(true));
        BandGroupFieldsRunOutsScope { was }
    }
}

impl Default for BandGroupFieldsRunOutsScope {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for BandGroupFieldsRunOutsScope {
    /// C#'s `Dispose`: puts back what was there before.
    fn drop(&mut self) {
        if let Some(budget) = LevelLineWork::current() {
            budget.run_outs_on.set(self.was);
        }
    }
}

/// C# `BandGroupFields.Unwrapped`: the pick field under a group's field: a wrapped group's inner
/// pick, any other field itself. What asks about the pick's own CHAINS - where two tie, where a
/// bounded end is a known cut - reads this; the run-out has no chain, tie or end plane.
pub fn unwrapped(field: &Rc<dyn ChainDistanceField>) -> Rc<dyn ChainDistanceField> {
    match field.as_with_run_outs() {
        Some(wrapped) => wrapped.inner().clone(),
        None => field.clone(),
    }
}

/// C# `BandGroupFields.WithRunOuts`: the least of a group's pick field and its run-outs.
///
/// A reading the run-out wins keeps the pick's chain and parameter (the loop walk needs an owner;
/// `CornerFill.TrimOffTheLine` hands the run back to the corner hole, since the chain's own field
/// reads it off its line), and is marked [`FieldSample::clipped`] so `ConvexCellBound` takes no
/// union-of-balls term from it - its `distance - value` is no setback.
pub struct BandGroupFieldsWithRunOuts {
    /// The pick field the run-outs are read beside (C# `Inner`).
    pub pick: Rc<dyn ChainDistanceField>,
    /// The pick field's sampler.
    pub pick_sample: Rc<dyn Fn(Vector3) -> Result<FieldSample, WorkError>>,
    /// The pick's end planes: the signed reading of a chain's end plane at a point; `None` where no
    /// member is clipped (C#'s null reader).
    pub end_planes: Option<Rc<dyn Fn(i32, Vector3) -> f64>>,
    /// The run-out fields read beside the pick.
    pub run_outs: Vec<Rc<dyn ChainDistanceField>>,
}

impl BandGroupFieldsWithRunOuts {
    /// C#'s constructor.
    pub fn new(
        pick: Rc<dyn ChainDistanceField>,
        pick_sample: Rc<dyn Fn(Vector3) -> Result<FieldSample, WorkError>>,
        end_planes: Option<Rc<dyn Fn(i32, Vector3) -> f64>>,
        run_outs: Vec<Rc<dyn ChainDistanceField>>,
    ) -> Self {
        BandGroupFieldsWithRunOuts {
            pick,
            pick_sample,
            end_planes,
            run_outs,
        }
    }

    /// C# `WithRunOuts.Inner`: the pick field the run-outs are read beside.
    pub fn inner(&self) -> &Rc<dyn ChainDistanceField> {
        &self.pick
    }

    /// True only when neither the pick nor any run-out field reaches below `margin` over the box
    /// `min`..`max`.
    pub fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        self.pick.nothing_reaches_below(min, max, margin)
            && self
                .run_outs
                .iter()
                .all(|run_out| run_out.nothing_reaches_below(min, max, margin))
    }
}

impl BandGroupFieldsWithRunOuts {
    /// C# `WithRunOuts.RunOutValueAt`: the least of the run-outs alone at `point` (NaN where any
    /// reads NaN, as C#'s `Enumerable.Min`).
    pub fn run_out_value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        let mut least: Option<f64> = None;
        for run_out in &self.run_outs {
            let value = run_out.value_at(point)?;
            if value.is_nan() {
                return Ok(f64::NAN);
            }

            if least.is_none_or(|l| value < l) {
                least = Some(value);
            }
        }

        // C# throws on no run-outs; a wrapper is made only with some (`BandGroupFields.Of`).
        Ok(least.unwrap_or(f64::NAN))
    }

    /// C# `WithRunOuts.Evaluate`: the pick's sample, lowered to any run-out that reads below it -
    /// keeping the pick's chain and parameter and marked clipped (see the type's remarks); a
    /// run-out that cannot reach below the sample is skipped unread.
    pub fn evaluate(&self, point: Vector3) -> Result<FieldSample, WorkError> {
        let mut sample = (self.pick_sample)(point)?;
        for run_out in &self.run_outs {
            if run_out.nothing_reaches_below(point, point, sample.value) {
                continue;
            }

            let value = run_out.value_at(point)?;
            if value < sample.value {
                sample = FieldSample {
                    value,
                    clipped: true,
                    ..sample
                };
            }
        }

        Ok(sample)
    }

    /// C# `WithRunOuts.MaxAlongSegment(a, b, tolerance)`.
    pub fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_along_segment_at_least(a, b, tolerance, f64::NEG_INFINITY)
    }

    /// C# `WithRunOuts.MaxAlongSegment(a, b, tolerance, atLeast)`: the bounded search over the
    /// wrapped field, clipped by the pick's end planes.
    pub fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        FieldCounters::with(|t| t.max_along += 1);
        LevelLineWork::charge(1)?;
        bounded_maximum::over_segment(
            &|p| self.evaluate(p),
            a,
            b,
            tolerance,
            at_least,
            self.end_planes.as_deref(),
        )
    }

    /// C# `WithRunOuts.MaxOverTriangle(a, b, c, tolerance)`.
    pub fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_over_triangle_bounded(a, b, c, tolerance, f64::INFINITY, f64::NEG_INFINITY)
    }

    /// C# `WithRunOuts.MaxOverTriangle(a, b, c, tolerance, stopAbove, atLeast)`.
    pub fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        FieldCounters::with(|t| t.max_tri += 1);
        LevelLineWork::charge(1)?;
        bounded_maximum::over_triangle(
            &|p| self.evaluate(p),
            a,
            b,
            c,
            tolerance,
            stop_above,
            at_least,
            None,
            self.end_planes.as_deref(),
        )
    }
}

impl ScalarField for BandGroupFieldsWithRunOuts {
    /// C# `WithRunOuts.ValueAt`: [`BandGroupFieldsWithRunOuts::evaluate`]'s value.
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok(self.evaluate(point)?.value)
    }

    fn kind(&self) -> FieldKind {
        FieldKind::WithRunOuts
    }

    fn as_with_run_outs(&self) -> Option<&BandGroupFieldsWithRunOuts> {
        Some(self)
    }
}
