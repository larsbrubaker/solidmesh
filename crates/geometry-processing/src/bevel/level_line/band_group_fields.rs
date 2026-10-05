//! The ONE field each side group of a band cut reads: the pick field of the chains that seed it -
//! and, where a `CreaseRunOut` leaves one of its corners across that group's faces, the least of
//! that and the run-out's own field (`RunOutField`). Ports the nested types of
//! `Bevel/LevelLine/BandGroupFields.cs` (a C# static class): the run-out gate ([`run_outs_on`],
//! [`turn_run_outs_on`] and its scope [`BandGroupFieldsRunOutsScope`], kept on the build's
//! [`WorkBudget`](super::WorkBudget) as C#'s `AsyncLocal`) and the wrapped field
//! ([`BandGroupFieldsWithRunOuts`]). The static class's other functions and the wrapped field's
//! queries land with later packages.

use std::rc::Rc;

use vector_math::Vector3;

use super::{ChainDistanceField, FieldSample, LevelLineWork};

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
    pub pick_sample: Rc<dyn Fn(Vector3) -> FieldSample>,
    /// The pick's end planes: the signed reading of a chain's end plane at a point.
    pub end_planes: Rc<dyn Fn(i32, Vector3) -> f64>,
    /// The run-out fields read beside the pick.
    pub run_outs: Vec<Rc<dyn ChainDistanceField>>,
}

impl BandGroupFieldsWithRunOuts {
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
