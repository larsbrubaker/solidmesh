//! The level-line build's work accounting (`Bevel/LevelLine/WorkBudget.cs`): which phase was
//! running ([`LevelLineWorkPhase`]), how far a build has got ([`LevelLineProgress`]), the
//! deterministic allowance a build spends ([`WorkBudget`]) with its cancellation and progress
//! report, the two ways out of a build it raises ([`WorkCanceled`], [`WorkBudgetExceeded`]) and
//! the budget in force on this thread ([`LevelLineWork`], C#'s `[ThreadStatic]` slot, set by a
//! [`LevelLineWorkScope`] guard), with the budget's default size, its progress-bar arithmetic
//! (phase weights and starts, the fraction now, the report) and the two exits' constructors and
//! messages. The charging, the phase changes and the nested-search slice land with later runs of
//! the work-budget package as further `impl` blocks.

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::Arc;

use polygon_mesh::CancelToken;
use vector_math::cs_math;

use super::stopwatch;

/// C# `LevelLineWorkPhase`: which part of the construction was running when the work budget ran
/// out.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LevelLineWorkPhase {
    /// Nothing is running.
    #[default]
    None = 0,

    /// Flooding the chain's two sides, or the band, out over the mesh.
    Sides,

    /// The refinement's worklist: splitting until the mesh can carry the line.
    Refinement,

    /// Rooting the crossed mesh edges.
    EdgeRoots,

    /// Walking the band in from the line to the chain, to say what comes out.
    BandDelete,

    /// Reading the room off the region's boundary.
    Room,

    /// Removing degenerate input strips before the refinement, and repairing the needles left.
    InputStrips,

    /// Walking the gap's boundary loops.
    GapLoops,

    /// Zipping the two rails together.
    Fill,

    /// Reading the room from INSIDE a cut - a build handed no room, or the pinch search after its
    /// probes. Named apart from [`LevelLineWorkPhase::Room`] because it runs after the cut has
    /// begun: reported as Room, the words under the bar jumped back to the up-front reading
    /// mid-cut. It carries no weight of its own, so the bar holds where the cut had got to.
    CutRoom,
}

/// C# `LevelLineProgress`: how far a level line build has got - what is running, the work it has
/// spent, and a 0..1 fraction of the whole call. Reported for the bar in front of the user;
/// nothing in the construction reads it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LevelLineProgress {
    /// What was running when this was reported.
    pub phase: LevelLineWorkPhase,
    /// Units of work spent by then.
    pub work_spent: i64,
    /// How far through the call this is, 0..1, never going backwards within one call.
    pub fraction: f64,
}

/// How many phases [`LevelLineWorkPhase`] has: the length of the budget's per-phase tables (C#
/// `Enum.GetValues<LevelLineWorkPhase>().Length`).
pub const LEVEL_LINE_WORK_PHASE_COUNT: usize = 10;

/// The progress callback a [`WorkBudget`] reports to (C# `Action<LevelLineProgress>`).
pub type LevelLineProgressReport = Arc<dyn Fn(LevelLineProgress) + Send + Sync>;

/// C# `WorkBudget`: a deterministic allowance of WORK - field queries, mesh splits and loop steps -
/// that a level line build may spend before it refuses.
///
/// Counted work, never wall clock: the same input spends the same units on any machine, so a
/// refusal is reproducible and a slow machine never refuses a build a fast one made. That is also
/// the owner's rule for a live drag - slow is allowed, running away is not - which a time limit
/// could not keep. The default is proportional to the input and generous: an ordinary build spends
/// a tiny fraction of it, and only a loop that is not bounded by the input at all can reach it.
///
/// The build charges it through the shared handle the ambient slot ([`LevelLineWork`]) holds, so
/// everything the C# mutates in place is interior-mutable here.
pub struct WorkBudget {
    /// The caller's cancellation, read as the build spends work (every `CancelCheckEvery` charges).
    pub cancellation_token: CancelToken,
    /// Who to tell how far along this is, or `None` when nobody is watching.
    pub progress: Option<LevelLineProgressReport>,
    /// How many units of work this budget allows.
    pub limit: i64,
    /// True when the call's input positions were stored as float (the app's meshes), so an input
    /// vertex is only where it is to a float's step; false (exact input) for everything that builds
    /// its own doubles. Carried here because this is the one object every build of a call, and
    /// every pinch-ladder probe, already receives.
    pub input_is_float_stored: bool,
    /// The shortest gap between two progress reports, in milliseconds of wall clock. Settable only
    /// so a test can see every charge's report without racing the clock (C# default 50).
    pub report_throttle_ms: i64,
    /// How many units have been charged so far.
    pub spent: Cell<i64>,
    /// What is running right now.
    pub phase: Cell<LevelLineWorkPhase>,
    /// The phase that was running when the budget ran out, or [`LevelLineWorkPhase::None`].
    pub exhausted_in: Cell<LevelLineWorkPhase>,
    /// Charges since the cancellation token was last read.
    pub since_cancel_check: Cell<i32>,
    /// When the last progress report went out, in milliseconds (the throttle's clock).
    pub last_report_ms: Cell<i64>,
    /// The last fraction reported, so the bar never goes backwards.
    pub last_fraction: Cell<f64>,
    /// Which side group's refinement is running.
    pub side_group_index: Cell<i32>,
    /// How many side groups the refinement runs over; 0 outside the per-group loop.
    pub side_group_count: Cell<i32>,
    /// How far the refinement's worklist has got: items taken.
    pub refinement_taken: Cell<i64>,
    /// How far the refinement's worklist has got: items left.
    pub refinement_left: Cell<i64>,
    /// [`WorkBudget::spent`] when the running phase was entered.
    pub phase_started_at: Cell<i64>,
    /// A running guess, per phase, at what the refinement and the room reading will spend, so those
    /// long phases keep the bar moving.
    pub phase_estimates: RefCell<[f64; LEVEL_LINE_WORK_PHASE_COUNT]>,
    /// Units spent per phase - the tally kept only under `FieldCounters.MeterOn`, at phase changes.
    pub phase_units: RefCell<[i64; LEVEL_LINE_WORK_PHASE_COUNT]>,
    /// Stopwatch ticks spent per phase, kept with [`WorkBudget::phase_units`].
    pub phase_ticks: RefCell<[i64; LEVEL_LINE_WORK_PHASE_COUNT]>,
    /// [`WorkBudget::spent`] at the last tally.
    pub phase_tally_units_at: Cell<i64>,
    /// The stopwatch timestamp of the last tally; 0 before the first.
    pub phase_tally_ticks_at: Cell<i64>,
}

impl std::fmt::Debug for WorkBudget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The progress callback has no Debug; say only whether there is one.
        f.debug_struct("WorkBudget")
            .field("limit", &self.limit)
            .field("spent", &self.spent.get())
            .field("phase", &self.phase.get())
            .field("exhausted_in", &self.exhausted_in.get())
            .field("has_progress", &self.progress.is_some())
            .finish_non_exhaustive()
    }
}

/// C# `WorkCanceledException`: raised inside the module when the caller's cancellation token is
/// cancelled, and turned into the caller's cancellation at every entry point. Its own type, so no
/// handler of the budget's refusal can mistake a cancel for a build that ran away.
#[derive(Debug, Clone)]
pub struct WorkCanceled {
    /// The budget whose token was cancelled.
    pub budget: Rc<WorkBudget>,
}

/// C# `WorkBudgetExceededException`: raised inside the module when a [`WorkBudget`] runs out, and
/// handled at every entry point. It is a way of leaving a deep worklist at once, not an error a
/// caller is meant to handle.
#[derive(Debug, Clone)]
pub struct WorkBudgetExceeded {
    /// The budget that ran out.
    pub budget: Rc<WorkBudget>,
}

thread_local! {
    /// C# `LevelLineWork.current` (`[ThreadStatic]`): the budget in force on this thread.
    static CURRENT: RefCell<Option<Rc<WorkBudget>>> = const { RefCell::new(None) };
}

/// C# `LevelLineWork`: the budget the work being done right now is charged to.
///
/// Ambient and per-thread rather than a parameter on every private helper: the places that charge
/// are the field's own query counters, the mesh's splits and a handful of worklists, all of them
/// many calls below the entry point, and threading a budget through every signature between would
/// move a lot of production code for a measurement. Per-thread, so two builds on two threads each
/// spend their own.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LevelLineWork;

impl LevelLineWork {
    /// The budget in force, or `None` when work is not being counted.
    pub fn current() -> Option<Rc<WorkBudget>> {
        CURRENT.with(|slot| slot.borrow().clone())
    }

    /// Puts a budget in force until the returned scope is dropped, restoring whatever was there.
    pub fn begin(budget: Option<Rc<WorkBudget>>) -> LevelLineWorkScope {
        let was = CURRENT.with(|slot| slot.replace(budget));
        LevelLineWorkScope { was }
    }
}

/// C# `LevelLineWork.Scope`: holds the budget that was in force before
/// [`LevelLineWork::begin`], and puts it back when dropped.
#[derive(Debug)]
#[must_use = "the budget stays in force only while the scope is alive"]
pub struct LevelLineWorkScope {
    was: Option<Rc<WorkBudget>>,
}

impl Drop for LevelLineWorkScope {
    fn drop(&mut self) {
        let was = self.was.take();
        CURRENT.with(|slot| *slot.borrow_mut() = was);
    }
}

impl LevelLineProgress {
    /// C#'s constructor.
    pub fn new(phase: LevelLineWorkPhase, work_spent: i64, fraction: f64) -> Self {
        Self {
            phase,
            work_spent,
            fraction,
        }
    }
}

/// Units of work allowed per triangle of the input - see [`WorkBudget::default_limit`].
pub const DEFAULT_PER_TRIANGLE: i64 = 4000;

/// Units allowed whatever the input's size, so a tiny mesh still has room to work.
pub const DEFAULT_FLOOR: i64 = 2_000_000;

/// How many charges apart the cancellation token is read.
pub const CANCEL_CHECK_EVERY: i32 = 256;

/// The order the phases run in, which is what makes the bar's starts add up the right way.
pub const PHASE_ORDER: [LevelLineWorkPhase; 8] = [
    LevelLineWorkPhase::Room,
    LevelLineWorkPhase::Sides,
    LevelLineWorkPhase::InputStrips,
    LevelLineWorkPhase::Refinement,
    LevelLineWorkPhase::EdgeRoots,
    LevelLineWorkPhase::BandDelete,
    LevelLineWorkPhase::GapLoops,
    LevelLineWorkPhase::Fill,
];

/// C# `Environment.TickCount64`: milliseconds of wall clock, for the progress throttle alone (no
/// result depends on it). Read off the meters' clock, so it stands still in the browser.
pub fn tick_count_ms() -> i64 {
    stopwatch::get_timestamp() / (stopwatch::FREQUENCY / 1000)
}

impl WorkBudget {
    /// The default allowance for a mesh of this many triangles: [`DEFAULT_PER_TRIANGLE`] units each
    /// over [`DEFAULT_FLOOR`]. A cylinder rim - the module's own speed fixture - spends well under a
    /// hundredth of this, so the number is a runaway catch and not a work limit anyone meets by
    /// being big.
    pub fn default_limit(triangle_count: i32) -> i64 {
        DEFAULT_FLOOR + (DEFAULT_PER_TRIANGLE * (triangle_count as i64).max(0))
    }

    /// How far each phase carries the bar, summing to 1 over one loop of the route (band cut then
    /// fill), by phase discriminant (C# `PhaseWeights = TheWeights()`).
    pub fn the_weights() -> [f64; LEVEL_LINE_WORK_PHASE_COUNT] {
        let mut weights = [0.0; LEVEL_LINE_WORK_PHASE_COUNT];
        weights[LevelLineWorkPhase::None as usize] = 0.0;
        weights[LevelLineWorkPhase::Room as usize] = 0.10;
        weights[LevelLineWorkPhase::Sides as usize] = 0.02;

        // A clean part has no input strips, so the removal is a scan and barely moves the bar; the
        // point is surrounding it, so a refusal inside it is named. Refinement carries most of the
        // bar: it measured about 83% of a build's time on Many Holes.
        weights[LevelLineWorkPhase::InputStrips as usize] = 0.01;
        weights[LevelLineWorkPhase::Refinement as usize] = 0.72;
        weights[LevelLineWorkPhase::EdgeRoots as usize] = 0.06;
        weights[LevelLineWorkPhase::BandDelete as usize] = 0.03;
        weights[LevelLineWorkPhase::GapLoops as usize] = 0.03;
        weights[LevelLineWorkPhase::Fill as usize] = 0.03;

        // No weight and no place in the phase order: a room read from inside the cut holds the bar
        // where the cut had got to, rather than claiming a span the up-front reading already owns.
        weights[LevelLineWorkPhase::CutRoom as usize] = 0.0;
        weights
    }

    /// Where each phase starts on the bar - the weights ahead of it in [`PHASE_ORDER`], added up
    /// (C# `PhaseStarts = TheStarts()`).
    pub fn the_starts() -> [f64; LEVEL_LINE_WORK_PHASE_COUNT] {
        let weights = Self::the_weights();
        let mut starts = [0.0; LEVEL_LINE_WORK_PHASE_COUNT];
        let mut ahead = 0.0;
        for phase in PHASE_ORDER {
            starts[phase as usize] = ahead;
            ahead += weights[phase as usize];
        }

        starts
    }

    /// The phases long enough to move the bar within their own span: the refinement, and the room
    /// reading - which is the first thing a pick does, so without this the bar sat at zero all
    /// through it.
    pub fn moves_within_its_span(phase: LevelLineWorkPhase) -> bool {
        phase == LevelLineWorkPhase::Refinement || phase == LevelLineWorkPhase::Room
    }

    /// Holds the bar's fraction monotone and at most 1.
    pub fn clamped(&self, fraction: f64) -> f64 {
        let held = cs_math::min_f64(1.0, cs_math::max_f64(self.last_fraction.get(), fraction));
        self.last_fraction.set(held);
        held
    }

    /// Where the bar is inside the per-side-group loop: the refinement and edge-root spans together
    /// are cut into one slice per group, so group i of G owns the i-th G-th of them. Inside a
    /// group's slice the refinement takes its weighted share and advances with its own worklist
    /// (taken over taken plus waiting); the edge roots take the rest.
    pub fn fraction_through_the_side_groups(&self) -> f64 {
        let weights = Self::the_weights();
        let refinement = weights[LevelLineWorkPhase::Refinement as usize];
        let roots = weights[LevelLineWorkPhase::EdgeRoots as usize];
        let span = refinement + roots;
        let group_weight = span / self.side_group_count.get() as f64;
        let group_start = Self::the_starts()[LevelLineWorkPhase::Refinement as usize]
            + (group_weight * self.side_group_index.get() as f64);
        let refinement_share = refinement / span;

        if self.phase.get() == LevelLineWorkPhase::EdgeRoots {
            return group_start + (group_weight * refinement_share);
        }

        let queued = self.refinement_taken.get() + self.refinement_left.get();
        let within = if queued > 0 {
            self.refinement_taken.get() as f64 / queued as f64
        } else {
            0.0
        };

        group_start + (group_weight * refinement_share * cs_math::min_f64(1.0, within))
    }

    /// How far through the call the bar stands now, never going backwards. Nothing knows how much
    /// splitting a pick - or reading its room - needs until it is done, so a long phase's estimate
    /// grows with the work: the bar keeps approaching the end of the phase without ever claiming
    /// to reach it.
    pub fn fraction_now(&self) -> f64 {
        let phase_now = self.phase.get();
        let phase = phase_now as usize;
        let start = Self::the_starts()[phase];
        let weight = Self::the_weights()[phase];
        let mut within = 0.0;

        if self.side_group_count.get() > 0
            && (phase_now == LevelLineWorkPhase::Refinement
                || phase_now == LevelLineWorkPhase::EdgeRoots)
        {
            return self.clamped(self.fraction_through_the_side_groups());
        }

        if Self::moves_within_its_span(phase_now) {
            let spent_here = (self.spent.get() - self.phase_started_at.get()) as f64;
            let mut estimates = self.phase_estimates.borrow_mut();
            if spent_here > estimates[phase] {
                estimates[phase] = spent_here * 1.6;
            }

            within = if estimates[phase] > 0.0 {
                spent_here / estimates[phase]
            } else {
                0.0
            };
        }

        self.clamped(start + (weight * cs_math::min_f64(1.0, cs_math::max_f64(0.0, within))))
    }

    /// Tells the progress callback where the build stands now; nothing when nobody is watching.
    pub fn report_now(&self) {
        let Some(progress) = &self.progress else {
            return;
        };

        self.last_report_ms.set(tick_count_ms());
        progress(LevelLineProgress::new(
            self.phase.get(),
            self.spent.get(),
            self.fraction_now(),
        ));
    }
}

impl WorkCanceled {
    /// C#'s constructor.
    pub fn new(budget: Rc<WorkBudget>) -> Self {
        Self { budget }
    }
}

impl std::fmt::Display for WorkCanceled {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("the level line build was cancelled by its caller")
    }
}

impl std::error::Error for WorkCanceled {}

impl WorkBudgetExceeded {
    /// C#'s constructor.
    pub fn new(budget: Rc<WorkBudget>) -> Self {
        Self { budget }
    }
}

impl std::fmt::Display for WorkBudgetExceeded {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "the level line work budget of {} units ran out in {:?}",
            self.budget.limit,
            self.budget.exhausted_in.get()
        )
    }
}

impl std::error::Error for WorkBudgetExceeded {}
