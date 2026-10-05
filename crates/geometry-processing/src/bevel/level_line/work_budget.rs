//! The level-line build's work accounting (`Bevel/LevelLine/WorkBudget.cs`): which phase was
//! running ([`LevelLineWorkPhase`]), how far a build has got ([`LevelLineProgress`]), the
//! deterministic allowance a build spends ([`WorkBudget`]) with its cancellation and progress
//! report, the two ways out of a build it raises ([`WorkCanceled`], [`WorkBudgetExceeded`]) and
//! the budget in force on this thread ([`LevelLineWork`], C#'s `[ThreadStatic]` slot, set by a
//! [`LevelLineWorkScope`] guard). The budget's charging, phase and progress methods land with the
//! core's work-budget package as further `impl` blocks.

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::Arc;

use polygon_mesh::CancelToken;

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
