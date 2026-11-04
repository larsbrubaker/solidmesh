//! The level-line build's work accounting: which phase was
//! running ([`LevelLineWorkPhase`]), how far a build has got ([`LevelLineProgress`]), the
//! deterministic allowance a build spends ([`WorkBudget`]) with its cancellation and progress
//! report, the way out of a build it gives ([`WorkError`]: cancelled, or the budget exceeded) and
//! the budget in force on this thread ([`LevelLineWork`], a thread-local slot, set by a
//! [`LevelLineWorkScope`] guard), with the budget's default size, its progress-bar arithmetic
//! (phase weights and starts, the fraction now, the report), the charging and cancel poll, its
//! constructors, the side-group and refinement-queue notes, the per-phase tally and its report. It also carries
//! the build's per-flow switches, which travel with the build rather than the thread. The phase
//! changes and the nested-search slice are further `impl` blocks.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use polygon_mesh::CancelToken;
use vector_math::{ieee_math, number_format};

use super::stopwatch;

/// Which part of the construction was running when the work budget ran
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

/// How far a level line build has got - what is running, the work it has
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

/// How many phases [`LevelLineWorkPhase`] has: the length of the budget's per-phase tables.
pub const LEVEL_LINE_WORK_PHASE_COUNT: usize = 10;

/// The progress callback a [`WorkBudget`] reports to.
pub type LevelLineProgressReport = Rc<dyn Fn(LevelLineProgress)>;

/// A watcher of the corner cap's hole fills: the mesh, the fill's vertices, how many of them are
/// the hole's own points, its triangles and the rows it closed.
pub type WatchHoleFills = Rc<
    dyn Fn(
        &super::LevelLineMesh,
        &[vector_math::Vector3],
        i32,
        &[super::HoleFillTriangle],
        &[super::CornerCapLastRow],
    ),
>;

/// A deterministic allowance of WORK - field queries, mesh splits and loop steps -
/// that a level line build may spend before it refuses.
///
/// Counted work, never wall clock: the same input spends the same units on any machine, so a
/// refusal is reproducible and a slow machine never refuses a build a fast one made. That is also
/// the owner's rule for a live drag - slow is allowed, running away is not - which a time limit
/// could not keep. The default is proportional to the input and generous: an ordinary build spends
/// a tiny fraction of it, and only a loop that is not bounded by the input at all can reach it.
///
/// The build charges it through the shared handle the ambient slot ([`LevelLineWork`]) holds, so
/// everything a charge updates is interior-mutable.
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
    /// so a test can see every charge's report without racing the clock (default 50).
    pub report_throttle_ms: i64,
    /// Whether the pair corner's end-face field is turned OFF for this build (a per-flow
    /// switch, on by default): a per-flow switch travels with the build, not the
    /// thread, so whoever creates the budget captures the caller's setting here, and
    /// `PairCornerEndFaceField::turn_off` flips it for the budget in force.
    pub pair_corner_end_face_field_off: Cell<bool>,
    /// Whether a pick's both-sided members are turned OFF for this build (a per-flow switch,
    /// on by default), carried the same way.
    pub both_sided_members_off: Cell<bool>,
    /// Whether the side groups read their crease run-outs' fields for this build (a per-flow
    /// switch, off by default); `BandGroupFields::turn_run_outs_on`
    /// flips it for the budget in force, and a budget created inside a scope starts on.
    pub run_outs_on: Cell<bool>,
    /// Whether the square-crease end clips are turned OFF for this build (a per-flow switch,
    /// on by default);
    /// `square_crease_end_clip::turn_square_crease_clips_off` flips it for the budget in force.
    pub square_crease_clips_off: Cell<bool>,
    /// Whether a square crease end clip may NOT lean off square for this build (off by default):
    /// the build that tries the narrow-face flow first sets it (`band_gap_entry_points`).
    pub leaning_crease_clips_off: Cell<bool>,
    /// Whether this build reads stopped ends (a per-flow switch, off by default);
    /// `stopped_end_clip::turn_stopped_ends_on` flips it for the budget in force, and a budget
    /// created inside a scope starts on.
    pub stopped_ends_on: Cell<bool>,
    /// Who watches each corner hole the cap closes with the hole fill for this build (a per-flow
    /// watcher, none by default): the mesh, the fill's
    /// vertices, how many of them are the hole's own points, its triangles and the rows it closed.
    /// `None` (the default) costs nothing.
    pub watch_hole_fills: RefCell<Option<WatchHoleFills>>,
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

/// How a level-line build leaves early: returned by every function that charges the budget or polls
/// for a cancel and passed up with `?`; the build's entry points turn it into the outcome - the
/// caller's cancellation, or a refusal by name. Two cases, so no handler of the budget's refusal
/// can mistake a cancel for a build that ran away.
#[derive(Debug, Clone)]
pub enum WorkError {
    /// The caller's cancellation token was cancelled.
    Cancelled,
    /// The budget ran out - a way of leaving a deep worklist at once, not an error a caller is
    /// meant to handle.
    BudgetExceeded(Rc<WorkBudget>),
}

impl std::fmt::Display for WorkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WorkError::Cancelled => f.write_str("the level line build was cancelled by its caller"),
            WorkError::BudgetExceeded(budget) => write!(
                f,
                "the level line work budget of {} units ran out in {:?}",
                budget.limit,
                budget.exhausted_in.get()
            ),
        }
    }
}

impl std::error::Error for WorkError {}

thread_local! {
    /// The budget in force on this thread.
    static CURRENT: RefCell<Option<Rc<WorkBudget>>> = const { RefCell::new(None) };
}

/// The budget the work being done right now is charged to.
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

/// Holds the budget that was in force before
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
    /// A report of `phase`, the work spent and the fraction done.
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

/// Milliseconds of wall clock, for the progress throttle alone (no
/// result depends on it). Read off the meters' clock.
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
    /// fill), by phase discriminant.
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

    /// Where each phase starts on the bar - the weights ahead of it in [`PHASE_ORDER`], added up.
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
        let held = ieee_math::min_f64(1.0, ieee_math::max_f64(self.last_fraction.get(), fraction));
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

        group_start + (group_weight * refinement_share * ieee_math::min_f64(1.0, within))
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

        self.clamped(start + (weight * ieee_math::min_f64(1.0, ieee_math::max_f64(0.0, within))))
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

impl WorkBudget {
    /// The plain charge, when that is all it is: adds `units` and counts the charge toward the
    /// next cancellation check, answering true. Answers false, having changed nothing, when this
    /// charge would pass the limit or reach the cancellation check, which [`WorkBudget::charge`]
    /// then does in full.
    fn charge_when_plain(&self, units: i64) -> bool {
        let spent = self.spent.get() + units;
        let since = self.since_cancel_check.get() + 1;
        if spent > self.limit || since >= CANCEL_CHECK_EVERY {
            return false;
        }

        self.spent.set(spent);
        self.since_cancel_check.set(since);
        true
    }

    /// Charges work. When the budget runs out this answers [`WorkError::BudgetExceeded`], which
    /// the construction's own entry points turn into a refusal BY NAME - no caller of this module
    /// ever sees it, and no partial mesh is ever handed back. Every [`CANCEL_CHECK_EVERY`] charges
    /// it reads the cancellation token and, when the throttle allows, reports progress.
    pub fn charge(self: &Rc<Self>, units: i64) -> Result<(), WorkError> {
        self.spent.set(self.spent.get() + units);
        if self.spent.get() > self.limit {
            let phase = self.phase.get();
            self.exhausted_in.set(if phase == LevelLineWorkPhase::None {
                LevelLineWorkPhase::Refinement
            } else {
                phase
            });
            return Err(WorkError::BudgetExceeded(Rc::clone(self)));
        }

        let since = self.since_cancel_check.get() + 1;
        self.since_cancel_check.set(since);
        if since < CANCEL_CHECK_EVERY {
            return Ok(());
        }

        // A counter compare on the hot path, and the token and the clock read only once every few
        // hundred charges - far too little work to notice, and short enough that a cancel lands
        // within milliseconds.
        self.since_cancel_check.set(0);

        if self.cancellation_token.is_cancelled() {
            return Err(WorkError::Cancelled);
        }

        if self.progress.is_some()
            && tick_count_ms() - self.last_report_ms.get() >= self.report_throttle_ms
        {
            self.report_now();
        }

        Ok(())
    }
}

impl LevelLineWork {
    /// Charges units of work to the budget in force; nothing when there is none (callers pass
    /// `units = 1` for a single step).
    pub fn charge(units: i64) -> Result<(), WorkError> {
        // The common charge touches two counters and nothing else, so it is answered from the
        // slot without cloning the budget's `Rc`; the charge that reaches the limit or the
        // cancellation check takes the full path.
        let answered = CURRENT.with(|slot| {
            let slot = slot.borrow();
            match slot.as_deref() {
                None => true,
                Some(budget) => budget.charge_when_plain(units),
            }
        });
        if answered {
            return Ok(());
        }

        match Self::current() {
            Some(budget) => budget.charge(units),
            None => Ok(()),
        }
    }

    /// Leaves at once if the build in force has been cancelled, WITHOUT charging anything - for
    /// loops that are long but charged coarsely (or not at all), where adding charges would move
    /// the work-unit totals the ratchet tests pin. A token read and nothing else, so it may sit in
    /// an inner loop.
    pub fn poll_cancel() -> Result<(), WorkError> {
        let cancelled = CURRENT.with(|slot| {
            let slot = slot.borrow();
            match slot.as_deref() {
                Some(budget) => budget.cancellation_token.is_cancelled(),
                None => false,
            }
        });
        if cancelled {
            Err(WorkError::Cancelled)
        } else {
            Ok(())
        }
    }
}

/// The refusal from the `WorkBudget` constructors: a limit that is not a positive count of work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NonPositiveWorkLimit(pub i64);

impl std::fmt::Display for NonPositiveWorkLimit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "a work budget is a count of work, so it must be positive (limit {})",
            self.0
        )
    }
}

impl std::error::Error for NonPositiveWorkLimit {}

impl WorkBudget {
    /// An allowance with no cancellation and no progress report.
    pub fn new(limit: i64) -> Result<WorkBudget, NonPositiveWorkLimit> {
        Self::with_cancellation(limit, CancelToken::default(), None)
    }

    /// An allowance that also
    /// carries the caller's cancellation and, where it wants one, a progress report. The token is
    /// read every `CancelCheckEvery` charges, so a cancel lands within a few milliseconds of work
    /// wherever the build has got to; the report is called on every phase change and at most every
    /// [`WorkBudget::report_throttle_ms`] between. Wall clock is allowed there alone: it only
    /// decides how often the bar is told, and no result depends on it. The settable options
    /// (`input_is_float_stored`, `report_throttle_ms`) are the public fields, set before the budget
    /// is shared. The per-flow switches start at their defaults.
    pub fn with_cancellation(
        limit: i64,
        cancellation_token: CancelToken,
        progress: Option<LevelLineProgressReport>,
    ) -> Result<WorkBudget, NonPositiveWorkLimit> {
        if limit <= 0 {
            return Err(NonPositiveWorkLimit(limit));
        }

        Ok(WorkBudget {
            cancellation_token,
            progress,
            limit,
            input_is_float_stored: false,
            report_throttle_ms: 50,
            pair_corner_end_face_field_off: Cell::new(false),
            both_sided_members_off: Cell::new(false),
            run_outs_on: Cell::new(super::band_group_fields::flow_run_outs_on()),
            square_crease_clips_off: Cell::new(false),
            leaning_crease_clips_off: Cell::new(false),
            stopped_ends_on: Cell::new(super::stopped_end_clip::flow_stopped_ends_on()),
            watch_hole_fills: RefCell::new(None),
            spent: Cell::new(0),
            phase: Cell::new(LevelLineWorkPhase::None),
            exhausted_in: Cell::new(LevelLineWorkPhase::None),
            since_cancel_check: Cell::new(0),
            last_report_ms: Cell::new(0),
            last_fraction: Cell::new(0.0),
            side_group_index: Cell::new(0),
            side_group_count: Cell::new(0),
            refinement_taken: Cell::new(0),
            refinement_left: Cell::new(0),
            phase_started_at: Cell::new(0),
            phase_estimates: RefCell::new([0.0; LEVEL_LINE_WORK_PHASE_COUNT]),
            phase_units: RefCell::new([0; LEVEL_LINE_WORK_PHASE_COUNT]),
            phase_ticks: RefCell::new([0; LEVEL_LINE_WORK_PHASE_COUNT]),
            phase_tally_units_at: Cell::new(0),
            phase_tally_ticks_at: Cell::new(0),
        })
    }

    /// True once the budget has run out; the build that owns it has
    /// refused by then.
    pub fn exhausted(&self) -> bool {
        self.exhausted_in.get() != LevelLineWorkPhase::None
    }

    /// A progress hook for a search this build runs on an
    /// allowance of its own (the pinch ladder): the search's fraction is mapped into what is left of
    /// the phase running now, so the bar moves through the search and never past the phase's end.
    /// `None` when nobody is watching this build.
    pub fn a_slice_for_a_nested_search(self: &Rc<Self>) -> Option<LevelLineProgressReport> {
        self.progress.as_ref()?;

        let from = self.last_fraction.get();
        let phase = self.phase.get();
        let starts = Self::the_starts();
        let weights = Self::the_weights();
        let mut end = starts[phase as usize] + weights[phase as usize];

        if self.side_group_count.get() > 0
            && (phase == LevelLineWorkPhase::Refinement || phase == LevelLineWorkPhase::EdgeRoots)
        {
            // Inside the per-group loop the phase's span is the whole loop's; the search stays in
            // its group's slice.
            let span = weights[LevelLineWorkPhase::Refinement as usize]
                + weights[LevelLineWorkPhase::EdgeRoots as usize];
            end = starts[LevelLineWorkPhase::Refinement as usize]
                + (span * f64::from(self.side_group_index.get() + 1)
                    / f64::from(self.side_group_count.get()));
        }

        let to = ieee_math::max_f64(from, end);
        let budget = Rc::clone(self);
        Some(Rc::new(move |progress: LevelLineProgress| {
            budget.last_report_ms.set(tick_count_ms());
            let fraction = budget.clamped(
                from + ((to - from)
                    * ieee_math::min_f64(1.0, ieee_math::max_f64(0.0, progress.fraction))),
            );
            if let Some(report) = budget.progress.as_ref() {
                report(LevelLineProgress::new(
                    budget.phase.get(),
                    budget.spent.get(),
                    fraction,
                ));
            }
        }))
    }

    /// Says which side group the refinement and edge roots about to
    /// run belong to, of how many, so the bar walks across the groups.
    pub fn enter_side_group(&self, index: i32, count: i32) {
        self.side_group_count.set(0.max(count));
        self.side_group_index.set(0.max(index.min(count - 1)));
    }

    /// Notes the refinement's own worklist - how many triangles it
    /// has taken and how many are waiting. Only the bar reads it: nothing is charged and no result
    /// depends on it.
    pub fn note_refinement_queue(&self, taken: i64, left: i64) {
        self.refinement_taken.set(taken);
        self.refinement_left.set(left);
    }

    /// Adds the units and stopwatch ticks spent since the last tally
    /// to the running phase (the per-phase meter, kept only under `FieldCounters.MeterOn`).
    pub fn tally_phase(&self) {
        let now = stopwatch::get_timestamp();
        if self.phase_tally_ticks_at.get() != 0 {
            let phase = self.phase.get() as usize;
            self.phase_units.borrow_mut()[phase] +=
                self.spent.get() - self.phase_tally_units_at.get();
            self.phase_ticks.borrow_mut()[phase] += now - self.phase_tally_ticks_at.get();
        }

        self.phase_tally_units_at.set(self.spent.get());
        self.phase_tally_ticks_at.set(now);
    }
}

impl LevelLineWork {
    /// Names the side group whose refinement runs next on the
    /// budget in force ([`WorkBudget::enter_side_group`]).
    pub fn enter_side_group(index: i32, count: i32) {
        if let Some(budget) = Self::current() {
            budget.enter_side_group(index, count);
        }
    }

    /// Tells the budget in force how far the refinement's
    /// worklist has got ([`WorkBudget::note_refinement_queue`]).
    pub fn note_refinement_queue(taken: i64, left: i64) {
        if let Some(budget) = Self::current() {
            budget.note_refinement_queue(taken, left);
        }
    }
}

/// Every phase in declaration order.
pub const LEVEL_LINE_WORK_PHASES: [LevelLineWorkPhase; LEVEL_LINE_WORK_PHASE_COUNT] = [
    LevelLineWorkPhase::None,
    LevelLineWorkPhase::Sides,
    LevelLineWorkPhase::Refinement,
    LevelLineWorkPhase::EdgeRoots,
    LevelLineWorkPhase::BandDelete,
    LevelLineWorkPhase::Room,
    LevelLineWorkPhase::InputStrips,
    LevelLineWorkPhase::GapLoops,
    LevelLineWorkPhase::Fill,
    LevelLineWorkPhase::CutRoom,
];

impl WorkBudget {
    /// Units and wall seconds per phase of the builds this budget
    /// metered (only with `FieldCounters.MeterOn`); closes the phase running now, so read it once
    /// the build is done.
    ///
    /// The numbers are written in the invariant format (`N0`, `P1` as "12.3 %", `F2`), so the
    /// report reads the same on every machine.
    pub fn phase_report(&self) -> String {
        self.tally_phase();
        let units = *self.phase_units.borrow();
        let ticks = *self.phase_ticks.borrow();
        let total_units: i64 = units.iter().sum();
        let total_ticks: i64 = ticks.iter().sum();
        let mut text = format!(
            "{:<14}{:>14}{:>12}{:>10}{:>12}\n",
            "phase", "units", "unit share", "seconds", "time share"
        );
        for phase in LEVEL_LINE_WORK_PHASES {
            let i = phase as usize;
            if units[i] == 0 && ticks[i] == 0 {
                continue;
            }

            text.push_str(&format!(
                "{:<14}{:>14}{:>12}{:>10}{:>12}\n",
                format!("{phase:?}"),
                number_format::format_i64_number(units[i]),
                number_format::format_f64_percent(units[i] as f64 / 1.max(total_units) as f64, 1),
                number_format::format_f64_fixed(stopwatch::seconds(ticks[i]), 2),
                number_format::format_f64_percent(ticks[i] as f64 / 1.max(total_ticks) as f64, 1),
            ));
        }

        text.push_str(&format!(
            "{:<14}{:>14}{:>12}{:>10}",
            "total",
            number_format::format_i64_number(total_units),
            "",
            number_format::format_f64_fixed(stopwatch::seconds(total_ticks), 2),
        ));
        text
    }
}

impl WorkBudget {
    /// Moves the build to `phase` - the old phase's meter tallied and the
    /// rule clock stopped (when metered), the phase's start noted, the refinement's queue cleared
    /// on entering it, the side groups forgotten on leaving the per-group loop, and the bar told.
    pub fn enter(&self, phase: LevelLineWorkPhase) {
        if phase == self.phase.get() {
            return;
        }

        if super::FieldCounters::meter_on() {
            self.tally_phase();
            super::FieldCounters::stop_rule_clock();
        }

        self.phase.set(phase);
        self.phase_started_at.set(self.spent.get());

        if phase == LevelLineWorkPhase::Refinement {
            self.refinement_taken.set(0);
            self.refinement_left.set(0);
        } else if phase != LevelLineWorkPhase::EdgeRoots && phase != LevelLineWorkPhase::CutRoom {
            // Out of the per-group loop: the phases after it own their global spans again.
            self.side_group_count.set(0);
        }

        // A phase change always reports, throttle or no.
        self.report_now();
    }
}

impl LevelLineWork {
    /// Moves the budget in force, if any, to `phase`
    /// ([`WorkBudget::enter`]).
    pub fn enter(phase: LevelLineWorkPhase) {
        if let Some(budget) = Self::current() {
            budget.enter(phase);
        }
    }
}
