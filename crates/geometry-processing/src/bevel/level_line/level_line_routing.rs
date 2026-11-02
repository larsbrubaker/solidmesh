//! Which route the app's bevel build takes, and the watch it can leave behind: the watched pick
//! ([`LevelLineChainWatch`]), the per-thread switch ([`use_level_line`],
//! [`reset_to_the_default`], [`turn_on_for_the_running_app`]), its route diagnostics
//! ([`last_route`], [`first_route`]) and the switch's readings of a setting ([`reads_as_on`],
//! [`reads_as_off`], [`the_running_app_wants_it_on`]) and its test seams ([`watch`],
//! [`watch_room`], [`watch_budget`], [`fail_the_crest_attempt`]).
//!
//! The switch's default is process-wide and read once from the developer variable, but a write
//! ([`set_use_level_line`]) holds only for the thread that made it: tests run on threads of one
//! process, and a test that pins the older construction for its own build must not change the
//! construction another test's build takes beside it. Work handed to another thread carries the
//! writing thread's choice along ([`LevelLineChoice::current`] on the submitting thread,
//! [`LevelLineChoice::enter`] on the worker; the model's compute pool does this for every job), so
//! a build sees the switch its own test set wherever its compute runs. The route strings and the
//! chain and room watches are a record each thread starts with its own of, and the same choice
//! carries it along, so the route a build took and the picks it was about to build reach the test
//! that asked for that build and no other; it sits behind a mutex because a carried record is
//! written on the worker and read on the asking thread. The budget watch is per thread and not
//! carried, because the allowances it is handed are single-threaded (`Rc`).

use std::cell::{Cell, RefCell};
use std::sync::{Arc, LazyLock, Mutex, MutexGuard};

use vector_math::Vector3;

/// The environment variable naming the opt-out.
pub const USE_LEVEL_LINE_VARIABLE: &str = "MATTERCAD_BEVEL_LEVELLINE";

/// The route string a pass the level line built writes.
pub const LEVEL_LINE_ROUTE: &str = "level-line";

/// The developer variable's value, or `None` where it is unset (always, in the browser).
fn use_level_line_setting() -> Option<String> {
    std::env::var(USE_LEVEL_LINE_VARIABLE).ok()
}

/// Whether the app route takes the level line, read from the developer variable once, on first
/// use: what every thread sees until it sets the switch itself.
static DEFAULT_USE_LEVEL_LINE: LazyLock<bool> =
    LazyLock::new(|| the_running_app_wants_it_on(use_level_line_setting().as_deref()));

/// One thread's route diagnostics and test watches, shared with the work it hands on.
#[derive(Default)]
struct RouteRecord {
    /// The last route taken ([`last_route`]).
    last: Option<String>,
    /// The first route since it was cleared ([`first_route`]).
    first: Option<String>,
    /// Whether the crest attempt is made to fail ([`fail_the_crest_attempt`]).
    crest_attempt_fails: bool,
    /// The chain watch ([`watch`]).
    chain_watch: Option<ChainWatch>,
    /// The room watch ([`watch_room`]).
    room_watch: Option<RoomWatch>,
}

type SharedRouteRecord = Arc<Mutex<RouteRecord>>;

thread_local! {
    // This thread's own setting of the switch; `None` follows the default.
    static USE_LEVEL_LINE: Cell<Option<bool>> = const { Cell::new(None) };

    // The record this thread's routes and watches live in: its own, or one carried in.
    static ROUTES: RefCell<SharedRouteRecord> = RefCell::new(SharedRouteRecord::default());
}

fn record() -> SharedRouteRecord {
    ROUTES.with(|routes| Arc::clone(&routes.borrow()))
}

fn locked(record: &Mutex<RouteRecord>) -> MutexGuard<'_, RouteRecord> {
    // Its slots are whole values, complete after any write, so a panic elsewhere cannot leave
    // one half-made: a poisoned lock's data is still good.
    record
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Whether a pass whose picked edges link into runs of mesh
/// edges - closed loops or open runs alike - goes through the level line. On from the start, in
/// tests exactly as in the running app, because the level line is what Surface mode does; a
/// developer comparing constructions turns it off with `MATTERCAD_BEVEL_LEVELLINE=0` (or "false").
/// Read on this thread: its own [`set_use_level_line`], or the one carried in by a
/// [`LevelLineChoice`], or else the default.
pub fn use_level_line() -> bool {
    USE_LEVEL_LINE
        .with(Cell::get)
        .unwrap_or(*DEFAULT_USE_LEVEL_LINE)
}

/// Sets [`use_level_line`] for this thread and the work it hands the compute pool: tests that pin
/// the older frozen construction set it false and restore it around one build, without touching
/// the switch other threads' builds read.
pub fn set_use_level_line(on: bool) {
    USE_LEVEL_LINE.with(|choice| choice.set(Some(on)));
}

/// The switch as one thread has it set and the record its routes and watches live in, captured
/// so work run on another thread builds, and reports, the way the thread that asked for it would.
#[derive(Clone)]
pub struct LevelLineChoice {
    use_level_line: Option<bool>,
    routes: SharedRouteRecord,
}

impl std::fmt::Debug for LevelLineChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LevelLineChoice")
            .field("use_level_line", &self.use_level_line)
            .finish_non_exhaustive()
    }
}

impl LevelLineChoice {
    /// This thread's setting (the default's when it has set none) and its record, right now.
    pub fn current() -> Self {
        Self {
            use_level_line: USE_LEVEL_LINE.with(Cell::get),
            routes: record(),
        }
    }

    /// Reads the captured setting, and writes routes and asks watches through the captured
    /// record, on this thread until the returned guard drops.
    pub fn enter(&self) -> LevelLineChoiceScope {
        let outer = USE_LEVEL_LINE.with(|choice| choice.replace(self.use_level_line));
        let outer_routes =
            ROUTES.with(|routes| std::mem::replace(&mut *routes.borrow_mut(), self.routes.clone()));
        LevelLineChoiceScope {
            outer,
            outer_routes: Some(outer_routes),
            _not_send: std::marker::PhantomData,
        }
    }
}

/// While alive, this thread builds by what a [`LevelLineChoice`] carried in; dropping it puts
/// back what the thread had before. It belongs to the thread that made it.
#[must_use = "the carried setting ends as soon as the scope is dropped"]
pub struct LevelLineChoiceScope {
    outer: Option<bool>,
    outer_routes: Option<SharedRouteRecord>,
    // A scope moved to another thread would restore that thread's setting wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl std::fmt::Debug for LevelLineChoiceScope {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LevelLineChoiceScope")
            .field("outer", &self.outer)
            .finish_non_exhaustive()
    }
}

impl Drop for LevelLineChoiceScope {
    fn drop(&mut self) {
        USE_LEVEL_LINE.with(|choice| choice.set(self.outer));
        if let Some(outer) = self.outer_routes.take() {
            ROUTES.with(|routes| *routes.borrow_mut() = outer);
        }
    }
}

/// Which way the last pass went - [`LEVEL_LINE_ROUTE`],
/// or "existing: " and why the level line did not take it - for a build this thread asked for. A
/// test seam only.
pub fn last_route() -> Option<String> {
    locked(&record()).last.clone()
}

/// Records the route, and makes it the
/// [`first_route`] too where none has been written since that was cleared.
pub fn set_last_route(value: Option<String>) {
    let record = record();
    let mut routes = locked(&record);
    if routes.first.is_none() {
        routes.first = value.clone();
    }

    routes.last = value;
}

/// The FIRST route written since this was last cleared to
/// `None` - the pick-wide call's answer, where [`last_route`] is overwritten by any later call in
/// the same build.
pub fn first_route() -> Option<String> {
    locked(&record()).first.clone()
}

/// Sets (or clears) [`first_route`].
pub fn set_first_route(value: Option<String>) {
    locked(&record()).first = value;
}

/// A test seam: makes this thread's crest attempts (`band_gap_crest::build_crest_in_pick`) fail
/// with a defect, carried to the worker like the routes, so a test can show a failed attempt
/// leaves the pick's own refusal word for word.
pub fn fail_the_crest_attempt(on: bool) {
    locked(&record()).crest_attempt_fails = on;
}

/// Whether [`fail_the_crest_attempt`] is set on this thread's record.
pub fn the_crest_attempt_fails() -> bool {
    locked(&record()).crest_attempt_fails
}

/// The default the running app wants - the level line on, unless the developer variable says off.
/// Called once at startup, on the thread that builds the app.
pub fn turn_on_for_the_running_app() {
    reset_to_the_default();
}

/// Puts this thread's switch back to the default it starts at - the same one the running app
/// wants - so a test that turned the level line off cannot hand that to the work after it.
pub fn reset_to_the_default() {
    USE_LEVEL_LINE.with(|choice| choice.set(None));
}

/// What [`watch`] is handed: every pick the app route is about to give the kernel.
pub type ChainWatch = Arc<dyn Fn(&LevelLineChainWatch) + Send + Sync>;

/// What [`watch_room`] is handed: the known room the app route is about to give the build.
pub type RoomWatch = Arc<dyn Fn(Option<f64>) + Send + Sync>;

/// The watch handed every chain the app route is about to give the
/// kernel, before it gives it. A test seam only - nothing in the app sets it, and `None` (the
/// normal case) costs one check per pass. It exists so a defect the SWEEP finds can be carried
/// into the kernel's own tests as a fixture without the test having to rebuild the app's pick,
/// features and working mesh for itself.
pub fn watch() -> Option<ChainWatch> {
    locked(&record()).chain_watch.clone()
}

/// Sets (or clears) [`watch`] for this thread's builds and the work it hands on.
pub fn set_watch(watch: Option<ChainWatch>) {
    locked(&record()).chain_watch = watch;
}

/// The watch handed the room the app route is about to give
/// the band gap build as its known room (`None` when it gives none), once that room is read. A test
/// seam like [`watch`], and separate from it because [`watch`] fires BEFORE the room read - a dump
/// may cancel there and never pay for it.
pub fn watch_room() -> Option<RoomWatch> {
    locked(&record()).room_watch.clone()
}

/// Sets (or clears) [`watch_room`] for this thread's builds and the work it hands on.
pub fn set_watch_room(watch: Option<RoomWatch>) {
    locked(&record()).room_watch = watch;
}

/// The whole PICK exactly as the app route hands it to the band gap build,
/// for [`watch`]. Plain data so a test can write it out and read it back.
///
/// A pick is a LIST of chains with a closedness each, because the route builds them in one call -
/// a face rim with a hole in it arrives here as two chains, not as two watches. The route's usual
/// `seam_epsilon` is 0 and `sharp_turn_degrees` positive infinity; the other members have no
/// usual value, so no `Default` is written here.
#[derive(Debug, Clone, PartialEq)]
pub struct LevelLineChainWatch {
    /// The pick's name, as the route labels it; `None` where the route names none,
    /// which the app route never does.
    pub pick: Option<String>,
    /// The mesh's vertex positions.
    pub positions: Vec<Vector3>,
    /// The mesh's triangles, as three vertex indices each.
    pub indices: Vec<(i32, i32, i32)>,
    /// The picked chains, each a run of vertex indices.
    pub chains: Vec<Vec<i32>>,
    /// Whether each chain is closed.
    pub closed: Vec<bool>,
    /// The feature edges, each as (low, high) vertex indices.
    pub feature_edges: Vec<(i32, i32)>,
    /// The bevel size asked for.
    pub size: f64,
    /// How many segments a fillet's arc is laid in.
    pub fillet_segments: i32,
    /// True for a round (fillet) profile, false for a chamfer.
    pub round_profile: bool,
    /// The seam epsilon the route passed (usually 0).
    pub seam_epsilon: f64,
    /// The sharp-turn angle the route passed, in degrees (usually positive infinity).
    pub sharp_turn_degrees: f64,
}

/// Whether a setting's text reads as ON: "1", or "true" in any case.
pub fn reads_as_on(value: &str) -> bool {
    value == "1" || value.eq_ignore_ascii_case("true")
}

/// Whether a setting's value asks for a switch to be OFF: "0" or
/// "false" in any casing. An unset variable (`None`) asks for nothing.
pub fn reads_as_off(value: Option<&str>) -> bool {
    matches!(value, Some(v) if v == "0" || v.eq_ignore_ascii_case("false"))
}

/// Whether the running app takes the level line,
/// given what the developer variable says - on unless it plainly says off, because the level line
/// is what Surface mode does. A plain function of the value rather than a read of the
/// environment, so a test can assert both directions without writing a process-wide variable.
pub fn the_running_app_wants_it_on(value: Option<&str>) -> bool {
    !reads_as_off(value)
}

thread_local! {
    /// The budget watch. The allowances are single-threaded (`Rc`), so the seam is
    /// held per thread - the kernel runs a build on one thread, and the test that sets the watch
    /// runs that build on its own.
    static WATCH_BUDGET: std::cell::RefCell<Option<BudgetWatch>> =
        const { std::cell::RefCell::new(None) };
}

/// What [`watch_budget`] is handed: every work allowance the kernel makes, as it is made.
pub type BudgetWatch = std::rc::Rc<dyn Fn(&std::rc::Rc<super::WorkBudget>)>;

/// The watch handed every allowance, or `None`.
pub fn watch_budget() -> Option<BudgetWatch> {
    WATCH_BUDGET.with(|w| w.borrow().clone())
}

/// Sets (or clears) [`watch_budget`]. A test seam like the chain watch: a test sums
/// `WorkBudget::spent` over a rebuild's allowances to judge how much WORK a build did.
pub fn set_watch_budget(watch: Option<BudgetWatch>) {
    WATCH_BUDGET.with(|w| *w.borrow_mut() = watch);
}
