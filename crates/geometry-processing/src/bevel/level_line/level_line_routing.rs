//! Which route the app's bevel build takes, and the watch it can leave behind: the watched pick
//! ([`LevelLineChainWatch`]), the per-thread switch ([`use_level_line`],
//! [`reset_to_the_default`], [`turn_on_for_the_running_app`]), its route diagnostics
//! ([`last_route`], [`first_route`]) and the switch's readings of a setting ([`reads_as_on`],
//! [`reads_as_off`], [`the_running_app_wants_it_on`]) and its test seams ([`watch`],
//! [`watch_room`], [`watch_budget`]).
//!
//! The switch's default is process-wide and read once from the developer variable, but a write
//! ([`set_use_level_line`]) holds only for the thread that made it: tests run on threads of one
//! process, and a test that pins the older construction for its own build must not change the
//! construction another test's build takes beside it. Work handed to another thread carries the
//! writing thread's choice along ([`LevelLineChoice::current`] on the submitting thread,
//! [`LevelLineChoice::enter`] on the worker; the model's compute pool does this for every job), so
//! a build sees the switch its own test set wherever its compute runs. The route strings and the
//! chain and room watches stay process-wide behind a mutex, because the kernel runs on worker
//! threads. The budget watch is per thread, because the allowances it is handed are
//! single-threaded (`Rc`).

use std::cell::Cell;
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

thread_local! {
    // This thread's own setting of the switch; `None` follows the default.
    static USE_LEVEL_LINE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// The last route taken and the first since it was cleared.
static ROUTES: Mutex<(Option<String>, Option<String>)> = Mutex::new((None, None));

fn routes() -> MutexGuard<'static, (Option<String>, Option<String>)> {
    // The pair is two plain strings, whole after any write, so a panic elsewhere cannot leave it
    // half-made: a poisoned lock's data is still good.
    ROUTES
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

/// The switch as one thread has it set, captured so work run on another thread builds the way
/// the thread that asked for it would.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LevelLineChoice(Option<bool>);

impl LevelLineChoice {
    /// This thread's setting right now (the default's when it has set none).
    pub fn current() -> Self {
        Self(USE_LEVEL_LINE.with(Cell::get))
    }

    /// Reads the captured setting on this thread until the returned guard drops.
    pub fn enter(self) -> LevelLineChoiceScope {
        let outer = USE_LEVEL_LINE.with(|choice| choice.replace(self.0));
        LevelLineChoiceScope {
            outer,
            _not_send: std::marker::PhantomData,
        }
    }
}

/// While alive, this thread reads the setting a [`LevelLineChoice`] carried in; dropping it puts
/// back what the thread had before. It belongs to the thread that made it.
#[must_use = "the carried setting ends as soon as the scope is dropped"]
#[derive(Debug)]
pub struct LevelLineChoiceScope {
    outer: Option<bool>,
    // A scope moved to another thread would restore that thread's setting wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl Drop for LevelLineChoiceScope {
    fn drop(&mut self) {
        USE_LEVEL_LINE.with(|choice| choice.set(self.outer));
    }
}

/// Which way the last pass went - [`LEVEL_LINE_ROUTE`],
/// or "existing: " and why the level line did not take it. A test seam only.
pub fn last_route() -> Option<String> {
    routes().0.clone()
}

/// Records the route, and makes it the
/// [`first_route`] too where none has been written since that was cleared.
pub fn set_last_route(value: Option<String>) {
    let mut routes = routes();
    if routes.1.is_none() {
        routes.1 = value.clone();
    }

    routes.0 = value;
}

/// The FIRST route written since this was last cleared to
/// `None` - the pick-wide call's answer, where [`last_route`] is overwritten by any later call in
/// the same build.
pub fn first_route() -> Option<String> {
    routes().1.clone()
}

/// Sets (or clears) [`first_route`].
pub fn set_first_route(value: Option<String>) {
    routes().1 = value;
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

/// The chain watch and the room watch.
static WATCHES: Mutex<(Option<ChainWatch>, Option<RoomWatch>)> = Mutex::new((None, None));

fn watches() -> MutexGuard<'static, (Option<ChainWatch>, Option<RoomWatch>)> {
    // Two plain slots, whole after any write: a poisoned lock's data is still good.
    WATCHES
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// The watch handed every chain the app route is about to give the
/// kernel, before it gives it. A test seam only - nothing in the app sets it, and `None` (the
/// normal case) costs one check per pass. It exists so a defect the SWEEP finds can be carried
/// into the kernel's own tests as a fixture without the test having to rebuild the app's pick,
/// features and working mesh for itself.
pub fn watch() -> Option<ChainWatch> {
    watches().0.clone()
}

/// Sets (or clears) [`watch`].
pub fn set_watch(watch: Option<ChainWatch>) {
    watches().0 = watch;
}

/// The watch handed the room the app route is about to give
/// the band gap build as its known room (`None` when it gives none), once that room is read. A test
/// seam like [`watch`], and separate from it because [`watch`] fires BEFORE the room read - a dump
/// may cancel there and never pay for it.
pub fn watch_room() -> Option<RoomWatch> {
    watches().1.clone()
}

/// Sets (or clears) [`watch_room`].
pub fn set_watch_room(watch: Option<RoomWatch>) {
    watches().1 = watch;
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
