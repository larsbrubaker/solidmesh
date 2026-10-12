//! Which route the app's bevel build takes, and the watch it can leave behind. Ports the record of
//! `Bevel/LevelLine/LevelLineRouting.cs` ([`LevelLineChainWatch`]); the C# static class
//! `LevelLineRouting`'s process-wide switch ([`use_level_line`], [`reset_to_the_default`],
//! [`turn_on_for_the_running_app`]), its route diagnostics ([`last_route`], [`first_route`]) and the
//! switch's readings of a setting ([`reads_as_on`], [`reads_as_off`],
//! [`the_running_app_wants_it_on`]). The watch slots land with later packages.
//!
//! C#'s statics stay process-wide here (the bevel-core decision on process-wide settings), made
//! thread-safe because the kernel runs on worker threads: an atomic for the switch, a mutex for
//! the route strings.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{LazyLock, Mutex, MutexGuard};

use vector_math::Vector3;

/// C# `LevelLineRouting.UseLevelLineVariable`: the environment variable naming the opt-out.
pub const USE_LEVEL_LINE_VARIABLE: &str = "MATTERCAD_BEVEL_LEVELLINE";

/// C# `LevelLineRouting.LevelLineRoute`: the route string a pass the level line built writes.
pub const LEVEL_LINE_ROUTE: &str = "level-line";

/// The developer variable's value, or `None` where it is unset (always, in the browser).
fn use_level_line_setting() -> Option<String> {
    std::env::var(USE_LEVEL_LINE_VARIABLE).ok()
}

/// C# `LevelLineRouting.UseLevelLine` (static field, read once when the type is first touched).
static USE_LEVEL_LINE: LazyLock<AtomicBool> = LazyLock::new(|| {
    AtomicBool::new(the_running_app_wants_it_on(
        use_level_line_setting().as_deref(),
    ))
});

/// C#'s `lastRoute` and `FirstRoute`.
static ROUTES: Mutex<(Option<String>, Option<String>)> = Mutex::new((None, None));

fn routes() -> MutexGuard<'static, (Option<String>, Option<String>)> {
    // The pair is two plain strings, whole after any write, so a panic elsewhere cannot leave it
    // half-made: a poisoned lock's data is still good.
    ROUTES
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// C# `LevelLineRouting.UseLevelLine`: whether a pass whose picked edges link into runs of mesh
/// edges - closed loops or open runs alike - goes through the level line. On from the start, in
/// tests exactly as in the running app, because the level line is what Surface mode does; a
/// developer comparing constructions turns it off with `MATTERCAD_BEVEL_LEVELLINE=0` (or "false").
pub fn use_level_line() -> bool {
    USE_LEVEL_LINE.load(Ordering::SeqCst)
}

/// Sets [`use_level_line`] (C#'s field write): tests that pin the older frozen construction set it
/// false and restore it around one build.
pub fn set_use_level_line(on: bool) {
    USE_LEVEL_LINE.store(on, Ordering::SeqCst);
}

/// C# `LevelLineRouting.LastRoute` (getter): which way the last pass went - [`LEVEL_LINE_ROUTE`],
/// or "existing: " and why the level line did not take it. A test seam only.
pub fn last_route() -> Option<String> {
    routes().0.clone()
}

/// C# `LevelLineRouting.LastRoute` (setter): records the route, and makes it the
/// [`first_route`] too where none has been written since that was cleared.
pub fn set_last_route(value: Option<String>) {
    let mut routes = routes();
    if routes.1.is_none() {
        routes.1 = value.clone();
    }

    routes.0 = value;
}

/// C# `LevelLineRouting.FirstRoute`: the FIRST route written since this was last cleared to
/// `None` - the pick-wide call's answer, where [`last_route`] is overwritten by any later call in
/// the same build.
pub fn first_route() -> Option<String> {
    routes().1.clone()
}

/// Sets (or clears) [`first_route`] (C#'s `FirstRoute` setter).
pub fn set_first_route(value: Option<String>) {
    routes().1 = value;
}

/// C# `LevelLineRouting.TurnOnForTheRunningApp`: the default the running app wants - the level
/// line on, unless the developer variable says off. Called once at startup.
pub fn turn_on_for_the_running_app() {
    reset_to_the_default();
}

/// C# `LevelLineRouting.ResetToTheDefault`: puts the flag back to the default it starts at - the
/// same one the running app wants. The reset between Standard tests calls this, so a test that
/// turned the level line off cannot hand that to the tests after it.
pub fn reset_to_the_default() {
    set_use_level_line(the_running_app_wants_it_on(
        use_level_line_setting().as_deref(),
    ));
}

/// C# `LevelLineChainWatch`: the whole PICK exactly as the app route hands it to `BandGap.Build`,
/// for `LevelLineRouting.Watch`. Plain data so a test can write it out and read it back.
///
/// A pick is a LIST of chains with a closedness each, because the route builds them in one call -
/// a face rim with a hole in it arrives here as two chains, not as two watches. C#'s positional
/// record defaults `seam_epsilon` to 0 and `sharp_turn_degrees` to positive infinity; the other
/// members have no default, so none is written here.
#[derive(Debug, Clone, PartialEq)]
pub struct LevelLineChainWatch {
    /// The pick's name, as the route labels it.
    pub pick: String,
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
    /// The seam epsilon the route passed (C# default 0).
    pub seam_epsilon: f64,
    /// The sharp-turn angle the route passed, in degrees (C# default positive infinity).
    pub sharp_turn_degrees: f64,
}

/// Whether a setting's text reads as ON: "1", or "true" in any case.
pub fn reads_as_on(value: &str) -> bool {
    value == "1" || value.eq_ignore_ascii_case("true")
}

/// C# `LevelLineRouting.ReadsAsOff`: whether a setting's value asks for a switch to be OFF: "0" or
/// "false" in any casing. An unset variable (`None`, C#'s null) asks for nothing.
pub fn reads_as_off(value: Option<&str>) -> bool {
    matches!(value, Some(v) if v == "0" || v.eq_ignore_ascii_case("false"))
}

/// C# `LevelLineRouting.TheRunningAppWantsItOn`: whether the running app takes the level line,
/// given what the developer variable says - on unless it plainly says off, because the level line
/// is what Surface mode does. A plain function of the value rather than a read of the
/// environment, so a test can assert both directions without writing a process-wide variable.
pub fn the_running_app_wants_it_on(value: Option<&str>) -> bool {
    !reads_as_off(value)
}

thread_local! {
    /// C# `LevelLineRouting.WatchBudget`. The allowances are single-threaded (`Rc`), so the seam is
    /// held per thread - the kernel runs a build on one thread, and the test that sets the watch
    /// runs that build on its own.
    static WATCH_BUDGET: std::cell::RefCell<Option<BudgetWatch>> =
        const { std::cell::RefCell::new(None) };
}

/// What [`watch_budget`] is handed: every work allowance the kernel makes, as it is made.
pub type BudgetWatch = std::rc::Rc<dyn Fn(&std::rc::Rc<super::WorkBudget>)>;

/// C# `LevelLineRouting.WatchBudget` (get): the watch handed every allowance, or `None`.
pub fn watch_budget() -> Option<BudgetWatch> {
    WATCH_BUDGET.with(|w| w.borrow().clone())
}

/// C# `LevelLineRouting.WatchBudget` (set). A test seam like the chain watch: a test sums
/// `WorkBudget::spent` over a rebuild's allowances to judge how much WORK a build did.
pub fn set_watch_budget(watch: Option<BudgetWatch>) {
    WATCH_BUDGET.with(|w| *w.borrow_mut() = watch);
}
