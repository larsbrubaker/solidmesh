//! The level line's route diagnostics and test seams - nothing here chooses a route: which
//! constructions a build may take is the request's routing (`BevelRouting`, handed down the surface
//! part builder). What is here is what a test reads or plants: the route a build took
//! ([`last_route`], [`first_route`], written by the builder as it goes), the watched pick
//! ([`LevelLineChainWatch`] through [`watch`]), the known room ([`watch_room`]), the work
//! allowances ([`watch_budget`]) and two failure injections ([`fail_the_crest_attempt`],
//! [`fail_the_second_half`]). Nothing in a product sets any of them, and each costs a build one
//! check where it is unset.
//!
//! They are per-thread rather than on the request because they are a test's own instruments,
//! reaching the build through whatever drives it (a document's rebuild, the app's panel) without
//! every layer in between carrying them. The route strings, the failure injections and the chain
//! and room watches are a record each thread starts with its own of: tests run on threads of one
//! process, so the route a build took and the picks it was about to build reach the test that
//! asked for that build and no other. Work handed to another thread carries the record along
//! ([`LevelLineRouteRecord::current`] on the submitting thread, [`LevelLineRouteRecord::enter`] on
//! the worker; the model's compute pool does this for every job); it sits behind a mutex because
//! a carried record is written on the worker and read on the asking thread. The budget watch is
//! per thread and not carried, because the allowances it is handed are single-threaded (`Rc`).

use std::cell::RefCell;
use std::sync::{Arc, Mutex, MutexGuard};

use vector_math::Vector3;

/// The route string a pass the level line built writes.
pub const LEVEL_LINE_ROUTE: &str = "level-line";

/// One thread's route diagnostics and test watches, shared with the work it hands on.
#[derive(Default)]
struct RouteRecord {
    /// The last route taken ([`last_route`]).
    last: Option<String>,
    /// The first route since it was cleared ([`first_route`]).
    first: Option<String>,
    /// Whether the crest attempt is made to fail ([`fail_the_crest_attempt`]).
    crest_attempt_fails: bool,
    /// Whether the second half of a pass built in two is made to refuse
    /// ([`fail_the_second_half`]).
    second_half_fails: bool,
    /// The chain watch ([`watch`]).
    chain_watch: Option<ChainWatch>,
    /// The room watch ([`watch_room`]).
    room_watch: Option<RoomWatch>,
}

type SharedRouteRecord = Arc<Mutex<RouteRecord>>;

thread_local! {
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

/// The record one thread's routes and watches live in, captured so work run on another thread
/// reports, and is watched, the way the thread that asked for it would be. A test seam, like the
/// record itself.
#[derive(Clone)]
pub struct LevelLineRouteRecord {
    routes: SharedRouteRecord,
}

impl std::fmt::Debug for LevelLineRouteRecord {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LevelLineRouteRecord")
            .finish_non_exhaustive()
    }
}

impl LevelLineRouteRecord {
    /// This thread's record, right now.
    pub fn current() -> Self {
        Self { routes: record() }
    }

    /// Writes routes and asks watches through the captured record, on this thread until the
    /// returned guard drops.
    pub fn enter(&self) -> LevelLineRouteRecordScope {
        let outer_routes =
            ROUTES.with(|routes| std::mem::replace(&mut *routes.borrow_mut(), self.routes.clone()));
        LevelLineRouteRecordScope {
            outer_routes: Some(outer_routes),
            _not_send: std::marker::PhantomData,
        }
    }
}

/// While alive, this thread reports to and is watched through what a [`LevelLineRouteRecord`]
/// carried in; dropping it puts back what the thread had before. It belongs to the thread that
/// made it.
#[must_use = "the carried record ends as soon as the scope is dropped"]
pub struct LevelLineRouteRecordScope {
    outer_routes: Option<SharedRouteRecord>,
    // A scope moved to another thread would restore that thread's record wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl std::fmt::Debug for LevelLineRouteRecordScope {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LevelLineRouteRecordScope")
            .finish_non_exhaustive()
    }
}

impl Drop for LevelLineRouteRecordScope {
    fn drop(&mut self) {
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

/// A test seam: makes the second half of a pass built in two (the app's face rim built after its
/// partner) refuse, so the branch that keeps the pass's earlier answer can be reached. Off by
/// default; nothing in the app sets it.
pub fn fail_the_second_half(on: bool) {
    locked(&record()).second_half_fails = on;
}

/// Whether [`fail_the_second_half`] is set on this thread's record.
pub fn the_second_half_fails() -> bool {
    locked(&record()).second_half_fails
}

/// Whether [`fail_the_crest_attempt`] is set on this thread's record.
pub fn the_crest_attempt_fails() -> bool {
    locked(&record()).crest_attempt_fails
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
