//! Whether a level-line pass reads its band's setbacks from the rolling ball, and the route the
//! last pass took: the per-thread switch ([`use_rolling_ball`], [`set_use_rolling_ball`],
//! [`reset_to_the_default`]), the choice carried onto worker threads ([`RollingBallChoice`]) and
//! the route record ([`last_route`], [`ROLLING_BALL_ROUTE`]).
//!
//! The switch is off unless the developer variable `MATTERCAD_BEVEL_ROLLINGBALL` reads as on ("1"
//! or "true"): the rolling ball is being built beside the level line's own setbacks and takes no
//! user's fillet until it is at parity. The default is read once per process; a write holds only
//! for the thread that made it, so a test that turns the ball on for its own build cannot change
//! the build another test runs beside it. Work handed to another thread carries the asking
//! thread's setting and route record along ([`RollingBallChoice::current`] on the submitting
//! thread, [`RollingBallChoice::enter`] on the worker; the model's compute pool does this for
//! every job beside the level line's own choice), so a build sees its own test's switch, and the
//! route its pass took reaches the test that asked for it, wherever its compute runs.
//!
//! The route is written by every pass that runs with the switch on: [`ROLLING_BALL_ROUTE`] where
//! the rolling ball decided the pass's setbacks, or [`ROLLING_BALL_REFUSED`] and why it did not
//! (`level_line::rolling_ball_setbacks`). A pass with the switch off writes nothing.
//!
//! **Strict.** With the switch on, an outer fillet pass the ball does not take is refused
//! (`BandGapRefusal::RollingBall`, the reason on the result's `rolling_ball_refusal`); it is never
//! built on the level line's own setbacks. The ball is being brought to parity, and a pass quietly
//! built the older way would pass every test while saying nothing about the ball. A caller reads
//! the reason off the result it was handed (`BandGapResult::rolling_ball_refusal`, carried on by
//! chains in turn and rounds first as `GapFillResult::rolling_ball_refusal`), never off this
//! record: the record is shared by every job one thread hands the compute pool, so two builds
//! running at once would read each other's. The route record is a test seam only.
//!
//! **Chamfers are outside the ball.** A chamfer is a flat cut at a distance along each face, which
//! no ball defines, so with the switch on a chamfer's pass is built exactly as with it off: never
//! offered to the ball, never refused by it, writing no route. That is a different tool, not a
//! fallback.

use std::cell::{Cell, RefCell};
use std::sync::{Arc, LazyLock, Mutex, MutexGuard};

use crate::bevel::level_line::level_line_routing::reads_as_on;

/// The environment variable that turns the rolling ball on for every thread that sets nothing.
pub const USE_ROLLING_BALL_VARIABLE: &str = "MATTERCAD_BEVEL_ROLLINGBALL";

/// The route string a pass whose setbacks the rolling ball decided writes.
pub const ROLLING_BALL_ROUTE: &str = "rolling-ball";

/// What the route of a pass the rolling ball refused starts with; the reason follows it.
pub const ROLLING_BALL_REFUSED: &str = "refused: ";

/// Whether the running app reads setbacks from the rolling ball, given the developer variable's
/// value: only where it plainly says on. A function of the value, so a test can check both
/// directions without writing a process-wide variable.
pub fn the_running_app_wants_it_on(value: Option<&str>) -> bool {
    value.is_some_and(reads_as_on)
}

/// The process default, read once from the developer variable (always unset in the browser).
static DEFAULT_USE_ROLLING_BALL: LazyLock<bool> = LazyLock::new(|| {
    the_running_app_wants_it_on(std::env::var(USE_ROLLING_BALL_VARIABLE).ok().as_deref())
});

type SharedRoute = Arc<Mutex<Option<String>>>;

thread_local! {
    // This thread's own setting; `None` follows the default.
    static USE_ROLLING_BALL: Cell<Option<bool>> = const { Cell::new(None) };

    // The record this thread's route lives in: its own, or one carried in.
    static ROUTE: RefCell<SharedRoute> = RefCell::new(SharedRoute::default());
}

fn route() -> SharedRoute {
    ROUTE.with(|route| Arc::clone(&route.borrow()))
}

fn locked(route: &Mutex<Option<String>>) -> MutexGuard<'_, Option<String>> {
    // The slot holds a whole value after any write, so a poisoned lock's data is still good.
    route
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Whether this thread's level-line passes read their setbacks from the rolling ball: its own
/// [`set_use_rolling_ball`], or the one carried in by a [`RollingBallChoice`], or else the default.
pub fn use_rolling_ball() -> bool {
    USE_ROLLING_BALL
        .with(Cell::get)
        .unwrap_or(*DEFAULT_USE_ROLLING_BALL)
}

/// Sets [`use_rolling_ball`] for this thread and the work it hands the compute pool.
pub fn set_use_rolling_ball(on: bool) {
    USE_ROLLING_BALL.with(|choice| choice.set(Some(on)));
}

/// Puts this thread's switch back to the process default, so a test that turned the ball on
/// cannot hand that to the work after it.
pub fn reset_to_the_default() {
    USE_ROLLING_BALL.with(|choice| choice.set(None));
}

/// The route the last pass with the switch on took, for a build this thread asked for: see the
/// file header. A test seam only.
pub fn last_route() -> Option<String> {
    locked(&route()).clone()
}

/// Records (or clears) [`last_route`].
pub fn set_last_route(value: Option<String>) {
    *locked(&route()) = value;
}

/// Records a pass the rolling ball refused: the route reads [`ROLLING_BALL_REFUSED`] and `why`.
pub fn record_refusal(why: &str) {
    set_last_route(Some(format!("{ROLLING_BALL_REFUSED}{why}")));
}

/// The switch as one thread has it set and the record its route lives in, captured so work run on
/// another thread builds, and reports, the way the thread that asked for it would.
#[derive(Clone)]
pub struct RollingBallChoice {
    use_rolling_ball: Option<bool>,
    route: SharedRoute,
}

impl std::fmt::Debug for RollingBallChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RollingBallChoice")
            .field("use_rolling_ball", &self.use_rolling_ball)
            .finish_non_exhaustive()
    }
}

impl RollingBallChoice {
    /// This thread's setting (`None` where it follows the default) and its route record.
    pub fn current() -> Self {
        Self {
            use_rolling_ball: USE_ROLLING_BALL.with(Cell::get),
            route: route(),
        }
    }

    /// Reads the captured setting, and writes the route through the captured record, on this
    /// thread until the returned guard drops.
    pub fn enter(&self) -> RollingBallChoiceScope {
        let outer = USE_ROLLING_BALL.with(|choice| choice.replace(self.use_rolling_ball));
        let outer_route =
            ROUTE.with(|route| std::mem::replace(&mut *route.borrow_mut(), self.route.clone()));
        RollingBallChoiceScope {
            outer,
            outer_route: Some(outer_route),
            _not_send: std::marker::PhantomData,
        }
    }
}

/// While alive, this thread builds by what a [`RollingBallChoice`] carried in; dropping it puts
/// back what the thread had before. It belongs to the thread that made it.
#[must_use = "the carried setting ends as soon as the scope is dropped"]
pub struct RollingBallChoiceScope {
    outer: Option<bool>,
    outer_route: Option<SharedRoute>,
    // A scope moved to another thread would restore that thread's setting wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl std::fmt::Debug for RollingBallChoiceScope {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RollingBallChoiceScope")
            .field("outer", &self.outer)
            .finish_non_exhaustive()
    }
}

impl Drop for RollingBallChoiceScope {
    fn drop(&mut self) {
        USE_ROLLING_BALL.with(|choice| choice.set(self.outer));
        if let Some(outer) = self.outer_route.take() {
            ROUTE.with(|route| *route.borrow_mut() = outer);
        }
    }
}
