//! The hard wall-clock guard every kernel call in the Thingi10K sweep runs behind: a kernel defect
//! shows up as a build that never returns, and no cancellation token can reach inside a loop that
//! never charges work.
//!
//! Besides abandoning a build that overruns, this also cancels the overrun build's token (a build
//! that charges work, which is every build that runs away through the budgeted loops, then ends on
//! its own within a few milliseconds) and waits two seconds for it, so a Hung row can carry the
//! work it spent: a `WorkBudget` is single-threaded (`Rc`/`Cell`) and is read only on the thread
//! that owns it. A build that does not end is abandoned and its row reads 0 work.
//!
//! Only abandoned builds are counted alive. A build that answered in time is not, even in the
//! moment between handing back its answer and its thread exiting, so the count read right after a
//! build returns is exact.

use std::any::Any;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use solidmesh::bevel::level_line::WorkBudget;
use solidmesh_mesh::CancelToken;

/// A build thread wants a real stack: the kernel's flood and refinement recurse nowhere near the
/// default, but a wedged one can, and a stack overflow takes the whole host down with it.
const BUILD_STACK_BYTES: usize = 32 * 1024 * 1024;

/// How long a build that was told to cancel gets to hand back the work it spent.
const GRACE: Duration = Duration::from_secs(2);

/// Abandoned builds still spinning, and which run of the sweep they count toward.
struct Abandoned {
    alive: i32,
    /// Bumped by `forget_abandoned`: a build abandoned in an earlier run that ends later must not
    /// take one off this run's count.
    run: u64,
}

/// The count and every build's state change under this one lock, so a build ending and the
/// watchdog giving up on it cannot interleave: the build is counted exactly when it was abandoned
/// and is still running.
static ABANDONED: Mutex<Abandoned> = Mutex::new(Abandoned { alive: 0, run: 0 });

/// Wall-clock milliseconds one build may take before it is abandoned as Hung (30 s by default).
static BUILD_CAP_MS: AtomicU64 = AtomicU64::new(30_000);

/// An error or panic out of a guarded call: its kind's name and what it said.
#[derive(Debug, Clone)]
pub struct Thrown {
    pub name: String,
    pub detail: String,
}

impl Thrown {
    pub fn new(name: &str, detail: impl Into<String>) -> Self {
        Thrown {
            name: name.to_string(),
            detail: detail.into(),
        }
    }
}

impl From<std::io::Error> for Thrown {
    fn from(error: std::io::Error) -> Self {
        Thrown::new("IOException", error.to_string())
    }
}

/// What one guarded call came back as: its value, or the class it ended in.
pub struct Guarded<T> {
    pub value: Option<T>,
    /// None when the work returned; else the class it returned as (Hung, or Threw:type).
    pub outcome: Option<String>,
    /// What a thrown exception said.
    pub detail: Option<String>,
    /// Work units the call spent, read off its budget.
    pub work: i64,
}

/// Seconds of wall clock one build may take before it is abandoned as Hung.
pub fn set_build_cap(cap: Duration) {
    BUILD_CAP_MS.store(cap.as_millis() as u64, Ordering::SeqCst);
}

pub fn build_cap() -> Duration {
    Duration::from_millis(BUILD_CAP_MS.load(Ordering::SeqCst))
}

/// How many abandoned builds are still spinning right now.
pub fn alive() -> i32 {
    abandoned().alive
}

/// Starts a run's count of abandoned builds from nothing.
pub fn forget_abandoned() {
    let mut abandoned = abandoned();
    abandoned.alive = 0;
    abandoned.run += 1;
}

/// Nothing panics while holding the lock, so a poisoned one still holds a sound count.
fn abandoned() -> MutexGuard<'static, Abandoned> {
    ABANDONED.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Where one build stands, as far as the count of abandoned builds is concerned.
#[derive(Clone, Copy)]
enum BuildState {
    Running,
    Ended,
    /// Given up on by the watchdog while still running, during the given run of the sweep.
    Abandoned {
        run: u64,
    },
}

/// One build's state, shared by its thread and the watchdog; only changed under `ABANDONED`.
type SharedState = Arc<Mutex<BuildState>>;

fn state_of(state: &SharedState) -> MutexGuard<'_, BuildState> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Marks the build ended when its thread finishes, whatever way it finishes; an abandoned one
/// leaves the count then.
struct EndGuard(SharedState);

impl Drop for EndGuard {
    fn drop(&mut self) {
        let mut abandoned = abandoned();
        let mut state = state_of(&self.0);
        if let BuildState::Abandoned { run } = *state {
            if run == abandoned.run {
                abandoned.alive -= 1;
            }
        }
        *state = BuildState::Ended;
    }
}

/// The watchdog gives up on a build: counted alive if it is still running, not if it ended
/// meanwhile.
fn abandon(state: &SharedState) {
    let mut abandoned = abandoned();
    let mut state = state_of(state);
    if let BuildState::Running = *state {
        *state = BuildState::Abandoned { run: abandoned.run };
        abandoned.alive += 1;
    }
}

enum Ended<T> {
    Done(T),
    Threw(Thrown),
}

/// Runs one build on a thread of its own, with a work budget of `limit`, under a hard wall-clock
/// limit. There is no way to stop a wedged build from outside, so a thread that overruns is
/// ABANDONED (the host does not wait on it) and counted alive until it finishes on its own.
pub fn run<T: Send + 'static>(
    limit: i64,
    work: impl FnOnce(Rc<WorkBudget>) -> Result<T, Thrown> + Send + 'static,
) -> Guarded<T> {
    let token = CancelToken::new();
    let build_token = token.clone();
    let (sender, receiver) = mpsc::channel::<(Ended<T>, i64)>();
    let state: SharedState = Arc::new(Mutex::new(BuildState::Running));
    let build_state = Arc::clone(&state);

    let spawned = std::thread::Builder::new()
        .stack_size(BUILD_STACK_BYTES)
        .spawn(move || {
            let _end = EndGuard(build_state);
            let budget = Rc::new(
                WorkBudget::with_cancellation(limit, build_token, None)
                    .expect("a sweep's work limit is positive"),
            );
            let ended = match catch_unwind(AssertUnwindSafe(|| work(Rc::clone(&budget)))) {
                Ok(Ok(value)) => Ended::Done(value),
                Ok(Err(thrown)) => Ended::Threw(thrown),
                Err(panic) => Ended::Threw(Thrown::new("Panic", panic_message(panic.as_ref()))),
            };
            let _ = sender.send((ended, budget.spent.get()));
        });
    if let Err(error) = spawned {
        return Guarded {
            value: None,
            outcome: Some("Threw:IOException".to_string()),
            detail: Some(error.to_string()),
            work: 0,
        };
    }

    match receiver.recv_timeout(build_cap()) {
        Ok((ended, spent)) => answer(ended, spent),
        Err(_) => {
            token.cancel();
            let work = match receiver.recv_timeout(GRACE) {
                Ok((_, spent)) => spent,
                Err(_) => {
                    abandon(&state);
                    0
                }
            };
            Guarded {
                value: None,
                outcome: Some("Hung".to_string()),
                detail: None,
                work,
            }
        }
    }
}

fn answer<T>(ended: Ended<T>, spent: i64) -> Guarded<T> {
    match ended {
        Ended::Done(value) => Guarded {
            value: Some(value),
            outcome: None,
            detail: None,
            work: spent,
        },
        Ended::Threw(thrown) => Guarded {
            value: None,
            outcome: Some(format!("Threw:{}", thrown.name)),
            detail: Some(thrown.detail),
            work: spent,
        },
    }
}

fn panic_message(panic: &(dyn Any + Send)) -> String {
    if let Some(text) = panic.downcast_ref::<&str>() {
        (*text).to_string()
    } else if let Some(text) = panic.downcast_ref::<String>() {
        text.clone()
    } else {
        "a panic with no message".to_string()
    }
}
