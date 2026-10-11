//! agg-sharp `agg/ProgressReporter.cs`: the progress channel a long running job reports through,
//! a synchronous [`ProgressReporter::report`] that is safe from anywhere plus a throttled
//! [`ProgressReporter::yield_to_ui`] that only costs something on a host where the job and the UI
//! share one thread. [`ProgressReporter::scaled`] carves a job's 0..1 budget into phases that
//! share one throttle.
//!
//! C# reads the host's yield from the process-global `ProgressReporter.UiYield`. Here the host
//! hands its yield to each `yield_to_ui` call instead (divergence 4252), the way the model holds
//! its yield hook per document (`mattercad_model::objects3d::ui_yield`): nothing process-wide to
//! install or restore, and a reporter decides only whether this report is worth a hop. C#'s
//! implicit conversions to and from `Action<double, string>` have no counterpart; a reporter is
//! built from a closure with [`ProgressReporter::new`].

use std::sync::{Arc, Mutex, OnceLock};
use web_time::{Duration, Instant};

/// What a reporter delivers to: the ratio (0 to 1) and what the job is doing now.
pub type ReportTarget = Arc<dyn Fn(f64, &str) + Send + Sync>;

/// C# `YieldThrottleMs`: the shortest gap between two real UI yields.
///
/// A yield is not free: on the browser it is a two idle turn hop back to the event loop, so it
/// costs on the order of one to two frames. Jobs report far more often than that - some per
/// triangle - and yielding on every report would spend more of the job hopping the loop than
/// working. At 50 ms the bar still updates 20 times a second, well past the ~10 per second where
/// motion already reads as smooth, while the hop overhead stays a few percent of a job of any
/// interesting length.
pub const YIELD_THROTTLE_MS: u64 = 50;

/// C# `ProgressReporter`. Cloning shares the reporter (C#'s reference): a clone reports to the
/// same target and spends the same throttle.
#[derive(Clone)]
pub struct ProgressReporter {
    inner: Arc<Inner>,
}

struct Inner {
    target: Option<ReportTarget>,
    /// The reporter this one yields through, for a child made by `scaled`; `None` for a reporter
    /// that owns its own throttle.
    yield_parent: Option<ProgressReporter>,
    /// When the last real yield happened. `None` to start (C# starts its tick stamp at zero), so
    /// the first yield of a job always goes through and the bar paints before the work begins.
    last_yield: Mutex<Option<Instant>>,
}

impl ProgressReporter {
    /// C# `new ProgressReporter(target)`; a `None` target reports nowhere and never yields.
    pub fn new(target: Option<ReportTarget>) -> Self {
        Self::with_parent(target, None)
    }

    /// A reporter delivering to `target` (C#'s conversion from an `Action`).
    pub fn from_fn(target: impl Fn(f64, &str) + Send + Sync + 'static) -> Self {
        Self::new(Some(Arc::new(target)))
    }

    fn with_parent(target: Option<ReportTarget>, yield_parent: Option<ProgressReporter>) -> Self {
        Self {
            inner: Arc::new(Inner {
                target,
                yield_parent,
                last_yield: Mutex::new(None),
            }),
        }
    }

    /// C# `ProgressReporter.Null`: the one reporter that swallows every report and never yields,
    /// so callers do not have to carry an `Option`. Every call hands back the same reporter.
    pub fn null() -> Self {
        static NULL: OnceLock<ProgressReporter> = OnceLock::new();
        NULL.get_or_init(|| ProgressReporter::new(None)).clone()
    }

    /// Whether `self` and `other` are the same reporter (C# `ReferenceEquals`).
    pub fn ptr_eq(&self, other: &ProgressReporter) -> bool {
        Arc::ptr_eq(&self.inner, &other.inner)
    }

    /// C# `HasTarget`: whether anything is actually listening. False for [`Self::null`] and for
    /// any reporter built around no target - the two shapes "nobody is watching" arrives in.
    /// Callers that have to tell a watched job from an unwatched one test this rather than
    /// comparing against the null reporter, which `ProgressReporter::new(None)` would slip past.
    pub fn has_target(&self) -> bool {
        self.inner.target.is_some()
    }

    /// C# `Scaled(amount, offset)`: a reporter that maps a 0..1 ratio into a slice of this one's
    /// budget - `offset + ratio * amount` - what a phase of a job is handed so it can report 0..1
    /// of its own work without knowing where in the whole it sits. The message is the child's,
    /// untouched.
    ///
    /// The child yields through its parent rather than keeping a throttle of its own, and that is
    /// the point of this method existing instead of a hand written wrapper. A wrapper starts life
    /// never having yielded, so its first `yield_to_ui` always hops the event loop; a job that
    /// makes one per item - a load with four hundred cached mesh links, say - then pays four
    /// hundred unthrottled hops for work that takes no time at all. Sharing the parent's state
    /// keeps the throttle a budget per job however many children the job carves it into.
    ///
    /// A reporter nobody is watching hands back itself: it reports nowhere and never yields, so
    /// there is nothing for a child to do differently.
    pub fn scaled(&self, amount: f64, offset: f64) -> ProgressReporter {
        if self.inner.target.is_none() {
            return self.clone();
        }
        let parent = self.clone();
        let target: ReportTarget = Arc::new(move |ratio: f64, message: &str| {
            parent.report(offset + (ratio * amount), message)
        });
        Self::with_parent(Some(target), Some(self.clone()))
    }

    /// C# `Report`: reports progress to whatever this reporter wraps. Synchronous and safe from
    /// any thread.
    pub fn report(&self, ratio: f64, message: &str) {
        if let Some(target) = &self.inner.target {
            target(ratio, message);
        }
    }

    /// C# `YieldToUi`, with the host's yield passed in (`None` on a host that needs none - the
    /// desktop path). Runs `ui_yield` and returns what it gave back (the hop to await, for an
    /// async host) when this host yields, this reporter has a target, and a full
    /// [`YIELD_THROTTLE_MS`] window has passed since the job's last real yield; `None` otherwise,
    /// with nothing run.
    ///
    /// Call this only from a job's top level flow. Calling it from inside a parallel body or a
    /// kernel callback is a bug - those are not the job's flow, and on the browser they are not
    /// even the thread that paints.
    pub fn yield_to_ui<T>(&self, ui_yield: Option<&dyn Fn() -> T>) -> Option<T> {
        // A scaled child has no throttle of its own - the whole job shares the one at the top, so
        // a job that carves its budget into many slices cannot buy itself extra hops by making
        // more of them.
        if let Some(parent) = &self.inner.yield_parent {
            return parent.yield_to_ui(ui_yield);
        }

        // No hook costs nothing. A reporter with no target never yields either: nothing is
        // showing this job's progress, so there is no frame worth painting.
        let ui_yield = ui_yield?;
        self.inner.target.as_ref()?;

        let now = Instant::now();
        {
            let mut last_yield = self
                .inner
                .last_yield
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            if let Some(last) = *last_yield {
                if now.duration_since(last) < Duration::from_millis(YIELD_THROTTLE_MS) {
                    return None;
                }
            }
            // Stamped before the hop, not after: the throttle is about how often we interrupt the
            // job, and timing from the start keeps a slow pump from stretching the gap further
            // than asked.
            *last_yield = Some(now);
        }
        Some(ui_yield())
    }

    /// C# `ReportAndYield`: [`Self::report`] followed by [`Self::yield_to_ui`] - the usual shape
    /// at a job's progress points.
    pub fn report_and_yield<T>(
        &self,
        ratio: f64,
        message: &str,
        ui_yield: Option<&dyn Fn() -> T>,
    ) -> Option<T> {
        self.report(ratio, message);
        self.yield_to_ui(ui_yield)
    }
}

impl std::fmt::Debug for ProgressReporter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ProgressReporter")
            .field("has_target", &self.has_target())
            .field("scaled", &self.inner.yield_parent.is_some())
            .finish()
    }
}
