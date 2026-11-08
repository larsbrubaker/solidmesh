//! The ONE bar a bevel rebuild shows ([`BevelProgressBar`]): every kernel call the rebuild makes
//! reports into a slice of it, and the bar only ever moves forward. The rebuild opens its windows
//! and hands [`BevelProgressBar::slice`] to the level line's `WorkBudget`; what it reports is a
//! [`BevelProgress`] (`progress`), which the caller words.

use std::rc::Rc;
use std::sync::{Arc, Mutex, MutexGuard};

use super::progress::{
    BevelProgress, BevelProgressPhase, BevelProgressReport, BevelSizeSearchRung,
};
use crate::bevel::level_line::work_budget::{
    LevelLineProgress, LevelLineProgressReport, LevelLineWorkPhase,
};

#[derive(Debug)]
struct BarState {
    high_water: f64,
    window_start: f64,
    window_end: f64,
    attempt_end: f64,
    leaving_out: i32,
    smaller_size: Option<BevelSizeSearchRung>,
}

/// Each kernel call reports its own 0..1, and handed straight to the caller the bar would reset to
/// zero at every call (settle, room, build, fill) and at every rung of the size search. Instead
/// the rebuild opens a WINDOW of the bar for a piece of its work, a call's fraction maps into a
/// slice of that window, and a locked high-water mark keeps what the caller sees monotone - a
/// window that opens behind the mark holds the bar until its work catches up. A retry or a
/// size-search rung does not know how many more will follow it, so each takes a share of what is
/// LEFT of its span ([`Self::open_a_share_of`]): the bar keeps moving and never claims the end
/// before the work reaches it.
///
/// Cloning shares the bar, which is how the hooks it hands out reach it.
#[derive(Clone)]
pub struct BevelProgressBar {
    gate: Arc<Mutex<BarState>>,
    report: Option<BevelProgressReport>,
}

impl BevelProgressBar {
    /// A bar reporting to `report` (`None` for nobody).
    pub fn new(report: Option<BevelProgressReport>) -> Self {
        BevelProgressBar {
            gate: Arc::new(Mutex::new(BarState {
                high_water: 0.0,
                window_start: 0.0,
                window_end: 1.0,
                attempt_end: 1.0,
                leaving_out: 0,
                smaller_size: None,
            })),
            report,
        }
    }

    fn lock(&self) -> MutexGuard<'_, BarState> {
        // A report that panicked while holding the lock left plain numbers behind.
        self.gate
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// How far the bar has got, 0..1; it never goes down.
    pub fn high_water(&self) -> f64 {
        self.lock().high_water
    }

    /// How many loops the attempt open now left out (0 on a first attempt), which every phase it
    /// reports carries.
    pub fn leaving_out(&self) -> i32 {
        self.lock().leaving_out
    }

    /// The size-search rung open now, which stands in for every phase's report until the next
    /// attempt opens, or `None`.
    pub fn smaller_size(&self) -> Option<BevelSizeSearchRung> {
        self.lock().smaller_size
    }

    /// `doing` as the bar reports it now - with the attempt's loops left out and the rung open
    /// now.
    pub fn said(&self, doing: BevelProgressPhase) -> BevelProgress {
        let state = self.lock();
        BevelProgress {
            doing,
            leaving_out: state.leaving_out,
            smaller_size: state.smaller_size,
        }
    }

    /// Opens `share` of what is left between the bar's mark and `limit` as the window the next
    /// piece of work reports into.
    pub fn open_a_share_of(&self, limit: f64, share: f64) {
        let mut state = self.lock();
        let start = state.high_water.min(limit);
        state.window_start = start;
        state.window_end = start + ((limit - start) * share.clamp(0.0, 1.0));
    }

    /// Opens one pass attempt: `share` of what is left up to `limit`. Its kernel calls report into
    /// the first `kernel_part` of it and a size search that follows takes rungs out of the rest
    /// ([`Self::open_a_rung`]), so a search can never carry the bar past where the next attempt
    /// starts - otherwise a search on a many-hole plate pushed the bar past the passes' span and
    /// held it still through the last 77% of the rebuild. `leaving_out` is how many loops earlier
    /// attempts left out.
    pub fn open_an_attempt(&self, limit: f64, share: f64, kernel_part: f64, leaving_out: i32) {
        let mut state = self.lock();
        state.leaving_out = leaving_out;
        state.smaller_size = None;
        let start = state.high_water.min(limit);
        state.attempt_end = start + ((limit - start) * share.clamp(0.0, 1.0));
        state.window_start = start;
        state.window_end = start + ((state.attempt_end - start) * kernel_part.clamp(0.0, 1.0));
    }

    /// One size-search rung: `share` of what is left of the attempt open now, reported as `rung`
    /// until the next attempt opens.
    pub fn open_a_rung(&self, share: f64, rung: Option<BevelSizeSearchRung>) {
        let mut state = self.lock();
        state.smaller_size = rung;
        let start = state.high_water.min(state.attempt_end);
        state.window_start = start;
        state.window_end = start + ((state.attempt_end - start) * share.clamp(0.0, 1.0));
    }

    /// A report hook covering what is left of the bar up to `limit`, for a nested rebuild (a pass
    /// built in two) that runs a bar of its own from 0 to 1; its reports pass through as they are.
    pub fn the_rest_up_to(&self, limit: f64) -> BevelProgressReport {
        let from = self.high_water();
        let bar = self.clone();

        Arc::new(move |ratio: f64, progress: &BevelProgress| {
            bar.tell(
                from + ((from.max(limit) - from) * ratio.clamp(0.0, 1.0)),
                progress,
            )
        })
    }

    /// A kernel call's report hook, mapped into [`from`, `to`] of the window open now - read when
    /// the hook is made, so a window opened later does not move a call already running. Each
    /// report is `doing_for` the call's phase, as the bar says it when the report comes
    /// ([`Self::said`]).
    pub fn slice(
        &self,
        from: f64,
        to: f64,
        doing_for: impl Fn(LevelLineWorkPhase) -> BevelProgressPhase + 'static,
    ) -> LevelLineProgressReport {
        let (start, end) = {
            let state = self.lock();
            let span = state.window_end - state.window_start;
            (
                state.window_start + (span * from),
                state.window_start + (span * to),
            )
        };

        let bar = self.clone();
        Rc::new(move |progress: LevelLineProgress| {
            bar.tell(
                start + ((end - start) * progress.fraction.clamp(0.0, 1.0)),
                &bar.said(doing_for(progress.phase)),
            )
        })
    }

    /// Moves the bar to the end of the window open now: that piece of work is done.
    pub fn finish_the_window(&self, progress: &BevelProgress) {
        let end = self.lock().window_end;

        self.tell(end, progress);
    }

    /// Tells the caller this ratio, held at the high-water mark where it would go backwards.
    pub fn tell(&self, ratio: f64, progress: &BevelProgress) {
        let ratio = {
            let mut state = self.lock();
            state.high_water = state.high_water.max(ratio.min(1.0));
            state.high_water
        };

        if let Some(report) = &self.report {
            report(ratio, progress);
        }
    }
}
