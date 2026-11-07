//! Maps the kernel's per-boolean phase reports onto a caller's progress window. Used by
//! `manifold_kernel` (the batch combine) and `manifold_pairwise_fold` (one step per pair).
//!
//! The caller owns the window `[ratio_completed, ratio_completed + amount_per_operation]`,
//! split evenly over `operation_count` booleans. Each boolean restarts its phases at fraction
//! 0, so the bar holds a high-water mark and never goes backwards.

use std::sync::{Arc, Mutex};

use manifold_rust::progress::{Phase, ProgressReporter};

use super::BooleanReporter;

struct AdapterState {
    completed_operations: usize,
    high_water_ratio: f64,
}

pub(crate) struct BooleanProgressAdapter {
    reporter: BooleanReporter,
    window_start: f64,
    window_size: f64,
    operation_count: usize,
    operation_label: &'static str,
    state: Mutex<AdapterState>,
}

impl BooleanProgressAdapter {
    pub(crate) fn new(
        reporter: BooleanReporter,
        ratio_completed: f64,
        amount_per_operation: f64,
        operation_count: usize,
    ) -> Self {
        Self {
            reporter,
            window_start: ratio_completed,
            window_size: amount_per_operation,
            operation_count: operation_count.max(1),
            operation_label: "Boolean",
            state: Mutex::new(AdapterState {
                completed_operations: 0,
                high_water_ratio: ratio_completed,
            }),
        }
    }

    /// [`Self::new`] with an operation label prefixed to each phase (Minkowski passes "Dilate"
    /// or "Erode").
    pub(crate) fn with_label(
        reporter: BooleanReporter,
        ratio_completed: f64,
        amount_per_operation: f64,
        operation_count: usize,
        operation_label: &'static str,
    ) -> Self {
        Self {
            operation_label,
            ..Self::new(
                reporter,
                ratio_completed,
                amount_per_operation,
                operation_count,
            )
        }
    }

    /// One kernel report. A `None` fraction is an indeterminate phase: it says nothing about
    /// how far along the phase is, so only the message changes.
    pub(crate) fn report(&self, phase: &str, fraction: Option<f64>) {
        let mut state = self.lock();
        let step_size = self.window_size / self.operation_count as f64;
        let step_start = self.window_start + step_size * state.completed_operations as f64;
        let candidate = match fraction {
            Some(f) => step_start + step_size * f.clamp(0.0, 1.0),
            None => state.high_water_ratio,
        };
        self.publish(&mut state, candidate, phase);
    }

    /// Closes out one boolean of the window.
    pub(crate) fn complete_operation(&self, phase: &str) {
        let mut state = self.lock();
        if state.completed_operations < self.operation_count {
            state.completed_operations += 1;
        }
        let candidate = self.window_start
            + self.window_size / self.operation_count as f64 * state.completed_operations as f64;
        self.publish(&mut state, candidate, phase);
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, AdapterState> {
        // A poisoned lock only means an earlier report panicked; the counters are still sound.
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn publish(&self, state: &mut AdapterState, candidate: f64, phase: &str) {
        if candidate > state.high_water_ratio {
            state.high_water_ratio = candidate;
        }
        let message = if phase.is_empty() {
            self.operation_label.to_string()
        } else {
            format!("{}: {phase}", self.operation_label)
        };
        // A progress sink that fails is not a reason to lose the geometry, but this can run inside
        // the kernel: a callback that panics is a bug in the callback and is left to surface rather
        // than caught.
        (self.reporter)(state.high_water_ratio, &message);
    }
}

/// The kernel-side sink that feeds an adapter, or `None` when nobody is watching.
pub(crate) fn progress_sink_for(
    adapter: Option<&Arc<BooleanProgressAdapter>>,
) -> Option<ProgressReporter> {
    adapter.map(|adapter| {
        let adapter = Arc::clone(adapter);
        ProgressReporter::new(move |phase: Phase, fraction: Option<f64>| {
            adapter.report(phase.name(), fraction);
        })
    })
}
