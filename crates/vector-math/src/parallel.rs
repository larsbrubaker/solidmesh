//! The loop wrapper every BVH builder and tracer routes through.
//!
//! Every loop runs in order on the calling thread: the library crates must build for
//! `wasm32-unknown-unknown`, which has no threads, and callers only rely on every index running
//! once, which the sequential path gives, with the same results on every host. A threaded path
//! can be added behind a feature once a measured need shows up. The `sequential` flag and the
//! per-thread inline mark record what a caller asked for (single-threaded hosts, the UI
//! thread) and do not change how a loop runs.

use std::cell::Cell;
use std::sync::atomic::{AtomicBool, Ordering};

static SEQUENTIAL: AtomicBool = AtomicBool::new(false);

thread_local! {
    static RUN_INLINE_ON_THIS_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// The `sequential` flag as last set (loops run sequentially either way).
pub fn sequential() -> bool {
    SEQUENTIAL.load(Ordering::Relaxed)
}

/// Sets the `sequential` flag.
pub fn set_sequential(value: bool) {
    SEQUENTIAL.store(value, Ordering::Relaxed);
}

/// True once [`run_inline_on_current_thread`] has marked the calling thread; per thread, and
/// stays set for the thread's lifetime.
pub fn current_thread_runs_inline() -> bool {
    RUN_INLINE_ON_THIS_THREAD.with(Cell::get)
}

/// Marks the calling thread so every loop routed through here from it runs on it alone
/// (agg-gui's UI thread is the caller in MatterCAD).
pub fn run_inline_on_current_thread() {
    RUN_INLINE_ON_THIS_THREAD.with(|inline| inline.set(true));
}

/// Runs `action` on every item of `source`, in order.
pub fn for_each<T>(source: impl IntoIterator<Item = T>, action: impl FnMut(T)) {
    source.into_iter().for_each(action);
}

/// Runs `action` for every index in `start_inclusive..end_exclusive`, in order.
pub fn for_range(start_inclusive: i32, end_exclusive: i32, mut action: impl FnMut(i32)) {
    for i in start_inclusive..end_exclusive {
        action(i);
    }
}

/// Runs the actions in order. The first panicking action propagates and later
/// actions never run, so a failure stops the work at the first broken step.
pub fn invoke(actions: &mut [&mut dyn FnMut()]) {
    for action in actions.iter_mut() {
        action();
    }
}

/// A loop with a per-worker local value: one local value is created, threaded through every
/// index in order, and handed to `local_finally`. The body takes no loop-state handle: a
/// sequential loop has no other workers to stop.
pub fn for_with_local<T>(
    start_inclusive: i32,
    end_exclusive: i32,
    local_init: impl FnOnce() -> T,
    mut body: impl FnMut(i32, T) -> T,
    local_finally: impl FnOnce(T),
) {
    let mut local = local_init();
    for i in start_inclusive..end_exclusive {
        local = body(i, local);
    }

    local_finally(local);
}
