//! The loop wrapper every BVH builder and tracer routes through (agg-sharp
//! `VectorMath/Parallel.cs`, C# `MatterHackers.Agg.Parallel`).
//!
//! The C# wrapper hands loops to `System.Threading.Tasks.Parallel` unless the process-wide
//! `Sequential` flag (single-threaded wasm hosts) or the per-thread inline mark (the UI thread)
//! keeps them on the calling thread. Here every loop runs in order on the calling thread: the
//! library crates must build for `wasm32-unknown-unknown`, which has no threads, and the C#
//! contract makes no promise the parallel path would keep — callers only rely on every index
//! running once, which the sequential path gives. That is the C# behavior with `Sequential` set
//! or on a marked thread, on every host (divergence 30). The flags are kept so call sites port
//! unchanged; they record what was asked for and do not change how a loop runs.

use std::cell::Cell;
use std::sync::atomic::{AtomicBool, Ordering};

static SEQUENTIAL: AtomicBool = AtomicBool::new(false);

thread_local! {
    static RUN_INLINE_ON_THIS_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// The C# `Sequential` flag as last set (loops run sequentially either way).
pub fn sequential() -> bool {
    SEQUENTIAL.load(Ordering::Relaxed)
}

/// Sets the C# `Sequential` flag.
pub fn set_sequential(value: bool) {
    SEQUENTIAL.store(value, Ordering::Relaxed);
}

/// True once [`run_inline_on_current_thread`] has marked the calling thread (C#
/// `CurrentThreadRunsInline`); per thread, and stays set for the thread's lifetime.
pub fn current_thread_runs_inline() -> bool {
    RUN_INLINE_ON_THIS_THREAD.with(Cell::get)
}

/// Marks the calling thread so every loop routed through here from it runs on it alone (C#
/// `RunInlineOnCurrentThread`; agg-gui's UI thread is the caller in MatterCAD).
pub fn run_inline_on_current_thread() {
    RUN_INLINE_ON_THIS_THREAD.with(|inline| inline.set(true));
}

/// Runs `action` on every item of `source`, in order (C# `ForEach`).
pub fn for_each<T>(source: impl IntoIterator<Item = T>, action: impl FnMut(T)) {
    source.into_iter().for_each(action);
}

/// Runs `action` for every index in `start_inclusive..end_exclusive`, in order (C# `For`).
pub fn for_range(start_inclusive: i32, end_exclusive: i32, mut action: impl FnMut(i32)) {
    for i in start_inclusive..end_exclusive {
        action(i);
    }
}

/// Runs the actions in order (C# `Invoke`). The first panicking action propagates and later
/// actions never run, as the C# `Sequential` path does with an exception.
pub fn invoke(actions: &mut [&mut dyn FnMut()]) {
    for action in actions.iter_mut() {
        action();
    }
}

/// The C# `For<T>(start, end, localInit, body, localFinally)` overload: one local value is
/// created, threaded through every index in order, and handed to `local_finally`. The C# body
/// also receives a `ParallelLoopState`; nothing in MatterCAD reads it, and a sequential loop
/// has no other replicas to stop, so the Rust body does not take one.
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
