// Port of `ParallelInlineOnMarkedThreadTests` (agg-sharp
// Tests/Agg.Tests/Agg.VectorMath/ParallelInlineOnMarkedThreadTests.cs). A thread marked with
// `run_inline_on_current_thread` (the UI thread) runs every loop routed through the wrapper on
// itself. The C# bodies spin so the unmarked parallel path would spread across pool threads;
// the Rust wrapper never leaves the calling thread, so the spin is dropped. The loop-state check
// of `ForWithLoopStateRunsEveryIterationOnTheMarkedThread` has no Rust counterpart: the body is
// not handed a `ParallelLoopState` (see `parallel::for_with_local`).

use std::sync::Mutex;
use std::thread::{self, ThreadId};
use vector_math::parallel;

const ITERATIONS: i32 = 1000;

fn run_on_marked_thread(
    work: impl FnOnce(&Mutex<Vec<ThreadId>>) + Send,
) -> (ThreadId, Vec<ThreadId>) {
    let ran_on = Mutex::new(Vec::new());
    let caller_id = thread::scope(|scope| {
        scope
            .spawn(|| {
                parallel::run_inline_on_current_thread();
                assert!(parallel::current_thread_runs_inline());
                work(&ran_on);
                thread::current().id()
            })
            .join()
            .expect("the marked thread's work panicked")
    });
    (caller_id, ran_on.into_inner().unwrap())
}

fn record(ids: &Mutex<Vec<ThreadId>>) {
    ids.lock().unwrap().push(thread::current().id());
}

fn assert_all_on(caller_id: ThreadId, ran_on: &[ThreadId], expected_count: usize) {
    assert_eq!(ran_on.len(), expected_count);
    let elsewhere = ran_on.iter().filter(|id| **id != caller_id).count();
    assert_eq!(
        elsewhere, 0,
        "a marked thread must not hand loop work to the thread pool, where a busy pool can leave \
         it waiting for minutes"
    );
}

#[test]
fn for_runs_every_iteration_on_the_marked_thread() {
    let (caller_id, ran_on) =
        run_on_marked_thread(|ids| parallel::for_range(0, ITERATIONS, |_| record(ids)));
    assert_all_on(caller_id, &ran_on, ITERATIONS as usize);
}

#[test]
fn for_each_runs_every_item_on_the_marked_thread() {
    let (caller_id, ran_on) =
        run_on_marked_thread(|ids| parallel::for_each(0..ITERATIONS, |_| record(ids)));
    assert_all_on(caller_id, &ran_on, ITERATIONS as usize);
}

#[test]
fn invoke_runs_every_action_on_the_marked_thread() {
    let (caller_id, ran_on) = run_on_marked_thread(|ids| {
        parallel::invoke(&mut [
            &mut || record(ids),
            &mut || record(ids),
            &mut || record(ids),
            &mut || record(ids),
            &mut || record(ids),
            &mut || record(ids),
            &mut || record(ids),
            &mut || record(ids),
        ])
    });
    assert_all_on(caller_id, &ran_on, 8);
}

/// The localInit/body/localFinally overload must fold every local into the result.
#[test]
fn for_with_loop_state_runs_every_iteration_on_the_marked_thread() {
    let sum = Mutex::new(0i64);
    let (caller_id, ran_on) = run_on_marked_thread(|ids| {
        parallel::for_with_local(
            0,
            ITERATIONS,
            || {
                record(ids);
                0i64
            },
            |i, local| {
                record(ids);
                local + i64::from(i)
            },
            |local| {
                record(ids);
                *sum.lock().unwrap() += local;
            },
        )
    });

    // One localInit and one localFinally on the single replica, plus every body.
    assert_all_on(caller_id, &ran_on, ITERATIONS as usize + 2);
    assert_eq!(
        sum.into_inner().unwrap(),
        i64::from(ITERATIONS) * i64::from(ITERATIONS - 1) / 2
    );
}
