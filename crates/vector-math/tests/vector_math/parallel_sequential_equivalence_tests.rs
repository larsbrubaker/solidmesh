// The parallel wrapper gives the same results with the process-wide sequential flag on and off.
// These are the checks of the wrapper itself; the cases that build ray-tracer BVHs live in
// polygon-mesh's tests/ray_tracer/parallel_sequential_equivalence_tests.rs.
//
// The flag is process-wide and tests run on several threads, so the tests that set it hold
// `FLAG_LOCK` and restore the incoming value.

use std::sync::Mutex;
use vector_math::parallel;

static FLAG_LOCK: Mutex<()> = Mutex::new(());

/// Restores the incoming `Sequential` value however the test ends.
struct RestoreSequential(bool);

impl Drop for RestoreSequential {
    fn drop(&mut self) {
        parallel::set_sequential(self.0);
    }
}

#[test]
fn invoke_runs_actions_in_order_when_sequential() {
    let _guard = FLAG_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let _restore = RestoreSequential(parallel::sequential());

    let order = std::cell::RefCell::new(Vec::new());

    parallel::set_sequential(true);
    parallel::invoke(&mut [
        &mut || order.borrow_mut().push(0),
        &mut || order.borrow_mut().push(1),
        &mut || order.borrow_mut().push(2),
        &mut || order.borrow_mut().push(3),
    ]);

    // Joined rather than compared as collections: the order is the one thing this test pins.
    let joined: Vec<String> = order.borrow().iter().map(|ran| ran.to_string()).collect();
    assert_eq!(joined.join(","), "0,1,2,3");

    // The parallel path makes no ordering promise, only that every action ran - so this leg is
    // deliberately order-insensitive, and says so by sorting before the same strict compare.
    let parallel_order = std::sync::Mutex::new(Vec::new());
    parallel::set_sequential(false);
    parallel::invoke(&mut [
        &mut || parallel_order.lock().unwrap().push(0),
        &mut || parallel_order.lock().unwrap().push(1),
        &mut || parallel_order.lock().unwrap().push(2),
        &mut || parallel_order.lock().unwrap().push(3),
    ]);

    let mut parallel_order = parallel_order.into_inner().unwrap();
    parallel_order.sort();
    let joined: Vec<String> = parallel_order.iter().map(|ran| ran.to_string()).collect();
    assert_eq!(joined.join(","), "0,1,2,3");
}

#[test]
fn for_and_for_each_cover_every_index_when_sequential() {
    let _guard = FLAG_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let _restore = RestoreSequential(parallel::sequential());

    parallel::set_sequential(true);

    let mut visited = Vec::new();
    parallel::for_range(3, 9, |i| visited.push(i));
    assert_eq!(visited, vec![3, 4, 5, 6, 7, 8]);

    let mut seen = Vec::new();
    parallel::for_each(["a", "b", "c"], |s| seen.push(s));
    assert_eq!(seen, vec!["a", "b", "c"]);
}
