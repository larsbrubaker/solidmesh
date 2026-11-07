//! `manifold_kernel_call_counts`: a tally counts the kernel calls of the work it measures and
//! nothing else, so a test pinning one boolean's kernel work holds while other booleans run on
//! other threads at the same time.

use std::sync::{Arc, Barrier};

use polygon_mesh::csg::csg_operations;
use polygon_mesh::csg::manifold_kernel_call_counts::{KernelCallContext, KernelCallCounts};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

fn overlapping_cubes() -> (Mesh, Mesh) {
    let a = create_cube(10.0, 10.0, 10.0);
    let mut b = create_cube(10.0, 10.0, 10.0);
    b.translate(Vector3::new(5.0, 0.0, 0.0));
    (a, b)
}

fn union_once() {
    let (a, b) = overlapping_cubes();
    let result = csg_operations::union(&a, &b, &CancelToken::new()).expect("union runs");
    assert!(!result.faces.is_empty());
}

#[test]
fn one_union_is_one_operand_combine() {
    let counts = KernelCallCounts::new();
    {
        let _scope = counts.enter();
        union_once();
    }
    assert_eq!(counts.operand_combines(), 1);
    assert_eq!(counts.pairwise_steps(), 0);

    // Once the scope has ended, later booleans no longer count.
    union_once();
    assert_eq!(counts.operand_combines(), 1);
}

#[test]
fn concurrent_operations_count_only_their_own_kernel_calls() {
    // Each thread runs a different number of unions, all in step with the others, so a shared
    // counter would show every thread the sum.
    let runs = [1usize, 2, 3, 4];
    let start = Arc::new(Barrier::new(runs.len()));
    let threads: Vec<_> = runs
        .iter()
        .map(|&unions| {
            let start = Arc::clone(&start);
            std::thread::spawn(move || {
                let counts = KernelCallCounts::new();
                let _scope = counts.enter();
                start.wait();
                for _ in 0..unions {
                    union_once();
                }
                counts.operand_combines()
            })
        })
        .collect();
    let seen: Vec<usize> = threads
        .into_iter()
        .map(|thread| thread.join().expect("the thread runs"))
        .collect();
    assert_eq!(seen, runs);
}

#[test]
fn work_handed_to_another_thread_counts_into_the_submitters_tally() {
    let counts = KernelCallCounts::new();
    let _scope = counts.enter();
    let context = KernelCallContext::current();
    std::thread::spawn(move || {
        let _worker = context.enter();
        union_once();
    })
    .join()
    .expect("the worker runs");
    // A thread that entered nothing counts into no tally.
    std::thread::spawn(union_once)
        .join()
        .expect("the bystander runs");
    assert_eq!(counts.operand_combines(), 1);
}

#[test]
fn nested_tallies_both_count_the_inner_work_once() {
    let outer = KernelCallCounts::new();
    let inner = KernelCallCounts::new();
    let _outer = outer.enter();
    union_once();
    {
        let _inner = inner.enter();
        // Re-entering a tally already counting on this thread does not count it twice.
        let _again = KernelCallContext::current().enter();
        union_once();
    }
    union_once();
    assert_eq!(inner.operand_combines(), 1);
    assert_eq!(outer.operand_combines(), 3);
}
