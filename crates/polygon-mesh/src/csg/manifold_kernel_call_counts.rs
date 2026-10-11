//! How many boolean evaluations the manifold kernel has run, by kind. Port of agg-sharp
//! `Csg/ManifoldKernelCallCounts.cs`; counted by `manifold_kernel` (the n-ary combine),
//! `manifold_cancellable_boolean` (each pairwise step) and `manifold_operand_batch` (the colour
//! group union at import).
//!
//! It lets a test pin that an n-ary boolean reached the kernel as ONE call rather than a pairwise
//! fold. Process-global and never read by production code; a test resets it before the work it
//! measures and must not run beside other boolean work (C# `NotInParallel`). C# keeps it
//! `internal` behind `InternalsVisibleTo`; here it is public for the same tests.

use std::sync::atomic::{AtomicUsize, Ordering};

static OPERAND_COMBINES: AtomicUsize = AtomicUsize::new(0);
static PAIRWISE_STEPS: AtomicUsize = AtomicUsize::new(0);
static COLOR_GROUP_UNIONS: AtomicUsize = AtomicUsize::new(0);

/// One n-ary CSG-tree evaluation over a boolean's operands (two or more of them).
pub fn operand_combines() -> usize {
    OPERAND_COMBINES.load(Ordering::SeqCst)
}

/// One binary step of a winding-rule pairwise fold - n-1 of them per folded boolean.
pub fn pairwise_steps() -> usize {
    PAIRWISE_STEPS.load(Ordering::SeqCst)
}

/// One union of a multi-coloured operand's colour groups at import, before the real operation
/// runs.
pub fn color_group_unions() -> usize {
    COLOR_GROUP_UNIONS.load(Ordering::SeqCst)
}

/// Sets every count back to zero.
pub fn reset() {
    OPERAND_COMBINES.store(0, Ordering::SeqCst);
    PAIRWISE_STEPS.store(0, Ordering::SeqCst);
    COLOR_GROUP_UNIONS.store(0, Ordering::SeqCst);
}

pub(crate) fn count_operand_combine() {
    OPERAND_COMBINES.fetch_add(1, Ordering::SeqCst);
}

pub(crate) fn count_pairwise_step() {
    PAIRWISE_STEPS.fetch_add(1, Ordering::SeqCst);
}

pub(crate) fn count_color_group_union() {
    COLOR_GROUP_UNIONS.fetch_add(1, Ordering::SeqCst);
}
