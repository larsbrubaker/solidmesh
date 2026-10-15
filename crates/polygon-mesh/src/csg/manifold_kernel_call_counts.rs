//! How many boolean evaluations the manifold kernel has run, by kind, for the work a caller is
//! measuring. Port of agg-sharp `Csg/ManifoldKernelCallCounts.cs`; counted by `manifold_kernel`
//! (the n-ary combine), `manifold_cancellable_boolean` (each pairwise step) and
//! `manifold_operand_batch` (the colour group union at import).
//!
//! It lets a test pin that an n-ary boolean reached the kernel as ONE call rather than a pairwise
//! fold. Never read by production code.
//!
//! A count is a [`KernelCallCounts`] tally, not a process-wide number: a test that measures one
//! operation must see that operation's calls only, while other tests (or other documents) run
//! booleans at the same time. The tally counts the kernel calls made on a thread while it is
//! entered there ([`KernelCallCounts::enter`]); tallies nest, and a call counts into every tally
//! entered around it. Work handed to another thread carries the tallies with it: the submitter
//! takes [`KernelCallContext::current`] and the worker runs the work inside
//! [`KernelCallContext::enter`] (the model's compute pool does this for every job), so a boolean
//! run on a worker counts into the tally of the operation that asked for it.

use std::cell::RefCell;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

#[derive(Debug, Default)]
struct Counts {
    operand_combines: AtomicUsize,
    pairwise_steps: AtomicUsize,
    color_group_unions: AtomicUsize,
}

/// One tally of kernel calls. Clones share the same counts.
#[derive(Debug, Clone, Default)]
pub struct KernelCallCounts(Arc<Counts>);

impl KernelCallCounts {
    /// A tally at zero, counting nothing until it is entered.
    pub fn new() -> Self {
        Self::default()
    }

    /// Counts the kernel calls made on this thread (and in work this thread hands the compute
    /// pool) until the returned guard drops.
    pub fn enter(&self) -> KernelCallScope {
        KernelCallScope::push(std::slice::from_ref(&self.0))
    }

    /// One n-ary CSG-tree evaluation over a boolean's operands (two or more of them).
    pub fn operand_combines(&self) -> usize {
        self.0.operand_combines.load(Ordering::SeqCst)
    }

    /// One binary step of a winding-rule pairwise fold - n-1 of them per folded boolean.
    pub fn pairwise_steps(&self) -> usize {
        self.0.pairwise_steps.load(Ordering::SeqCst)
    }

    /// One union of a multi-coloured operand's colour groups at import, before the real
    /// operation runs.
    pub fn color_group_unions(&self) -> usize {
        self.0.color_group_unions.load(Ordering::SeqCst)
    }
}

thread_local! {
    // The tallies entered on this thread, outermost first.
    static ENTERED: RefCell<Vec<Arc<Counts>>> = const { RefCell::new(Vec::new()) };
}

/// The tallies entered on one thread, captured so work run on another thread counts into them.
#[derive(Debug, Clone, Default)]
pub struct KernelCallContext(Vec<Arc<Counts>>);

impl KernelCallContext {
    /// The tallies entered on this thread right now (empty when none is).
    pub fn current() -> Self {
        Self(ENTERED.with(|entered| entered.borrow().clone()))
    }

    /// Counts this thread's kernel calls into the captured tallies until the guard drops.
    pub fn enter(&self) -> KernelCallScope {
        KernelCallScope::push(&self.0)
    }
}

/// While alive, this thread's kernel calls count into the tallies it entered. Dropping it leaves
/// them; it belongs to the thread that made it. Scopes on one thread must drop innermost first
/// (hold each in a local for the block it measures): dropping an outer scope leaves every tally
/// entered since it as well.
#[must_use = "the tally stops counting as soon as the scope is dropped"]
#[derive(Debug)]
pub struct KernelCallScope {
    // How many tallies were entered before this scope, restored on drop.
    outer: usize,
    // A scope moved to another thread would restore that thread's list wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl KernelCallScope {
    fn push(tallies: &[Arc<Counts>]) -> Self {
        let outer = ENTERED.with(|entered| {
            let mut entered = entered.borrow_mut();
            let outer = entered.len();
            entered.extend(tallies.iter().cloned());
            outer
        });
        Self {
            outer,
            _not_send: std::marker::PhantomData,
        }
    }
}

impl Drop for KernelCallScope {
    fn drop(&mut self) {
        ENTERED.with(|entered| entered.borrow_mut().truncate(self.outer));
    }
}

fn count(counter: impl Fn(&Counts) -> &AtomicUsize) {
    ENTERED.with(|entered| {
        // A tally entered twice on this thread (an outer scope's context re-entered inside it)
        // still counts each call once.
        let entered = entered.borrow();
        for (index, tally) in entered.iter().enumerate() {
            if !entered[..index].iter().any(|seen| Arc::ptr_eq(seen, tally)) {
                counter(tally).fetch_add(1, Ordering::SeqCst);
            }
        }
    });
}

pub(crate) fn count_operand_combine() {
    count(|counts| &counts.operand_combines);
}

pub(crate) fn count_pairwise_step() {
    count(|counts| &counts.pairwise_steps);
}

pub(crate) fn count_color_group_union() {
    count(|counts| &counts.color_group_unions);
}
