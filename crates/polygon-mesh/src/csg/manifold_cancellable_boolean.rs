//! The two cancellable kernel entry points a boolean combines through: one binary boolean with
//! an explicit engine and winding rule, and the n-ary batch as a CSG tree. Port of agg-sharp
//! `Csg/ManifoldCancellableBoolean.cs`; called by `manifold_kernel` and
//! `manifold_pairwise_fold`.
//!
//! Cancellation comes back from the kernel as an empty manifold with status `Cancelled`; here
//! it becomes [`BooleanError::Cancelled`] (C# throws `OperationCanceledException`).

use manifold_rust::csg_tree::CsgNode;
use manifold_rust::manifold::Manifold;
use manifold_rust::progress::ProgressReporter;
use manifold_rust::types::{BooleanEngine, Error as KernelStatus, OpType, WindingRule};

use super::{manifold_kernel, manifold_kernel_call_counts, BooleanError};
use crate::CancelToken;

/// One binary boolean.
pub(crate) fn boolean(
    a: &Manifold,
    b: &Manifold,
    operation_type: OpType,
    engine: BooleanEngine,
    winding_rule: WindingRule,
    progress: Option<&ProgressReporter>,
    cancel: &CancelToken,
) -> Result<Manifold, BooleanError> {
    manifold_kernel_call_counts::count_pairwise_step();
    manifold_kernel::ensure_configured();
    let result = a.boolean_with_engine_rule_and_progress(
        b,
        operation_type,
        engine,
        winding_rule,
        Some(cancel.kernel_token()),
        progress,
    );
    if result.status() == KernelStatus::Cancelled {
        return Err(BooleanError::Cancelled);
    }
    Ok(result)
}

/// The n-ary boolean as one CSG tree (what makes a large n-ary union tractable). It reads the
/// process-global engine and the default winding rule.
///
/// `progress` goes to every binary boolean the tree runs (`CsgNode::
/// evaluate_with_token_and_progress`), so each one streams its own kernel phases from the
/// start; observation only, the result is the same either way.
pub(crate) fn batch_boolean(
    manifolds: &[Manifold],
    operation_type: OpType,
    progress: Option<&ProgressReporter>,
    cancel: &CancelToken,
) -> Result<Manifold, BooleanError> {
    manifold_kernel::ensure_configured();
    let result = batch_boolean_core(manifolds, operation_type, progress, cancel);
    if result.status() == KernelStatus::Cancelled {
        return Err(BooleanError::Cancelled);
    }
    Ok(result)
}

fn batch_boolean_core(
    manifolds: &[Manifold],
    operation_type: OpType,
    progress: Option<&ProgressReporter>,
    cancel: &CancelToken,
) -> Manifold {
    // A single operand has nothing to combine with and every operation is identity on it - but
    // an already-cancelled token still wins, so a one-operand call never reports success after
    // a cancel.
    if manifolds.len() == 1 {
        return if cancel.is_cancelled() {
            Manifold::make_empty(KernelStatus::Cancelled)
        } else {
            manifolds[0].clone()
        };
    }

    // Cloned, not aliased: the tree takes ownership of what it evaluates, and the operands stay
    // the caller's to use again afterwards.
    let leaves = manifolds
        .iter()
        .map(|m| CsgNode::leaf(m.as_impl().clone()))
        .collect();

    Manifold::from_impl(
        CsgNode::op_n(operation_type, leaves)
            .evaluate_with_token_and_progress(Some(cancel.kernel_token()), progress),
    )
}
