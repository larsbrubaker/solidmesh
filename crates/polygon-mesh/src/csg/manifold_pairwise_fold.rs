//! The n-ary combine spelled out as a left fold of binary booleans, for a winding rule the kernel's
//! batch tree cannot take, with a variant that yields between steps; called by `manifold_kernel`.

use std::future::Future;
use std::sync::Arc;

use agg_path::progress_reporter::ProgressReporter;

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanEngine, OpType, WindingRule};

use super::boolean_progress_adapter::{progress_sink_for, BooleanProgressAdapter};
use super::manifold_cancellable_boolean;
use super::manifold_kernel::{throw_if_cancelled, yield_to_ui, COMBINE_COMPLETE_PHASE};
use super::{BooleanError, BooleanReporter};
use crate::CancelToken;

/// Folds `manifolds` (at least two) left to right with `operation_type` under `winding_rule`.
pub(crate) fn combine_pairwise(
    manifolds: &[Manifold],
    operation_type: OpType,
    winding_rule: WindingRule,
    cancel: &CancelToken,
    reporter: Option<&BooleanReporter>,
    amount_per_operation: f64,
    ratio_completed: f64,
) -> Result<Manifold, BooleanError> {
    let progress = adapter_for(manifolds, reporter, amount_per_operation, ratio_completed);
    let sink = progress_sink_for(progress.as_ref());

    // Only ever holds an intermediate this function created; manifolds[0] is the caller's.
    let mut accumulated: Option<Manifold> = None;
    for operand in &manifolds[1..] {
        let left = accumulated.as_ref().unwrap_or(&manifolds[0]);
        accumulated = Some(manifold_cancellable_boolean::boolean(
            left,
            operand,
            operation_type,
            BooleanEngine::Auto,
            winding_rule,
            sink.as_ref(),
            cancel,
        )?);
        if let Some(progress) = &progress {
            progress.complete_operation(COMBINE_COMPLETE_PHASE);
        }
    }

    Ok(accumulated.unwrap_or_else(|| manifolds[0].clone()))
}

/// [`combine_pairwise`] with the UI given its thread back between two operands. Between, and
/// only between: one pairwise boolean is one call that cannot hand anything back part way
/// through. The yield comes after the step's own completion report so the bar has somewhere new
/// to move to before it paints. `sink_reporter` is the kernel-side form of `yield_reporter`
/// (`None` when nobody watches it).
#[allow(clippy::too_many_arguments)]
pub(crate) async fn combine_pairwise_async<F: Future<Output = ()>, Y: Fn() -> F + ?Sized>(
    manifolds: &[Manifold],
    operation_type: OpType,
    winding_rule: WindingRule,
    cancel: &CancelToken,
    sink_reporter: Option<&BooleanReporter>,
    amount_per_operation: f64,
    ratio_completed: f64,
    yield_reporter: &ProgressReporter,
    ui_yield: Option<&Y>,
) -> Result<Manifold, BooleanError> {
    let progress = adapter_for(
        manifolds,
        sink_reporter,
        amount_per_operation,
        ratio_completed,
    );
    let sink = progress_sink_for(progress.as_ref());

    let mut accumulated: Option<Manifold> = None;
    for operand in &manifolds[1..] {
        let left = accumulated.as_ref().unwrap_or(&manifolds[0]);
        accumulated = Some(manifold_cancellable_boolean::boolean(
            left,
            operand,
            operation_type,
            BooleanEngine::Auto,
            winding_rule,
            sink.as_ref(),
            cancel,
        )?);
        if let Some(progress) = &progress {
            progress.complete_operation(COMBINE_COMPLETE_PHASE);
        }

        // The token is read with the yield: the yield is what lets the user press Stop at all,
        // so reading it here is what makes the press land on this pair rather than after the
        // whole fold.
        throw_if_cancelled(cancel)?;
        yield_to_ui(yield_reporter, ui_yield).await;
    }

    Ok(accumulated.unwrap_or_else(|| manifolds[0].clone()))
}

/// The fold's bar, one step per pair, or `None` when nobody is watching.
fn adapter_for(
    manifolds: &[Manifold],
    reporter: Option<&BooleanReporter>,
    amount_per_operation: f64,
    ratio_completed: f64,
) -> Option<Arc<BooleanProgressAdapter>> {
    reporter.map(|reporter| {
        Arc::new(BooleanProgressAdapter::new(
            Arc::clone(reporter),
            ratio_completed,
            amount_per_operation,
            manifolds.len() - 1,
        ))
    })
}
