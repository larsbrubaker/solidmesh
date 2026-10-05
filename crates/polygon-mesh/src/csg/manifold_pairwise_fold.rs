//! The n-ary combine spelled out as a left fold of binary booleans, for a winding rule the
//! kernel's batch tree cannot take. Port of agg-sharp `Csg/ManifoldPairwiseFold.cs`
//! (`CombinePairwise`; the async variant is not ported yet); called by `manifold_kernel`.

use std::sync::Arc;

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanEngine, OpType, WindingRule};

use super::boolean_progress_adapter::{progress_sink_for, BooleanProgressAdapter};
use super::manifold_cancellable_boolean;
use super::manifold_kernel::COMBINE_COMPLETE_PHASE;
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
    let progress = reporter.map(|reporter| {
        Arc::new(BooleanProgressAdapter::new(
            Arc::clone(reporter),
            ratio_completed,
            amount_per_operation,
            manifolds.len() - 1,
        ))
    });
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
