//! The boolean's driver: the kernel's one-time configuration, the operand loop, and which
//! combine runs. Port of agg-sharp `Csg/ManifoldKernel.cs` (`RunBoolean`, `CombineAndRead`;
//! the async variants are not ported yet). Each stage it drives is its own module:
//! `manifold_operand_batch`, `manifold_import`, `manifold_cancellable_boolean`,
//! `manifold_pairwise_fold` and `manifold_result_reader`.

use std::sync::{Arc, Once};

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanConfig, BooleanEngine, OpType, WindingRule};
use vector_math::Matrix4X4;

use super::boolean_progress_adapter::{progress_sink_for, BooleanProgressAdapter};
use super::manifold_operand_batch::{AddOutcome, ManifoldOperandBatch};
use super::{manifold_cancellable_boolean, manifold_pairwise_fold, manifold_result_reader};
use super::{BooleanError, BooleanOptions, BooleanReporter, CsgModes};
use crate::{CancelToken, Mesh};

/// Message phase for the edges of a combine - either end of the batch, or between two
/// pairwise booleans - where the kernel itself has nothing to report.
pub(crate) const COMBINE_COMPLETE_PHASE: &str = "combining";

static CONFIGURE: Once = Once::new();

/// Selects the kernel's boolean engine once, before the first boolean runs.
///
/// `Auto` keeps the fast exact pipeline for strictly manifold operands and only pays for the
/// slower rational-arithmetic engine when an operand came in through the robust import as
/// non-manifold soup. The setting is process-global, so it is made once rather than per call.
/// (agg-sharp also switches the kernel's parallelism here; in Rust that is the `parallel`
/// Cargo feature, on for native targets and off for wasm32 in polygon-mesh's Cargo.toml.)
pub(crate) fn ensure_configured() {
    CONFIGURE.call_once(|| BooleanConfig::set_default_engine(BooleanEngine::Auto));
}

/// One boolean over `items` (each mesh with its transform). Every failure is an error the
/// caller sees, including cancellation.
///
/// A union whose operands are not all usable degrades rather than failing outright: the ones
/// that imported are combined and the answer comes back as [`BooleanError::Partial`].
pub(crate) fn run_boolean(
    items: &[(&Mesh, Matrix4X4)],
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    ensure_configured();
    let mut batch = ManifoldOperandBatch::new(
        operation,
        options.mesh_colors.clone(),
        options.repair_orientation,
    );

    for &(mesh, matrix) in items {
        if batch.try_add(mesh, matrix, cancel)? == AddOutcome::ResultIsEmpty {
            return Ok(Mesh::new());
        }
    }

    batch.throw_if_nothing_imported()?;

    let result = combine_and_read(&batch, operation, cancel, options)?;

    batch.throw_if_any_skipped(result)
}

/// Runs the n-ary boolean over already-imported operands and reads the result back.
fn combine_and_read(
    batch: &ManifoldOperandBatch,
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    let reporter = options.reporter.as_ref();
    let amount_per_operation = options.amount_per_operation;
    let ratio_completed = options.ratio_completed;
    let winding_rule = options.winding_rule;
    if batch.manifolds.is_empty() {
        return Ok(Mesh::new());
    }

    let operation_type = operation_type_of(operation);

    let combined: Manifold;
    let bool_result = if batch.manifolds.len() == 1 {
        // A single operand is its own answer; running a one-operand boolean would only cost a
        // copy.
        &batch.manifolds[0]
    } else if needs_explicit_boolean(winding_rule) {
        combined = manifold_pairwise_fold::combine_pairwise(
            &batch.manifolds,
            operation_type,
            winding_rule,
            cancel,
            reporter,
            amount_per_operation,
            ratio_completed,
        )?;
        &combined
    } else {
        let progress = batch_progress_for(reporter, amount_per_operation, ratio_completed);
        combined = manifold_cancellable_boolean::batch_boolean(
            &batch.manifolds,
            operation_type,
            progress_sink_for(progress.as_ref()).as_ref(),
            cancel,
        )?;
        if let Some(progress) = &progress {
            progress.complete_operation(COMBINE_COMPLETE_PHASE);
        }
        &combined
    };

    manifold_result_reader::read_result(bool_result, batch)
}

/// The kernel's name for one of our operations.
fn operation_type_of(operation: CsgModes) -> OpType {
    match operation {
        CsgModes::Union => OpType::Add,
        CsgModes::Subtract => OpType::Subtract,
        CsgModes::Intersect => OpType::Intersect,
    }
}

/// Whether the n-ary combine has to be a pairwise fold rather than the kernel's batch tree.
///
/// The batch tree is what makes a large n-ary union tractable, but it takes no winding rule,
/// so only a non-default rule drops to the fold. Progress is NOT a reason to fold: the fold
/// re-runs a full binary boolean against the growing result for every operand, which turned a
/// 20-operand union from seconds into minutes.
fn needs_explicit_boolean(winding_rule: WindingRule) -> bool {
    winding_rule != WindingRule::Positive
}

/// The bar for a batch boolean, already told the combine has started, or `None` when nobody is
/// watching. The whole batch is one operation window.
fn batch_progress_for(
    reporter: Option<&BooleanReporter>,
    amount_per_operation: f64,
    ratio_completed: f64,
) -> Option<Arc<BooleanProgressAdapter>> {
    let reporter = reporter?;
    let progress = Arc::new(BooleanProgressAdapter::new(
        Arc::clone(reporter),
        ratio_completed,
        amount_per_operation,
        1,
    ));
    progress.report(COMBINE_COMPLETE_PHASE, None);
    Some(progress)
}
