//! The boolean's driver: the kernel's one-time configuration, the operand loop, and which combine
//! runs, each with a yielding variant for a caller that hands the UI its thread back. Each stage it
//! drives is its own module: `manifold_operand_batch`, `manifold_import`,
//! `manifold_cancellable_boolean`, `manifold_pairwise_fold` and `manifold_result_reader`.

use std::future::Future;
use std::sync::{Arc, Once};

use vector_math::progress::ProgressReporter;

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanConfig, BooleanEngine, OpType, WindingRule};
use vector_math::Matrix4X4;

use super::boolean_progress_adapter::{progress_sink_for, BooleanProgressAdapter};
use super::manifold_operand_batch::{AddOutcome, ManifoldOperandBatch};
use super::{
    manifold_cancellable_boolean, manifold_kernel_call_counts, manifold_pairwise_fold,
    manifold_result_reader,
};
use super::{BooleanError, BooleanOptions, BooleanReporter, CsgModes};
use crate::{CancelToken, Mesh};

/// Message phase for the edges of a combine - either end of the batch, or between two
/// pairwise booleans - where the kernel itself has nothing to report.
pub(crate) const COMBINE_COMPLETE_PHASE: &str = "combining";

static CONFIGURE: Once = Once::new();

/// Selects the kernel's boolean engine once, before the first boolean runs.
///
/// `Auto` keeps the fast exact pipeline for strictly manifold operands and only pays for the slower
/// rational-arithmetic engine when an operand came in through the robust import as non-manifold
/// soup. The setting is process-global, so it is made once rather than per call. (The kernel's
/// parallelism is the `parallel` Cargo feature instead, on for native targets and off for wasm32 in
/// polygon-mesh's Cargo.toml.)
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
        options.single_precision,
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

/// [`run_boolean`] for a caller that can hand the UI its thread back: the
/// same boolean, with a yield after every operand's import and around the combine. `reporter`
/// is the progress sink (`options.reporter` is not read) and the yields go through it, so a
/// reporter nobody is watching never yields ([`anyone_watching`]).
///
/// The yields are only ever between kernel calls: the n-ary batch (or one pairwise boolean of a
/// winding-rule fold) is one uninterruptible call, so the frame it holds is still frozen for
/// however long that call takes.
pub(crate) async fn run_boolean_async<F: Future<Output = ()>, Y: Fn() -> F + ?Sized>(
    items: &[(&Mesh, Matrix4X4)],
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
    reporter: &ProgressReporter,
    ui_yield: Option<&Y>,
) -> Result<Mesh, BooleanError> {
    ensure_configured();
    let options = BooleanOptions {
        reporter: boolean_reporter_for(reporter),
        ..options.clone()
    };
    let mut batch = ManifoldOperandBatch::new(
        operation,
        options.mesh_colors.clone(),
        options.repair_orientation,
        options.single_precision,
    );

    for &(mesh, matrix) in items {
        if batch.try_add(mesh, matrix, cancel)? == AddOutcome::ResultIsEmpty {
            return Ok(Mesh::new());
        }

        // A mesh copy, a transform and an import each - seconds apiece on a large set, and all
        // of it before the boolean itself starts. The token is read with the yield because the
        // yield is what lets the user press Stop at all, so reading it here is what makes the
        // press land on this operand rather than after the import.
        throw_if_cancelled(cancel)?;
        yield_to_ui(reporter, ui_yield).await;
    }

    batch.throw_if_nothing_imported()?;

    let result =
        combine_and_read_async(&batch, operation, cancel, &options, reporter, ui_yield).await?;

    batch.throw_if_any_skipped(result)
}

/// [`combine_and_read`] with a yield on either side of the batch, or between each pair of a
/// winding-rule fold. Reading the result back is one export with no point inside it where handing
/// the frame away would leave the kernel's data half-read.
async fn combine_and_read_async<F: Future<Output = ()>, Y: Fn() -> F + ?Sized>(
    batch: &ManifoldOperandBatch,
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
    reporter: &ProgressReporter,
    ui_yield: Option<&Y>,
) -> Result<Mesh, BooleanError> {
    if batch.manifolds.is_empty() {
        return Ok(Mesh::new());
    }

    let operation_type = operation_type_of(operation);

    let combined: Manifold;
    let bool_result = if batch.manifolds.len() == 1 {
        &batch.manifolds[0]
    } else if needs_explicit_boolean(options.winding_rule) {
        combined = manifold_pairwise_fold::combine_pairwise_async(
            &batch.manifolds,
            operation_type,
            options.winding_rule,
            cancel,
            options.reporter.as_ref(),
            options.amount_per_operation,
            options.ratio_completed,
            reporter,
            ui_yield,
        )
        .await?;
        &combined
    } else {
        // The batch is one kernel call: its booleans stream their phases into the bar from
        // inside it (on whatever thread runs them), but nothing inside it can yield. Yield on
        // both sides of it: before, so the "combining" report paints before the frame is held;
        // after, so the finished bar paints and a Stop pressed while the call ran lands here
        // rather than after the result is read back.
        let progress = batch_progress_for(
            options.reporter.as_ref(),
            options.amount_per_operation,
            options.ratio_completed,
        );
        throw_if_cancelled(cancel)?;
        yield_to_ui(reporter, ui_yield).await;

        manifold_kernel_call_counts::count_operand_combine();
        combined = manifold_cancellable_boolean::batch_boolean(
            &batch.manifolds,
            operation_type,
            progress_sink_for(progress.as_ref()).as_ref(),
            cancel,
        )?;

        if let Some(progress) = &progress {
            progress.complete_operation(COMBINE_COMPLETE_PHASE);
        }
        throw_if_cancelled(cancel)?;
        yield_to_ui(reporter, ui_yield).await;
        &combined
    };

    manifold_result_reader::read_result(bool_result, batch)
}

/// Whether anything is listening to `reporter`. `ProgressReporter::null()` and a reporter built
/// around no target are both nobody.
pub(crate) fn anyone_watching(reporter: &ProgressReporter) -> bool {
    reporter.has_target()
}

/// The kernel-side sink for a reporter, or `None` when nobody is watching it - so every shape
/// of "nobody is watching" reaches the combine exactly as no reporter does.
fn boolean_reporter_for(reporter: &ProgressReporter) -> Option<BooleanReporter> {
    if !anyone_watching(reporter) {
        return None;
    }
    let reporter = reporter.clone();
    Some(Arc::new(move |ratio: f64, message: &str| {
        reporter.report(ratio, message)
    }))
}

/// Hands the UI its thread when the reporter's throttle lets the hop through.
pub(crate) async fn yield_to_ui<F: Future<Output = ()>, Y: Fn() -> F + ?Sized>(
    reporter: &ProgressReporter,
    ui_yield: Option<&Y>,
) {
    // The host's yield is only borrowed as a `dyn Fn` for the throttle's decision, never across
    // the await, so a caller whose yield is `Sync` keeps a `Send` future (a stepped compute job
    // on the model's worker pool).
    let hop = ui_yield.and_then(|ui_yield| {
        let call = || ui_yield();
        reporter.yield_to_ui(Some(&call as &dyn Fn() -> F))
    });
    if let Some(hop) = hop {
        hop.await;
    }
}

pub(crate) fn throw_if_cancelled(cancel: &CancelToken) -> Result<(), BooleanError> {
    if cancel.is_cancelled() {
        return Err(BooleanError::Cancelled);
    }
    Ok(())
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
        manifold_kernel_call_counts::count_operand_combine();
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
