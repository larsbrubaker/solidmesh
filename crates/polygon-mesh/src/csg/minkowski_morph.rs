//! agg-sharp `Csg/MinkowskiProcessing.cs`'s Minkowski runs: `MinkowskiSum`,
//! `MinkowskiDifference` (with and without the `ErosionPath` it took), their progress and
//! cancellation, and the kernel routing (`Run`, `Morph`): the closed form for a convex
//! erosion, the parallel union tree for a dilation or erosion by a convex tool, the sweep for
//! the rest. The operand import and shell repair they share live in `minkowski_processing.rs`;
//! the bar's time weighting is `minkowski_progress_model.rs`.
//!
//! C#'s async entry points (`MinkowskiSumAsync`, `MinkowskiDifferenceWithPathAsync`) only put
//! this same call on a worker and yield around it; a Rust caller runs these on its own worker
//! (`RebuildCx::compute`), so the synchronous calls with a reporter and token are the port of
//! both.

use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, OnceLock};

use manifold_rust::manifold::Manifold;
use manifold_rust::progress::{Phase, ProgressReporter};

use super::boolean_progress_adapter::BooleanProgressAdapter;
use super::minkowski_processing::{import_operand, throw_if_empty, MinkowskiError};
use super::{
    manifold_kernel, manifold_result_reader, minkowski_progress_model, minkowski_shell_orientation,
    BooleanReporter, KernelStatus,
};
use crate::{CancelToken, Mesh};

/// C# `ErosionPath`: which algorithm an erosion actually ran. A cost signal, not a
/// correctness one - both paths compute the same erosion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ErosionPath {
    /// The halfspace-intersection closed form: two convex hulls, whatever the triangle count.
    ClosedForm,
    /// The per-triangle sweep, or the erosion tree that reduces the same hulls (the same cost
    /// regime).
    Sweep,
}

static DILATION_TREE_RUNS: AtomicI64 = AtomicI64::new(0);
static EROSION_TREE_RUNS: AtomicI64 = AtomicI64::new(0);

/// C# `DilationTreeRuns`: how many dilations in this process took the kernel's parallel union
/// tree rather than the sweep. A path signal for tests.
pub fn dilation_tree_runs() -> i64 {
    DILATION_TREE_RUNS.load(Ordering::SeqCst)
}

/// C# `ErosionTreeRuns`: the erosion counterpart of [`dilation_tree_runs`].
pub fn erosion_tree_runs() -> i64 {
    EROSION_TREE_RUNS.load(Ordering::SeqCst)
}

/// C# `MinkowskiSum(solid, tool)`: dilation, every point of `solid` swept by `tool`.
pub fn minkowski_sum(solid: &Mesh, tool: &Mesh) -> Result<Mesh, MinkowskiError> {
    run(solid, tool, false, None, None).map(|(mesh, _)| mesh)
}

/// C# `MinkowskiSumAsync`: [`minkowski_sum`] reporting to `reporter` and stopping between
/// hulls when `cancel` is signalled (`Err(Cancelled)`; a run that finished first wins).
pub fn minkowski_sum_with_progress(
    solid: &Mesh,
    tool: &Mesh,
    reporter: Option<BooleanReporter>,
    cancel: &CancelToken,
) -> Result<Mesh, MinkowskiError> {
    run(solid, tool, false, reporter, Some(cancel)).map(|(mesh, _)| mesh)
}

/// C# `MinkowskiDifference(solid, tool)`: erosion, the points of `solid` that `tool` still
/// fits inside. A tool too large erodes the solid away: an empty mesh, not a failure.
pub fn minkowski_difference(solid: &Mesh, tool: &Mesh) -> Result<Mesh, MinkowskiError> {
    run(solid, tool, true, None, None).map(|(mesh, _)| mesh)
}

/// C# `MinkowskiDifference(solid, tool, out erosionPath)`.
pub fn minkowski_difference_with_path(
    solid: &Mesh,
    tool: &Mesh,
) -> Result<(Mesh, ErosionPath), MinkowskiError> {
    run(solid, tool, true, None, None)
}

/// C# `MinkowskiDifferenceWithPathAsync`: the erosion with progress and cancellation, and the
/// algorithm that produced it.
pub fn minkowski_difference_with_path_and_progress(
    solid: &Mesh,
    tool: &Mesh,
    reporter: Option<BooleanReporter>,
    cancel: &CancelToken,
) -> Result<(Mesh, ErosionPath), MinkowskiError> {
    run(solid, tool, true, reporter, Some(cancel))
}

/// C# `Run`: refuse empty operands, import and shell-repair both, refuse an operand that
/// imported with no volume, then morph.
fn run(
    solid: &Mesh,
    tool: &Mesh,
    inset: bool,
    reporter: Option<BooleanReporter>,
    cancel: Option<&CancelToken>,
) -> Result<(Mesh, ErosionPath), MinkowskiError> {
    // Empty is not a degenerate case the kernel answers usefully: Minkowski with an empty
    // operand is empty, but the kernel hands back the *other* operand unchanged. That is a
    // silent no-op on a fillet, so it is refused here instead.
    throw_if_empty(solid)?;
    throw_if_empty(tool)?;

    let never = CancelToken::new();
    let repair_token = cancel.unwrap_or(&never);
    let solid_manifold = minkowski_shell_orientation::repair(import_operand(solid)?, repair_token)
        .ok_or(MinkowskiError::Cancelled)?;
    let tool_manifold = minkowski_shell_orientation::repair(import_operand(tool)?, repair_token)
        .ok_or(MinkowskiError::Cancelled)?;

    // A zero-thickness shell is closed, passes every mesh check and imports with no error at
    // all, as an empty manifold - the input the kernel's own early exit answers with the other
    // operand's clone, so it has to be caught after the import or not at all.
    throw_if_no_volume(&solid_manifold)?;
    throw_if_no_volume(&tool_manifold)?;

    let (result, erosion_path) = morph(&solid_manifold, &tool_manifold, inset, reporter, cancel)?;
    let name = if inset {
        "minkowski difference"
    } else {
        "minkowski sum"
    };
    Ok((
        manifold_result_reader::to_mesh(&result, name)?,
        erosion_path,
    ))
}

/// C# `Morph`: the kernel call with the token bridged and the phase callback adapted. A
/// cancelled run (the kernel's `Cancelled` status on an empty result) is `Err(Cancelled)`;
/// completion wins over a late cancel.
fn morph(
    solid: &Manifold,
    tool: &Manifold,
    inset: bool,
    reporter: Option<BooleanReporter>,
    cancel: Option<&CancelToken>,
) -> Result<(Manifold, ErosionPath), MinkowskiError> {
    manifold_kernel::ensure_configured();
    // ErosionPath names erosion algorithms only; on a sum this only gives it a defined value.
    let mut erosion_path = ErosionPath::Sweep;

    let adapter = reporter.map(|reporter| {
        Arc::new(BooleanProgressAdapter::with_label(
            reporter,
            0.0,
            1.0,
            1,
            if inset { "Erode" } else { "Dilate" },
        ))
    });

    // The kernel counts work items; MinkowskiProgressModel reweights them by their measured
    // cost so the bar tracks the clock.
    let solid_triangles = i32::try_from(solid.num_tri()).unwrap_or(i32::MAX);
    let progress = adapter.as_ref().map(|adapter| {
        let adapter = Arc::clone(adapter);
        ProgressReporter::new(move |phase: Phase, fraction: Option<f64>| {
            adapter.report(
                phase.name(),
                fraction.map(|f| minkowski_progress_model::time_fraction(solid_triangles, f)),
            );
        })
    });

    // The union tree counts different units and reports them from worker threads, so it gets
    // its own time mapping; its hull count is recovered from the phase total the kernel set,
    // read back through the reporter itself (C# closes over the variable it assigns).
    let tree_parallelism = if cfg!(target_arch = "wasm32") {
        1
    } else {
        std::thread::available_parallelism()
            .map(|n| i32::try_from(n.get()).unwrap_or(1))
            .unwrap_or(1)
    };
    let tree_progress = adapter.as_ref().map(|adapter| {
        let adapter = Arc::clone(adapter);
        let me: Arc<OnceLock<Arc<ProgressReporter>>> = Arc::new(OnceLock::new());
        let me_in_callback = Arc::clone(&me);
        let reporter = Arc::new(ProgressReporter::new(
            move |phase: Phase, fraction: Option<f64>| {
                let total = me_in_callback
                    .get()
                    .map(|r| i64::try_from(r.phase_total()).unwrap_or(i64::MAX))
                    .unwrap_or(0);
                adapter.report(
                    phase.name(),
                    fraction.map(|f| {
                        minkowski_progress_model::tree_time_fraction(
                            minkowski_progress_model::tree_hull_units(total, inset),
                            f,
                            tree_parallelism,
                            inset,
                        )
                    }),
                );
            },
        ));
        // Set before the kernel runs, so every report finds it.
        let _ = me.set(Arc::clone(&reporter));
        reporter
    });

    let token = cancel.map(CancelToken::kernel_token);
    let progress_ref = progress.as_ref();
    let tree_ref = tree_progress.as_deref();

    let result = if let Some(closed_form) = inset
        .then(|| solid.try_convex_erosion(tool, token, progress_ref))
        .flatten()
    {
        erosion_path = ErosionPath::ClosedForm;
        closed_form
    } else if let Some(tree_sum) = (!inset)
        .then(|| solid.try_dilate_by_convex(tool, token, tree_ref))
        .flatten()
    {
        DILATION_TREE_RUNS.fetch_add(1, Ordering::SeqCst);
        tree_sum
    } else if let Some(tree_difference) = inset
        .then(|| solid.try_erode_by_convex(tool, token, tree_ref))
        .flatten()
    {
        EROSION_TREE_RUNS.fetch_add(1, Ordering::SeqCst);
        tree_difference
    } else if inset {
        solid.minkowski_difference_with_progress(tool, token, progress_ref)
    } else {
        solid.minkowski_sum_with_progress(tool, token, progress_ref)
    };

    if token.is_some() && result.status() == KernelStatus::Cancelled {
        return Err(MinkowskiError::Cancelled);
    }
    Ok((result, erosion_path))
}

/// C# `ThrowIfNoVolume`.
fn throw_if_no_volume(manifold: &Manifold) -> Result<(), MinkowskiError> {
    if manifold.is_empty() {
        return Err(MinkowskiError::InvalidArgument(
            "A Minkowski operand needs volume; this mesh is closed but encloses nothing, so the kernel holds it as empty.".to_string(),
        ));
    }
    Ok(())
}
