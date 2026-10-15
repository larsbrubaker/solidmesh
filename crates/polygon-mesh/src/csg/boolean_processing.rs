//! The public entry points for booleans over [`Mesh`]es: the n-ary boolean, its yielding variant,
//! the two-operand spelling, and the operand classifier.
//!
//! Polygon mode runs on manifold-rust, the one and only boolean kernel (`manifold_kernel`).
//! There is no second engine behind it: an input the kernel cannot take is an error the caller
//! sees, not geometry built by different rules. The implicit-surface processing modes
//! ([`super::ProcessingModes`], chosen in [`BooleanOptions`]) resample the operands as implicit
//! surfaces instead (`implicit_boolean`); they never fail.

use std::future::Future;

use agg_path::progress_reporter::ProgressReporter;
use vector_math::Matrix4X4;

use super::{
    implicit_boolean, manifold_import, manifold_kernel, BooleanError, BooleanOptions, CsgModes,
    ProcessingModes,
};
use crate::{CancelToken, Mesh};

/// What the kernel makes of a mesh offered to it as a boolean operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BooleanOperandVerdict {
    /// The kernel would not take it at all.
    Refused,
    /// It imports, but the solid intersects itself - which forces the slow robust engine.
    SelfIntersecting,
    /// It imports and is clean.
    Clean,
}

/// Combines every item (a mesh and the transform applied to it first) with one n-ary boolean.
/// `Subtract` removes every later operand from the first.
pub fn do_array(
    items: &[(&Mesh, Matrix4X4)],
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    if options.processing_mode != ProcessingModes::Polygons {
        return Ok(implicit_boolean::as_implicit_meshes(
            items,
            operation,
            options.processing_mode,
            options.input_resolution,
            options.output_resolution,
        ));
    }
    // Every input goes to the kernel, and whatever it says is the answer: no manifold pre-gate
    // (the robust import plus the Auto engine handle closed non-manifold geometry) and no
    // fallback (a mesh the kernel refused is better reported than silently replaced).
    manifold_kernel::run_boolean(items, operation, cancel, options)
}

/// [`do_array`] for a job that can hand the UI its thread back: the same
/// boolean, yielding through `reporter` after each operand's import and around the combine.
///
/// `reporter` is the progress sink and replaces `options.reporter`; the host's yield is `ui_yield`,
/// passed in rather than read from a process-global hook so callers and tests running side by side
/// never share one, and a hop runs only when `reporter`'s throttle lets it through. Nobody
/// watching -
/// [`ProgressReporter::null`], or a reporter with no target - never yields, and watched or not
/// the kernel takes the same n-ary route. The implicit-surface modes report nothing and have no
/// per-operand boundary to hand a frame back at, so for them this is [`do_array`].
///
/// `ui_yield` is any `Fn` (a `&dyn Fn` included): with a `Send + Sync` yield and `Send` inputs
/// the returned future is `Send`, so a job that owns its operands can be polled step by step
/// wherever it runs.
pub async fn do_array_async<F: Future<Output = ()>, Y: Fn() -> F + ?Sized>(
    items: &[(&Mesh, Matrix4X4)],
    operation: CsgModes,
    reporter: &ProgressReporter,
    ui_yield: Option<&Y>,
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    if options.processing_mode != ProcessingModes::Polygons {
        return do_array(items, operation, cancel, options);
    }
    manifold_kernel::run_boolean_async(items, operation, cancel, options, reporter, ui_yield).await
}

/// The two-operand spelling of [`do_array`]. In an implicit mode it meshes with plain marching
/// cubes whichever implicit mode is asked for, and an operand with fewer than four faces gives back
/// the other operand.
pub fn do_boolean(
    mesh_a: &Mesh,
    matrix_a: Matrix4X4,
    mesh_b: &Mesh,
    matrix_b: Matrix4X4,
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    if options.processing_mode != ProcessingModes::Polygons {
        return Ok(implicit_boolean::two_operand_implicit(
            mesh_a,
            matrix_a,
            mesh_b,
            matrix_b,
            operation,
            options.input_resolution,
            options.output_resolution,
        ));
    }
    do_array(
        &[(mesh_a, matrix_a), (mesh_b, matrix_b)],
        operation,
        cancel,
        options,
    )
}

/// Whether the kernel would take `mesh` (under `matrix`) as a boolean operand, and if so
/// whether the solid intersects itself.
///
/// Self-intersecting operands are the difference between a union that finishes and one that
/// does not: they force the kernel off its exact pipeline onto the robust one. The import is
/// the same one [`do_array`] performs - transform first - so the verdict is about the manifold
/// the boolean would actually see. A mirroring matrix can change the answer on its own.
pub fn classify_boolean_operand(
    mesh: &Mesh,
    matrix: Matrix4X4,
    repair_orientation: bool,
) -> BooleanOperandVerdict {
    if mesh.faces.is_empty() || mesh.vertices.is_empty() {
        return BooleanOperandVerdict::Refused;
    }

    let mut mesh_copy = mesh.copy();
    mesh_copy.transform(matrix);

    match manifold_import::import(&mesh_copy, repair_orientation) {
        Ok(imported) if imported.has_self_intersections() => {
            BooleanOperandVerdict::SelfIntersecting
        }
        Ok(_) => BooleanOperandVerdict::Clean,
        // A mesh the kernel cannot be made to hold is one it would refuse as an operand too.
        Err(_) => BooleanOperandVerdict::Refused,
    }
}
