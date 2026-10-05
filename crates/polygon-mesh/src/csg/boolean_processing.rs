//! The public entry points for booleans over [`Mesh`]es. Port of agg-sharp
//! `Csg/BooleanProcessing.cs` (polygon mode: `DoArray`, `Do`, `ClassifyBooleanOperand`) and
//! `Csg/BooleanOperandVerdict.cs`.
//!
//! Polygon mode runs on manifold-rust, the one and only boolean kernel (`manifold_kernel`).
//! There is no second engine behind it: an input the kernel cannot take is an error the caller
//! sees, not geometry built by different rules. The implicit-surface processing modes
//! (`ProcessingModes.Marching_Cubes`/`Dual_Contouring`, which resample the operands through g3
//! implicits) come with the Sdf/DualContouring slice; every call here is polygon mode.

use vector_math::Matrix4X4;

use super::{manifold_import, manifold_kernel, BooleanError, BooleanOptions, CsgModes};
use crate::{CancelToken, Mesh};

/// What the kernel makes of a mesh offered to it as a boolean operand (C#
/// `BooleanOperandVerdict`).
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
    // Every input goes to the kernel, and whatever it says is the answer: no manifold pre-gate
    // (the robust import plus the Auto engine handle closed non-manifold geometry) and no
    // fallback (a mesh the kernel refused is better reported than silently replaced).
    manifold_kernel::run_boolean(items, operation, cancel, options)
}

/// The two-operand spelling of [`do_array`].
pub fn do_boolean(
    mesh_a: &Mesh,
    matrix_a: Matrix4X4,
    mesh_b: &Mesh,
    matrix_b: Matrix4X4,
    operation: CsgModes,
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
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
