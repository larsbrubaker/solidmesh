//! Constructive solid geometry over [`crate::Mesh`]: a port of agg-sharp `PolygonMesh/Csg`'s
//! polygon-mode booleans, which run on manifold-rust (agg-sharp runs ManifoldSharp, the C# port
//! of the same Rust source).
//!
//! [`csg_operations`] is the two-mesh convenience face and [`boolean_processing`] the public
//! n-ary entry point. Behind it, `manifold_kernel` drives one boolean: `manifold_operand_batch`
//! collects and imports the operands (`manifold_import`), `manifold_cancellable_boolean` and
//! `manifold_pairwise_fold` combine them, and `manifold_result_reader` reads the answer back
//! into a `Mesh`. `boolean_progress_adapter` maps the kernel's phases onto a caller's progress
//! window. This module holds the types they share: the operation, the failures and the
//! progress callback.
//!
//! Not yet here: the implicit-surface processing modes (`Marching_Cubes`, `Dual_Contouring`,
//! g3 implicits, Phase 2e), face-colour tracking through the run data (Phase 2f), the async
//! yielding entry points, Minkowski and `SliceLayer`.

pub mod boolean_processing;
mod boolean_progress_adapter;
pub mod csg_operations;
mod manifold_cancellable_boolean;
mod manifold_import;
mod manifold_kernel;
mod manifold_operand_batch;
mod manifold_pairwise_fold;
mod manifold_result_reader;

use std::fmt;
use std::sync::Arc;

use crate::Mesh;

pub use boolean_processing::BooleanOperandVerdict;
pub use manifold_rust::types::{Error as KernelStatus, WindingRule};

/// C# `CsgModes`: which boolean to run. An n-ary `Subtract` removes every later operand from
/// the first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CsgModes {
    Union,
    Subtract,
    Intersect,
}

/// A progress sink: ratio done (0..1) and a status message (C# `Action<double, string>`).
///
/// Shared and thread-safe because the kernel reports from inside the boolean, possibly from a
/// worker thread. `None` is "nobody is watching" (C# `ManifoldKernel.AnyoneWatching`).
pub type BooleanReporter = Arc<dyn Fn(f64, &str) + Send + Sync>;

/// The optional half of a boolean call (C# `DoArray`'s defaulted parameters).
#[derive(Clone)]
pub struct BooleanOptions {
    /// Progress sink; `None` when nobody is watching.
    pub reporter: Option<BooleanReporter>,
    /// The share of the caller's progress bar this boolean covers.
    pub amount_per_operation: f64,
    /// Where on the caller's progress bar this boolean starts.
    pub ratio_completed: f64,
    /// Which winding numbers the kernel counts as solid. `Nonzero` keeps inside-out shells as
    /// material rather than letting them cancel, at the cost of forcing the robust engine and a
    /// pairwise combine.
    pub winding_rule: WindingRule,
    /// Rewind each operand's inside-out shells before combining - the alternative to
    /// `Nonzero`, fixing the data once instead of redefining "solid".
    pub repair_orientation: bool,
}

impl Default for BooleanOptions {
    fn default() -> Self {
        Self {
            reporter: None,
            amount_per_operation: 1.0,
            ratio_completed: 0.0,
            winding_rule: WindingRule::Positive,
            repair_orientation: false,
        }
    }
}

impl fmt::Debug for BooleanOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BooleanOptions")
            .field("reporter", &self.reporter.is_some())
            .field("amount_per_operation", &self.amount_per_operation)
            .field("ratio_completed", &self.ratio_completed)
            .field("winding_rule", &self.winding_rule)
            .field("repair_orientation", &self.repair_orientation)
            .finish()
    }
}

/// An operand a union left out because the kernel refused its geometry (C#
/// `SkippedBooleanOperand`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkippedBooleanOperand {
    /// Zero-based position in the operand list.
    pub index: usize,
    /// Why the kernel refused it.
    pub reason: String,
}

/// Why a boolean did not produce its whole answer. C# throws these as exceptions.
#[derive(Debug, Clone)]
pub enum BooleanError {
    /// The caller's [`crate::CancelToken`] was cancelled (C# `OperationCanceledException`).
    Cancelled,
    /// The kernel refused an operand's geometry (C# `MeshImportRejectedException`). A union
    /// skips such operands instead (see [`BooleanError::Partial`]); subtract and intersect are
    /// defined by every operand, so they fail.
    ImportRejected {
        message: String,
        status: KernelStatus,
    },
    /// A union combined the operands that imported and left the rest out (C#
    /// `PartialBooleanException`). Still an error, so no caller loses a part quietly; the
    /// result is carried so a caller that accepts the loss can use it.
    Partial {
        message: String,
        partial_result: Box<Mesh>,
        skipped_operands: Vec<SkippedBooleanOperand>,
    },
    /// The kernel's result carried an error status (C# `InvalidOperationException`).
    Kernel { message: String },
}

impl fmt::Display for BooleanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BooleanError::Cancelled => write!(f, "The operation was canceled."),
            BooleanError::ImportRejected { message, .. }
            | BooleanError::Partial { message, .. }
            | BooleanError::Kernel { message } => write!(f, "{message}"),
        }
    }
}

impl std::error::Error for BooleanError {}
