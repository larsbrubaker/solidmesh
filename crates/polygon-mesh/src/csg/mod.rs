//! Constructive solid geometry over [`crate::Mesh`]: polygon-mode booleans, which run on
//! manifold-rust.
//!
//! [`csg_operations`] is the two-mesh convenience face and [`boolean_processing`] the public
//! n-ary entry point. Behind it, `manifold_kernel` drives one boolean: `manifold_operand_batch`
//! collects and imports the operands (`manifold_import`), `manifold_cancellable_boolean` and
//! `manifold_pairwise_fold` combine them, and `manifold_result_reader` reads the answer back
//! into a `Mesh`. [`manifold_kernel_call_counts`] counts the kernel evaluations by kind, for
//! tests. `boolean_progress_adapter` maps the kernel's phases onto a caller's progress
//! window. This module holds the types they share: the operation, the failures and the
//! progress callback.
//!
//! The implicit-surface processing modes ([`ProcessingModes::MarchingCubes`],
//! [`ProcessingModes::DualContouring`]) resample the operands instead (`implicit_boolean`).
//!
//! The Minkowski sum and difference are [`minkowski_morph`] (operand import, shell repair and
//! the ball in [`minkowski_processing`], the bar's time weighting in
//! [`minkowski_progress_model`]). [`slice_layer`] cuts a mesh into integer polygons and
//! [`path_stitcher`] walls polygon loops back into a mesh. [`mesh_repair_rust`] is the
//! kernel-backed repair a caller asks for directly.

pub mod boolean_processing;
mod boolean_progress_adapter;
pub mod consistent_winding;
pub mod csg_operations;
mod implicit_boolean;
mod manifold_cancellable_boolean;
pub mod manifold_import;
mod manifold_kernel;
pub mod manifold_kernel_call_counts;
mod manifold_operand_batch;
mod manifold_pairwise_fold;
pub mod manifold_result_reader;
pub mod mesh_repair_rust;
pub mod minkowski_morph;
pub mod minkowski_processing;
pub mod minkowski_progress_model;
pub mod minkowski_shell_orientation;
pub mod path_stitcher;
pub mod slice_layer;

use std::fmt;
use std::sync::Arc;

use crate::Mesh;

pub use boolean_processing::BooleanOperandVerdict;
pub use manifold_rust::types::{Error as KernelStatus, WindingRule};

/// Which boolean to run. An n-ary `Subtract` removes every later operand from the first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CsgModes {
    Union,
    Subtract,
    Intersect,
}

/// How a boolean is computed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ProcessingModes {
    /// "Default CSG processing": the exact polygon kernel (manifold-rust).
    #[default]
    Polygons,
    /// "Experimental Marching Cubes": the operands resampled as signed distance grids and the
    /// result meshed with marching cubes.
    MarchingCubes,
    /// "Experimental Dual Contouring": the operands resampled as signed distance grids and the
    /// result meshed with octree dual contouring.
    DualContouring,
}

/// The grid resolution of the implicit modes. The value is the exponent (`_64 = 6`); the cell count
/// across the largest dimension is `1 << value`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ProcessingResolution {
    #[default]
    R64 = 6,
    R128 = 7,
    R256 = 8,
    R512 = 9,
}

impl ProcessingResolution {
    /// The exponent.
    pub fn exponent(self) -> i32 {
        self as i32
    }

    /// The cell count.
    pub fn cells(self) -> i32 {
        1 << self.exponent()
    }
}

/// A progress sink: ratio done (0..1) and a status message.
///
/// Shared and thread-safe because the kernel reports from inside the boolean, possibly from a
/// worker thread. `None` is "nobody is watching".
pub type BooleanReporter = Arc<dyn Fn(f64, &str) + Send + Sync>;

/// The optional half of a boolean call: everything with a sensible default.
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
    /// Polygons (the kernel) unless an implicit mode is asked for. The implicit modes ignore the
    /// reporter, the cancel token, the winding rule and the repair.
    pub processing_mode: ProcessingModes,
    /// How finely the implicit modes sample each operand.
    pub input_resolution: ProcessingResolution,
    /// How finely the implicit modes mesh the result.
    pub output_resolution: ProcessingResolution,
    /// One color per operand, in operand order, to paint the result's faces from; `None` asks for
    /// no color tracking (the result has no face colors). An operand that has face colors of its
    /// own is painted from those instead. Polygon mode only.
    pub mesh_colors: Option<Vec<agg_path::Color>>,
}

impl Default for BooleanOptions {
    fn default() -> Self {
        Self {
            reporter: None,
            amount_per_operation: 1.0,
            ratio_completed: 0.0,
            winding_rule: WindingRule::Positive,
            repair_orientation: false,
            processing_mode: ProcessingModes::Polygons,
            input_resolution: ProcessingResolution::R64,
            output_resolution: ProcessingResolution::R64,
            mesh_colors: None,
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
            .field("processing_mode", &self.processing_mode)
            .field("input_resolution", &self.input_resolution)
            .field("output_resolution", &self.output_resolution)
            .field("mesh_colors", &self.mesh_colors)
            .finish()
    }
}

/// An operand a union left out because the kernel refused its geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkippedBooleanOperand {
    /// Zero-based position in the operand list.
    pub index: usize,
    /// Why the kernel refused it.
    pub reason: String,
}

/// Why a boolean did not produce its whole answer.
#[derive(Debug, Clone)]
pub enum BooleanError {
    /// The caller's [`crate::CancelToken`] was cancelled.
    Cancelled,
    /// The kernel refused an operand's geometry. A union skips such operands instead (see
    /// [`BooleanError::Partial`]); subtract and intersect are defined by every operand, so they
    /// fail.
    ImportRejected {
        message: String,
        status: KernelStatus,
    },
    /// A union combined the operands that imported and left the rest out. Still an error, so no
    /// caller loses a part quietly; the result is carried so a caller that accepts the loss can use
    /// it.
    Partial {
        message: String,
        partial_result: Box<Mesh>,
        skipped_operands: Vec<SkippedBooleanOperand>,
    },
    /// The kernel's result carried an error status.
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
