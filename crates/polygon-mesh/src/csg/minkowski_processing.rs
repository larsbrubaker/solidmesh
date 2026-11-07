//! The Minkowski operand side: [`orient_shells_as_solid`] (the solid exactly as Dilate and Erode
//! read it), with the operand import it shares with the Minkowski runs (`import_operand` and its
//! backward-patch retry, `throw_if_empty`), and [`sphere_mesh`]. The Minkowski sum and difference
//! themselves are `minkowski_morph.rs`.

use std::fmt;

use manifold_rust::manifold::Manifold;

use super::manifold_import::{self, ImportRejected};
use super::{consistent_winding, manifold_result_reader, minkowski_shell_orientation};
use super::{BooleanError, KernelStatus};
use crate::{CancelToken, Mesh};

/// Why a Minkowski-family call produced no mesh.
#[derive(Debug, Clone)]
pub enum MinkowskiError {
    /// An argument was out of range or an operand had no geometry.
    InvalidArgument(String),
    /// The kernel refused an operand.
    ImportRejected {
        message: String,
        status: KernelStatus,
    },
    /// The caller's token was cancelled.
    Cancelled,
    /// The kernel could not build the result.
    Kernel { message: String },
}

impl fmt::Display for MinkowskiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MinkowskiError::Cancelled => write!(f, "The operation was canceled."),
            MinkowskiError::InvalidArgument(message)
            | MinkowskiError::ImportRejected { message, .. }
            | MinkowskiError::Kernel { message } => write!(f, "{message}"),
        }
    }
}

impl std::error::Error for MinkowskiError {}

impl From<ImportRejected> for MinkowskiError {
    fn from(rejected: ImportRejected) -> Self {
        MinkowskiError::ImportRejected {
            message: rejected.message,
            status: rejected.status,
        }
    }
}

impl From<BooleanError> for MinkowskiError {
    fn from(error: BooleanError) -> Self {
        match error {
            BooleanError::Cancelled => MinkowskiError::Cancelled,
            BooleanError::ImportRejected { message, status } => {
                MinkowskiError::ImportRejected { message, status }
            }
            other => MinkowskiError::Kernel {
                message: other.to_string(),
            },
        }
    }
}

/// The kernel's ball as a mesh, centred on the origin; 0 segments asks the kernel to choose.
pub fn sphere_mesh(radius: f64, segments: i32) -> Result<Mesh, MinkowskiError> {
    if radius.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return Err(MinkowskiError::InvalidArgument(
            "A sphere needs a positive radius.".to_string(),
        ));
    }
    if segments < 0 {
        return Err(MinkowskiError::InvalidArgument(
            "A sphere needs a non-negative segment count; 0 asks the kernel to choose.".to_string(),
        ));
    }
    Ok(manifold_result_reader::to_mesh(
        &Manifold::sphere(radius, segments),
        "sphere",
    )?)
}

/// The solid imported as Dilate and Erode import it (a backward patch rewound, split seams joined)
/// with every shell whose winding contradicts its nesting turned the right way, so an inside-out
/// part comes back as the solid it looks like while a correctly wound cavity stays a cavity.
pub fn orient_shells_as_solid(solid: &Mesh, cancel: &CancelToken) -> Result<Mesh, MinkowskiError> {
    throw_if_empty(solid)?;
    let oriented = minkowski_shell_orientation::repair(import_operand(solid)?, cancel)
        .ok_or(MinkowskiError::Cancelled)?;
    Ok(manifold_result_reader::to_mesh(
        &oriented,
        "shell orientation",
    )?)
}

/// The import with the backward-patch retry, and for a soup import one more try with exactly
/// coincident vertices joined, kept only if that pairs up.
pub(crate) fn import_operand(mesh: &Mesh) -> Result<Manifold, MinkowskiError> {
    let imported = import_rewinding_a_backward_patch(mesh)?;
    if imported.as_impl().is_soup {
        if let Some(joined) = consistent_winding::join_coincident_vertices(mesh) {
            if let Some(paired) = try_import(&joined) {
                if !paired.as_impl().is_soup {
                    return Ok(paired);
                }
            }
        }
    }
    Ok(imported)
}

/// A `NotClosed` refusal gets the rewound copy, then the tolerance-welded rewound copy; failing
/// both, the original refusal stands.
fn import_rewinding_a_backward_patch(mesh: &Mesh) -> Result<Manifold, ImportRejected> {
    match manifold_import::import(mesh, false) {
        Ok(imported) => Ok(imported),
        Err(refused) if refused.status == KernelStatus::NotClosed => {
            if let Some(imported) = consistent_winding::rewind(mesh).and_then(|m| try_import(&m)) {
                return Ok(imported);
            }
            let welded_rewound =
                manifold_import::weld_seams(mesh).and_then(|w| consistent_winding::rewind(&w));
            if let Some(imported) = welded_rewound.and_then(|m| try_import(&m)) {
                return Ok(imported);
            }
            Err(refused)
        }
        Err(refused) => Err(refused),
    }
}

fn try_import(mesh: &Mesh) -> Option<Manifold> {
    manifold_import::import(mesh, false).ok()
}

pub(crate) fn throw_if_empty(mesh: &Mesh) -> Result<(), MinkowskiError> {
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return Err(MinkowskiError::InvalidArgument(
            "A Minkowski operand needs geometry; this mesh has none.".to_string(),
        ));
    }
    Ok(())
}
