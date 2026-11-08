//! What stops [`bevel_part`](super::bevel_part) outright ([`BevelPartError`]): the caller's cancel,
//! or a build that could not run at all ([`BevelBuildFailure`], with a coarse
//! [`BevelBuildFailureKind`] and a message). A refusal is neither: it is a row of the
//! [`BevelPartOutcome`](super::BevelPartOutcome) with a code.
//!
//! The failure is opaque on purpose. What went wrong inside is one of the kernel's own detail
//! types (a level-line work budget that ran out, a surface construction that could not start, a
//! Point group's fault), which change as the constructions do, so the documented error carries
//! only a kind and the words; [`BevelBuildFailure::cause`] (hidden from the docs) hands the
//! detail to MatterCAD and the kernel's tests.

use super::surface_part_builder::SurfacePartBuildError;
use crate::bevel::level_line::point_bevel::PointBevelFault;
use crate::bevel::level_line::WorkError;

/// What stops a part's build outright. Only [`BevelPartError::Cancelled`] is the caller's to pass
/// on; for a [`BevelPartError::Failed`] the rows to show are
/// [`BevelPartOutcome::every_group_refused`](super::BevelPartOutcome::every_group_refused).
#[derive(Debug)]
pub enum BevelPartError {
    /// The cancellation token was cancelled.
    Cancelled,
    /// The build could not run.
    Failed(BevelBuildFailure),
}

/// Why a part's build could not run: a [`kind`](Self::kind) to act on and a message
/// ([`Display`](std::fmt::Display)) for a log.
#[derive(Debug)]
pub struct BevelBuildFailure {
    cause: BevelBuildCause,
}

/// What kind of failure stopped a build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BevelBuildFailureKind {
    /// The build ran out of the work it is allowed (a pick that would take unreasonably long).
    WorkBudget,
    /// A construction could not run on this part (a kernel check failed, or a point was not a
    /// number).
    Construction,
}

/// The kernel's own detail of a [`BevelBuildFailure`]: which construction failed and how. Not
/// part of the documented API; it changes with the constructions.
#[doc(hidden)]
#[derive(Debug)]
pub enum BevelBuildCause {
    /// A level-line walk ran out of its budget.
    Work(WorkError),
    /// The surface construction could not run.
    Surface(SurfacePartBuildError),
    /// A Point group's bevel left by a fault.
    Point(PointBevelFault),
}

impl BevelBuildFailure {
    /// What kind of failure this is.
    pub fn kind(&self) -> BevelBuildFailureKind {
        match &self.cause {
            BevelBuildCause::Work(_)
            | BevelBuildCause::Surface(SurfacePartBuildError::BudgetExceeded)
            | BevelBuildCause::Point(PointBevelFault::Work(_)) => BevelBuildFailureKind::WorkBudget,
            BevelBuildCause::Surface(_) | BevelBuildCause::Point(_) => {
                BevelBuildFailureKind::Construction
            }
        }
    }

    /// The kernel's detail of the failure, for MatterCAD and the kernel's tests.
    #[doc(hidden)]
    pub fn cause(&self) -> &BevelBuildCause {
        &self.cause
    }
}

impl std::fmt::Display for BevelBuildFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.cause {
            BevelBuildCause::Work(error) => write!(f, "{error}"),
            BevelBuildCause::Surface(error) => write!(f, "{error:?}"),
            BevelBuildCause::Point(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for BevelBuildFailure {}

impl std::fmt::Display for BevelPartError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BevelPartError::Cancelled => f.write_str("The operation was canceled."),
            BevelPartError::Failed(failure) => write!(f, "{failure}"),
        }
    }
}

impl std::error::Error for BevelPartError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            BevelPartError::Cancelled => None,
            BevelPartError::Failed(failure) => Some(failure),
        }
    }
}

impl BevelPartError {
    fn failed(cause: BevelBuildCause) -> Self {
        BevelPartError::Failed(BevelBuildFailure { cause })
    }
}

impl From<PointBevelFault> for BevelPartError {
    fn from(error: PointBevelFault) -> Self {
        match error {
            PointBevelFault::Work(WorkError::Cancelled) => BevelPartError::Cancelled,
            other => BevelPartError::failed(BevelBuildCause::Point(other)),
        }
    }
}

impl From<SurfacePartBuildError> for BevelPartError {
    fn from(error: SurfacePartBuildError) -> Self {
        match error {
            SurfacePartBuildError::Cancelled => BevelPartError::Cancelled,
            other => BevelPartError::failed(BevelBuildCause::Surface(other)),
        }
    }
}

impl From<WorkError> for BevelPartError {
    fn from(error: WorkError) -> Self {
        match error {
            WorkError::Cancelled => BevelPartError::Cancelled,
            other => BevelPartError::failed(BevelBuildCause::Work(other)),
        }
    }
}
