//! What stops [`bevel_part`](super::bevel_part) outright ([`BevelPartError`]): the caller's cancel,
//! or a build that could not run at all ([`BevelBuildFailure`], with a coarse
//! [`BevelBuildFailureKind`] and a message). A refusal is neither: it is a row of the
//! [`BevelPartOutcome`](super::BevelPartOutcome) with a code.
//!
//! The failure is opaque on purpose. What went wrong inside is one of the kernel's own detail
//! types (a level-line work budget that ran out, a surface construction that could not start, a
//! Point group's fault), which change as the constructions do and are not `Send` (a work budget
//! is shared by `Rc`). So the failure is read off the detail where it is made, at the `From`
//! conversions below, into owned data: its kind, its words, and the detail's `Debug` text for a
//! log. Both error types are therefore `Send + Sync + 'static`, so a caller can move them across
//! threads or box them as `Box<dyn Error + Send + Sync>`.

use super::surface_part_builder::SurfacePartBuildError;
use crate::bevel::level_line::point_bevel::PointBevelFault;
use crate::bevel::level_line::WorkError;

/// What stops a part's build outright. Only [`BevelPartError::Cancelled`] is the caller's to pass
/// on; for a [`BevelPartError::Failed`] the rows to show are
/// [`BevelPartOutcome::every_group_refused`](super::BevelPartOutcome::every_group_refused).
/// Later releases may add variants, so a `match` needs a catch-all arm.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum BevelPartError {
    /// The cancellation token was cancelled.
    Cancelled,
    /// The build could not run.
    Failed(BevelBuildFailure),
}

/// Why a part's build could not run: a [`kind`](Self::kind) to act on and a message
/// ([`Display`](std::fmt::Display)) for a log. Its `Debug` also shows the kernel's own detail.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BevelBuildFailure {
    kind: BevelBuildFailureKind,
    message: String,
    detail: String,
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

impl BevelBuildFailure {
    /// What kind of failure this is.
    pub fn kind(&self) -> BevelBuildFailureKind {
        self.kind
    }

    /// A failure of `kind` worded `message`, with the kernel's detail as its `Debug` text.
    fn read_off(
        kind: BevelBuildFailureKind,
        message: String,
        detail: &dyn std::fmt::Debug,
    ) -> Self {
        BevelBuildFailure {
            kind,
            message,
            detail: format!("{detail:?}"),
        }
    }
}

impl std::fmt::Display for BevelBuildFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
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

impl From<PointBevelFault> for BevelPartError {
    fn from(error: PointBevelFault) -> Self {
        let kind = match &error {
            PointBevelFault::Work(WorkError::Cancelled) => return BevelPartError::Cancelled,
            PointBevelFault::Work(_) => BevelBuildFailureKind::WorkBudget,
            PointBevelFault::Defect(_) => BevelBuildFailureKind::Construction,
        };
        BevelPartError::Failed(BevelBuildFailure::read_off(kind, error.to_string(), &error))
    }
}

impl From<SurfacePartBuildError> for BevelPartError {
    fn from(error: SurfacePartBuildError) -> Self {
        let kind = match &error {
            SurfacePartBuildError::Cancelled => return BevelPartError::Cancelled,
            SurfacePartBuildError::BudgetExceeded => BevelBuildFailureKind::WorkBudget,
            SurfacePartBuildError::LevelLine(_) | SurfacePartBuildError::NotANumber => {
                BevelBuildFailureKind::Construction
            }
        };
        BevelPartError::Failed(BevelBuildFailure::read_off(
            kind,
            format!("{error:?}"),
            &error,
        ))
    }
}

impl From<WorkError> for BevelPartError {
    fn from(error: WorkError) -> Self {
        match error {
            WorkError::Cancelled => BevelPartError::Cancelled,
            other => BevelPartError::Failed(BevelBuildFailure::read_off(
                BevelBuildFailureKind::WorkBudget,
                other.to_string(),
                &other,
            )),
        }
    }
}
