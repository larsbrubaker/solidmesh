//! How the band gap's build leaves by a fault: every way out of
//! [`build_within`](super::band_gap_build_within::build_within) other than a result, gathered into
//! one [`BuildFault`] with the conversions from every callee's error. Split out of
//! `band_gap_build_within.rs` for size;
//! [`band_gap::build_unvalidated`](super::band_gap::build_unvalidated) turns each into the
//! caller's outcome.

use super::band_gap::BandGapBuildError;
use super::band_gap_reading::ReadRoomError;
use super::band_refinement::RefineError;
use super::band_room::RoomWithinError;
use super::band_side_tagging_ends::SideTaggingError;
use super::chain_field_from_polyline::ChainFieldArgumentError;
use super::level_line_mesh_edits::LevelLineMeshError;
use super::{band_cut, ChainEdgeNotOnTwoTriangles, WorkError};

/// How the build within leaves by a fault: the two the build turns into refusals (the work ran
/// out or was cancelled; a chain edge not on two triangles) and the caller's bugs it lets through.
#[derive(Debug, Clone)]
pub enum BuildFault {
    /// The work budget ran out, or the work was cancelled.
    Work(WorkError),
    /// A chain edge does not stand on exactly two triangles.
    ChainEdge(ChainEdgeNotOnTwoTriangles),
    /// Every other fault, passed to the caller as it comes.
    Error(BandGapBuildError),
}

impl From<WorkError> for BuildFault {
    fn from(error: WorkError) -> Self {
        BuildFault::Work(error)
    }
}

impl From<ChainEdgeNotOnTwoTriangles> for BuildFault {
    fn from(error: ChainEdgeNotOnTwoTriangles) -> Self {
        BuildFault::ChainEdge(error)
    }
}

impl From<SideTaggingError> for BuildFault {
    fn from(error: SideTaggingError) -> Self {
        match error {
            SideTaggingError::Work(work) => BuildFault::Work(work),
            SideTaggingError::ChainEdge(edge) => BuildFault::ChainEdge(edge),
        }
    }
}

impl From<LevelLineMeshError> for BuildFault {
    fn from(error: LevelLineMeshError) -> Self {
        match error {
            LevelLineMeshError::Work(work) => BuildFault::Work(work),
            other => BuildFault::Error(BandGapBuildError::Mesh(other)),
        }
    }
}

impl From<ChainFieldArgumentError> for BuildFault {
    fn from(error: ChainFieldArgumentError) -> Self {
        BuildFault::Error(BandGapBuildError::ChainField(error))
    }
}

impl From<RoomWithinError> for BuildFault {
    fn from(error: RoomWithinError) -> Self {
        match error {
            RoomWithinError::Work(work) => BuildFault::Work(work),
            RoomWithinError::ChainEdge(edge) => BuildFault::ChainEdge(edge),
            RoomWithinError::Read(read) => BuildFault::from(read),
        }
    }
}

impl From<ReadRoomError> for BuildFault {
    fn from(error: ReadRoomError) -> Self {
        match error {
            ReadRoomError::Arguments(bad) => BuildFault::Error(BandGapBuildError::Arguments(bad)),
            ReadRoomError::Cancelled => BuildFault::Work(WorkError::Cancelled),
            ReadRoomError::ChainField(bad) => BuildFault::from(bad),
            ReadRoomError::Mesh(bad) => BuildFault::from(bad),
        }
    }
}

impl From<RefineError> for BuildFault {
    fn from(error: RefineError) -> Self {
        match error {
            RefineError::Mesh(mesh) => BuildFault::from(mesh),
            RefineError::OutOfRange(message) | RefineError::NoParents(message) => {
                BuildFault::Error(BandGapBuildError::Defect(message))
            }
        }
    }
}

impl From<band_cut::BandCutError> for BuildFault {
    fn from(error: band_cut::BandCutError) -> Self {
        match error {
            band_cut::BandCutError::Mesh(mesh) => BuildFault::from(mesh),
            band_cut::BandCutError::RootEdgeGone(message)
            | band_cut::BandCutError::Straddles(message) => {
                BuildFault::Error(BandGapBuildError::Defect(message))
            }
        }
    }
}

/// A defect in the build (a broken invariant), named by its message.
pub fn defect(message: impl std::fmt::Display) -> BuildFault {
    BuildFault::Error(BandGapBuildError::Defect(message.to_string()))
}
