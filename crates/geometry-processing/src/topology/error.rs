//! The errors the topology module reports: an index that does not belong to the mesh or set it was
//! given, a non-finite pick, an empty tolerance ladder, and the like. Each carries the message the
//! caller shows.

use std::fmt;

/// Why a topology call refused its input.
#[derive(Debug, Clone, PartialEq)]
pub enum TopologyError {
    /// A face index that is not a face of the mesh.
    FaceIndexOutOfRange {
        /// The name of the parameter that was out of range.
        parameter: &'static str,
        /// The index given.
        index: i32,
        /// How many faces the mesh has.
        face_count: usize,
    },
    /// A seed edge that is not an index into the feature edge set.
    EdgeIndexOutOfRange {
        /// The index given.
        index: i32,
        /// How many edges the set has.
        edge_count: usize,
    },
    /// A non-finite argument, with its message.
    NotFinite {
        /// The parameter's name.
        parameter: &'static str,
        /// The message.
        message: String,
    },
    /// An edge that bounds none of a face group's faces (from
    /// `bevel_edge_group_resolver::create_from_face_group`), with its message.
    EdgeDoesNotBoundFaceGroup {
        /// The message.
        message: String,
    },
    /// A tolerance ladder with no rungs (from `bevel_edge_readings::for_group`), so there is no
    /// angle to read a group at.
    EmptyLadder,
    /// An index that is not into the list of readings it was given (from
    /// `bevel_edge_readings::label`).
    ReadingIndexOutOfRange {
        /// The index given.
        index: i32,
        /// How many readings the list has.
        reading_count: usize,
    },
    /// A corner index that is not into the path corner set (from
    /// `bevel_corner_group_resolver::create_from_corner`).
    CornerIndexOutOfRange {
        /// The index given.
        index: i32,
        /// How many corners the set has.
        corner_count: usize,
    },
}

impl fmt::Display for TopologyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TopologyError::FaceIndexOutOfRange {
                parameter,
                face_count,
                ..
            } if *parameter == "hitFaceIndex" => write!(
                f,
                "The mesh has {face_count} faces, so this hit did not come from it."
            ),
            TopologyError::FaceIndexOutOfRange { parameter, .. } => write!(
                f,
                "Specified argument was out of the range of valid values. (Parameter '{parameter}')"
            ),
            TopologyError::EdgeIndexOutOfRange { index, edge_count } => write!(
                f,
                "Edge {index} is not an index into a feature edge set of {edge_count} edges."
            ),
            TopologyError::NotFinite { message, .. } => write!(f, "{message}"),
            TopologyError::EdgeDoesNotBoundFaceGroup { message } => {
                write!(f, "{message} (Parameter 'boundaryEdge')")
            }
            TopologyError::EmptyLadder => write!(
                f,
                "The tolerance ladder has no rungs, so there is no angle to read this group at. (Parameter 'ladder')"
            ),
            TopologyError::ReadingIndexOutOfRange { .. } => write!(
                f,
                "Specified argument was out of the range of valid values. (Parameter 'index')"
            ),
            TopologyError::CornerIndexOutOfRange { .. } => write!(
                f,
                "Specified argument was out of the range of valid values. (Parameter 'cornerIndex')"
            ),
        }
    }
}

impl std::error::Error for TopologyError {}
