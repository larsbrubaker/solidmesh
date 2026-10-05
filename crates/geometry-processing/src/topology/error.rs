//! The errors the topology module reports where C# throws: `ArgumentOutOfRangeException` for an
//! index that does not belong to the mesh or set it was given, and `ArgumentException` for a
//! non-finite pick. Each keeps the C# message text.

use std::fmt;

/// Why a topology call refused its input.
#[derive(Debug, Clone, PartialEq)]
pub enum TopologyError {
    /// A face index that is not a face of the mesh (C# `ArgumentOutOfRangeException`).
    FaceIndexOutOfRange {
        /// The parameter that was out of range, by its C# name.
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
    /// A non-finite argument (C# `ArgumentException`), with C#'s message.
    NotFinite {
        /// The parameter, by its C# name.
        parameter: &'static str,
        /// C#'s message.
        message: String,
    },
    /// An edge that bounds none of a face group's faces (C# `ArgumentException` from
    /// `BevelEdgeGroupResolver.CreateFromFaceGroup`), with C#'s message.
    EdgeDoesNotBoundFaceGroup {
        /// C#'s message.
        message: String,
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
        }
    }
}

impl std::error::Error for TopologyError {}
