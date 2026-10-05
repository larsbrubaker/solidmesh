//! Dropping the zero-area slivers a boolean leaves over a T-junction in the input, and putting the
//! vertex each stitched into the face whose edge it stands on (`Bevel/BevelTVertexSlivers.cs`, a
//! C# `internal static class`, so a module here): one face's split ([`Split`]) and the repair's
//! faces with where each came from ([`Rebuild`]). The repair lands with the core's packages as
//! free functions in this module.

use polygon_mesh::FaceList;

/// C# `BevelTVertexSlivers.Split`: which vertices a face has to take in, and along which of its
/// own edges.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Split {
    /// The start of the face's edge being split, in the direction the face winds round it.
    pub x: i32,
    /// The end of that edge.
    pub y: i32,
    /// The vertices standing on it, in the order they will be met walking `x` to `y`.
    pub middles: Vec<i32>,
}

/// C# `BevelTVertexSlivers.Rebuild`: the faces of the repair, and where each of them came from
/// in the mesh it is replacing.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Rebuild {
    /// The repaired faces.
    pub faces: FaceList,
    /// For each of [`Rebuild::faces`], the index of the source face it replaces.
    pub source_face_index: Vec<i32>,
}
