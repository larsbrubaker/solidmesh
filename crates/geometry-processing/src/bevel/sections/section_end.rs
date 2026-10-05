//! The section at the END of an open run (`Bevel/Sections/SectionEnd.cs`, a C# `internal static
//! class`, so a module here): what is read off the part at one such end ([`EndReading`]). The
//! reading and the section built from it land with the core's section packages as free functions
//! in this module.

use vector_math::Vector3;

/// C# `SectionEnd.EndReading`: what `TryRead` reads off the part at one end of an open run.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EndReading {
    /// The chain's end vertex.
    pub vertex: i32,
    /// Where that vertex stands.
    pub station: Vector3,
    /// The unit direction the chain runs INTO the end - the axis the profile slides along. At the
    /// run's start that is the leaving edge read backwards.
    pub along: Vector3,
    /// The unit normal of the face on the left of the walk.
    pub left: Vector3,
    /// The unit normal of the face on the right of the walk.
    pub right: Vector3,
    /// The unit normal of the transverse face the run dies on.
    pub transverse: Vector3,
    /// `along . transverse`: how steeply the chain runs into the transverse face.
    pub lean: f64,
    /// `left . right`: the cosine between the two side normals.
    pub cosine: f64,
    /// The crease the left boundary rides: the mesh edge the transverse face shares with the left
    /// side.
    pub left_slide: i32,
    /// The crease the right boundary rides.
    pub right_slide: i32,
}
