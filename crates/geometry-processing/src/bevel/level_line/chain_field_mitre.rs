//! A segment's line carried past a sharp vertex ([`ChainFieldExtension`]), from
//! `Bevel/LevelLine/ChainField.Mitre.cs`. Finding the mitres, the ray's reading and the other
//! `ChainField` methods of that partial land with their method package.

use vector_math::Vector3;

/// C# `ChainField.Extension`: one segment's line carried past a sharp vertex (a mitre ray), or past
/// an open chain's end (an end ray).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChainFieldExtension {
    /// The vertex the ray starts from.
    pub origin: Vector3,
    /// Unit direction the ray runs from the vertex.
    pub direction: Vector3,
    /// The ray is read only where `(q - origin) . keep >= 0` - its side of the bisector.
    pub keep: Vector3,
    /// How far the ray runs from its origin.
    pub reach: f64,
    /// The setback the ray carries (the vertex's).
    pub setback: f64,
    /// The chain parameter at the ray's origin.
    pub parameter: f64,
    /// How the parameter runs along the ray: 0 on a mitre ray (the whole wedge reads the vertex),
    /// +1 past a chain's last point and -1 before its first on an end ray, so the parameter goes on
    /// past the end.
    pub parameter_slope: f64,
}
