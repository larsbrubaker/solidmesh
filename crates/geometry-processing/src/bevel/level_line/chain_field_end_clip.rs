//! One bounded end's clipping plane ([`ChainFieldEndClip`]), from
//! `Bevel/LevelLine/ChainField.EndClip.cs`. The `ChainField` methods of that partial - the clipped
//! readings and the end-plane reader - land with their method package.

use vector_math::Vector3;

/// C# `ChainField.EndClip`: one bounded end's plane - through its vertex, unit normal faced on past
/// the end.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChainFieldEndClip {
    /// The end vertex the plane passes through.
    pub vertex: Vector3,
    /// The plane's unit normal, faced on past the end.
    pub normal: Vector3,
    /// The chain parameter at the end vertex.
    pub parameter: f64,
}
