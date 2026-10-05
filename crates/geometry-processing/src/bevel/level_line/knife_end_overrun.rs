//! The corner strip's end at a bounded free end whose END PLANE E is the knife neighbour's own
//! plane: one rail reaches E inside the chain while the other runs on past the end along the end
//! ray to E, and the round past the first rail's end is laid as rows clipped by E, so the end hole
//! left is a flat piece of E. Ports the nested type of `Bevel/LevelLine/KnifeEndOverrun.cs` (a C#
//! static class): the end read ([`KnifeEndOverrunEnd`]) and the on-the-plane test
//! ([`on_the_plane`]). Its reading lands with a later package.

use vector_math::{cs_math, Vector3};

/// C# `KnifeEndOverrun.End`: the end read - which end, its plane, and the direction square to the
/// chain, in the stopped rail's face, the rows are drawn toward. Its init-only properties are
/// public fields here.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct KnifeEndOverrunEnd {
    /// True at the chain's last end, false at its first.
    pub last: bool,
    /// Where the end vertex stands.
    pub vertex: Vector3,
    /// The unit normal of the end plane E.
    pub normal: Vector3,
    /// The direction square to the chain, in the stopped rail's face, the rows are drawn toward.
    pub across: Vector3,
}

/// True when `p` stands on the plane through `vertex` of normal `normal`, to a millionth of its
/// distance from the vertex (at least 1).
pub fn on_the_plane(p: Vector3, vertex: Vector3, normal: Vector3) -> bool {
    (p - vertex).dot(normal).abs() <= 1e-6 * cs_math::max_f64(1.0, (p - vertex).length())
}
