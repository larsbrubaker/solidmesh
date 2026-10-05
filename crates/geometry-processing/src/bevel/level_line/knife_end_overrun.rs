//! The corner strip's end at a bounded free end whose END PLANE E is the knife neighbour's own
//! plane: one rail reaches E inside the chain while the other runs on past the end along the end
//! ray to E, and the round past the first rail's end is laid as rows clipped by E, so the end hole
//! left is a flat piece of E. Ports the nested type of `Bevel/LevelLine/KnifeEndOverrun.cs` (a C#
//! static class): the end read ([`KnifeEndOverrunEnd`]). Its reading lands with a later package.

use vector_math::Vector3;

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
