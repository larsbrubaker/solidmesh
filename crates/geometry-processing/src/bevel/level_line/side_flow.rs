//! Whether the band flows from a side onto a gentle neighbour. Ports
//! `Bevel/LevelLine/SideFlow.cs` (a C# static class): so far its constant; the flow test lands with
//! later packages.

/// How far under a right angle (as the cosine between the unit normals) a convex crease must turn
/// to be gentle: an exact right angle reads its normals' dot as +-1e-16 and must stay a wall (a 2 mm
/// wall under a fillet of 2 is consumed to its rim, not flowed over), as in the foreign-edge
/// room's knife test.
pub const RIGHT_ANGLE_NOISE: f64 = 1e-9;
