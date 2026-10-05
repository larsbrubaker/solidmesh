//! A picked point's field for the kernel's level-line cut ([`PointField`]). Ports the type of
//! `Bevel/LevelLine/PointField.cs`; its constructors and field queries land with a later package.

use vector_math::Vector3;

/// C# `PointField`: a picked point's field for the kernel's level-line cut - negative on the cap
/// the bevel removes, zero on the ring where it starts.
///
/// A fillet's is the straight-line distance to the point less the setback (a sphere - its ring is
/// the ball's tangent ring on a cone); a chamfer's is the depth below the point along an inward
/// normal less the plane's depth (the cut plane). Both are true distances, so 1-Lipschitz as
/// `BandRefinement` requires, and both answer their extremes over a piece exactly (the sphere's
/// least is the distance to the piece, its greatest a corner, as it is convex; the plane's are
/// corners), so the refinement takes the asking path and splits only where the field names - a ring
/// tangent to an edge at the room included. It plays `PickField`'s role for a pick with no chains.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PointField {
    /// The picked point.
    pub point: Vector3,
    /// The unit vector into the part for a plane; zero for a sphere.
    pub inward: Vector3,
    /// The setback (sphere) or depth (plane) at which the field is zero.
    pub level: f64,
}
