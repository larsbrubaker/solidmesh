//! A measurement only: every vertex the cut ADDS to a `LevelLineMesh`, and whether it lands on top
//! of one already there. Ports the nested types of `Bevel/LevelLine/SplitMeter.cs` (a C# static
//! class): what added a vertex ([`SplitMeterSplitKind`]) and the record kept of it
//! ([`SplitMeterSplitRecord`]). The meter's state, hooks and the record's description land with
//! later packages.

use vector_math::Vector3;

use super::{LevelLineRule, PointPlace};

/// C# `SplitMeter.SplitKind`: what a vertex was added BY - the three ways a position enters the
/// mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SplitMeterSplitKind {
    /// An edge split: every triangle on the edge was divided with it.
    #[default]
    OnEdge = 0,

    /// A point put in a triangle's interior; its three edges were left alone.
    Interior,

    /// A free-standing vertex the fill laid in, on no triangle yet.
    Free,
}

/// C# `SplitMeter.SplitRecord`: one added vertex, kept so a later coincidence can say what the
/// vertex it landed on was.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SplitMeterSplitRecord {
    /// The added vertex.
    pub vertex: i32,
    /// Where it was added.
    pub point: Vector3,
    /// How it was added.
    pub kind: SplitMeterSplitKind,
    /// Whichever refinement rule's search was running when the split was asked for.
    pub rule: LevelLineRule,
    /// The context triangle.
    pub triangle: i32,
    /// The context triangle's first corner.
    pub corner_a: i32,
    /// The context triangle's second corner.
    pub corner_b: i32,
    /// The context triangle's third corner.
    pub corner_c: i32,
    /// The edge that was split, for [`SplitMeterSplitKind::OnEdge`]: its first end; -1 otherwise.
    pub edge_from: i32,
    /// The split edge's second end; -1 otherwise.
    pub edge_to: i32,
    /// How many triangles stood on the split edge - ONE means the split could not be conforming.
    pub triangles_on_the_edge: i32,
    /// How far the point stood off the context triangle's edge AB.
    pub distance_ab: f64,
    /// How far the point stood off the context triangle's edge BC.
    pub distance_bc: f64,
    /// How far the point stood off the context triangle's edge CA.
    pub distance_ca: f64,
    /// The length `RefinementSplits.WhereThePointStands` compared those three against when the
    /// field rules ask it - a millionth of the context triangle.
    pub edge_tolerance: f64,
    /// That routine's own three-way verdict on the context triangle. It is recorded rather than
    /// derived, because a point at a CORNER and a point in the INTERIOR are two different findings
    /// and the on-an-edge question answers no to both of them.
    pub place: PointPlace,
    /// The corner, or the named edge's first end; -1 in the interior.
    pub judged_from: i32,
    /// The same corner again, or the named edge's second end; -1 in the interior.
    pub judged_to: i32,
}
