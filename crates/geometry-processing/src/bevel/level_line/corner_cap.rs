//! The hole round a corner vertex V after the strips are laid: k last rows (one per chain end at
//! V) alternating with k short rail pieces, possibly a single tie vertex each, closed about the
//! corner's AXIS - the normalised sum of the distinct face normals round V: a chamfer's by a fan, a
//! fillet's by rings. Ports the nested type of `Bevel/LevelLine/CornerCap.cs` (a C# static partial
//! class): a strip's end spoke ([`CornerCapLastRow`]). Its constructor and the caps' functions land
//! with later packages.

use vector_math::Vector3;

/// C# `CornerCap.LastRow`: a strip's end spoke as a fillet lays it - the arc's vertices, and the
/// circle it is drawn on, its radius the mean of the two legs.
///
/// The round's centre is where the sides' normals at the arc's two ends meet: on the bisector of
/// the two setbacks from the chain point. `p + q - c` is that only where the sides are square; at a
/// 45 degree knife tip it put the centre a setback out and read the radius as the setback, so no
/// test that asks the rows' radius (the ball, the mitre) could ever hold there.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CornerCapLastRow {
    /// The row's first end, on one side face.
    pub first: Vector3,
    /// The row's last end, on the other side face.
    pub last: Vector3,
    /// The unit normal of the plane through the row's two ends and its centre - its round's axis.
    pub plane_normal: Vector3,
    /// The chain's point the row's arc is drawn about - on the chain, beside the corner.
    pub chain_point: Vector3,
    /// True where the row ends a CONCAVE stretch: its ball rolls in the air, not the part.
    pub concave: bool,
    /// The arc's mesh vertices, first end to last.
    pub arc: Vec<i32>,
    /// The chain's unit direction at a SKEW row (its two ends cross the chain at different points),
    /// else zero. A skew row is not a plane arc, so its cross-tangent is the fillet's ruling - this -
    /// and not the normal of the plane through its ends and [`CornerCapLastRow::centre`].
    pub axis: Vector3,
    /// The centre of the circle the row is drawn on.
    pub centre: Vector3,
    /// The row's radius: the mean of its two legs from the centre.
    pub radius: f64,
}
