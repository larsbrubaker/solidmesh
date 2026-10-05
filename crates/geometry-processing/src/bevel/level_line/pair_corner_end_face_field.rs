//! The part of a picked edge's round that runs out through its END FACE at a mitred pair corner
//! ([`PairCornerEndFaceField`]), and the scope that turns it off for a flow
//! ([`PairCornerEndFaceFieldScope`]). Ports the types of
//! `Bevel/LevelLine/PairCornerEndFaceField.cs`; the finder, the field queries and the per-flow gate
//! land with later packages.

use vector_math::Vector3;

/// C# `PairCornerEndFaceField`: the part of a picked edge's round that runs out through its END
/// FACE at a mitred pair corner - negative in the prism the round's cylinder leaves in front of it
/// (between the two tangent planes through the round's axis, outside the cylinder), so a group
/// holding that end face reads the least of it and its pick field (`BandGroupFields`) and cuts the
/// end face along the round.
///
/// **Where it applies** (`Find`): two members meeting at a corner vertex with a turn between them,
/// sharing exactly ONE face, whose other sides are two different faces joined through the corner
/// by an unselected convex crease - Bevel to Center Feature's right tip (x = 20 face shared, slope
/// and top unshared). The shared face is mitred as it always was; nothing cut the unshared END face
/// with the member's round, so the top face kept a wedge in front of the tip's round
/// (`BevelToCenterPairCornerTests`).
///
/// **The field.** `max(size - r, -(q . m1), -(q . m2), bounds)`, where q is the point's offset
/// from the round's axis across the edge, r its length, m1, m2 the directions from the axis to the
/// two tangent lines, and bounds four linear pieces that end it at the corner's end region.
/// Continuous, and zero on the cylinder inside the prism. Not convex (size - r is concave), so the
/// least and greatest over a piece are not golden-section searched as a run-out's are: they are
/// read exactly at the piece's corners, where pieces tie, and where it comes nearest the axis.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PairCornerEndFaceField {
    /// A point on the round's axis.
    pub axis_point: Vector3,
    /// The round's unit axis direction.
    pub axis_direction: Vector3,
    /// The round's radius.
    pub size: f64,
    /// The linear pieces, each `gradient . p + constant`: the two tangent planes, then the bounds
    /// of the corner's end region.
    pub planes: Vec<(Vector3, f64)>,
}

/// C# `PairCornerEndFaceField.Scope`: turns the end-face field off for this flow until it is
/// dropped, which puts back what was there before.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PairCornerEndFaceFieldScope {
    /// Whether the field was off when the scope was opened - restored when it closes.
    pub was: bool,
}
