//! Sewing a cut's new vertices into the faces whose sides they stand on. Ports
//! `Bevel/Surface/SurfaceSplitSewing.cs` (C# `internal static class`; a module of free functions
//! here). The sewing itself (`SewInTheSplits`) lands with a later package; what is here is the one
//! reading of "on this side" that the whole build shares ([`stands_on_the_side`]), which the fan
//! ([`super::surface_fan`]) asks of its diagonals too.

use vector_math::Vector3;

use super::SurfaceVertexPool;

/// C# `StandsOnTheSide`: whether `point` stands part way along the side from `from` to `to` -
/// strictly between its two ends, and within the pool's ruler of the line through them - and
/// where along it if so (C#'s `out double where`, which is written - zero where the side has no
/// length - whatever the answer).
///
/// The one reading of "on this side" the build has: the sewing asks it of a face's edge, and the
/// seam re-read asks it of a strip's tangent run (`SurfaceSeamReRead`). They have to agree,
/// because a vertex sewn into the face and not into the seam across from it is a T junction.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn stands_on_the_side(
    pool: &SurfaceVertexPool,
    from: Vector3,
    to: Vector3,
    point: Vector3,
) -> (bool, f64) {
    let run = to - from;

    if !(run.length_squared() > 0.0) {
        return (false, 0.0);
    }

    let at = point - from;

    let where_ = at.dot(run) / run.length_squared();

    (
        where_ > 0.0 && where_ < 1.0 && (at - (run * where_)).length() < pool.near_enough(),
        where_,
    )
}
