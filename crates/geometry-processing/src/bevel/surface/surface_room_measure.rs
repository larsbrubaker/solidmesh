//! Pacing out the room a cut has on its region (`Bevel/Surface/SurfaceRoomMeasure.cs`, a static
//! class: free functions here). The rulers and the exact offset of a rim segment in front of the
//! chain edge land here first, with the bands a pass takes off a region; the measure itself
//! (`RoomAcross`, `MeasureCorner`) lands with later packages.

use std::collections::{HashMap, HashSet};
use std::sync::LazyLock;

use vector_math::Vector3;

use super::SurfaceBevelPlan;
use crate::topology::FaceGroup;

/// How near a rim edge has to come to the chain edge's own line before it is read as being that
/// line rather than in front of it, and how much of the span two segments must share before they
/// are facing each other at all. Relative to the chain edge's length, with a floor of one unit so
/// that a tiny edge on a large part is not measured against a tolerance finer than its own
/// arithmetic.
pub const RELATIVE_TOLERANCE: f64 = 1e-7;

/// How near two points have to be to be the same corner of the rim. Looser than
/// [`RELATIVE_TOLERANCE`], and relative to the chain edge for the same reason, because a mesh
/// keeps its vertices as floats: two readings of one corner differ in the last bit or two of a
/// float, which at 20 mm is already coarser than a tenth of a millionth.
pub const VERTEX_TOLERANCE: f64 = 1e-5;

/// What a region no band of this pass lands on has, so that the reading needs no null case: one
/// shared empty set, as C#'s static instance is.
static NO_BANDS: LazyLock<HashSet<i32>> = LazyLock::new(HashSet::new);

/// Which edges the pass is beveling take a band off `region`, or an empty set where none does.
/// Those are the partners a band on that region negotiates its share of the face with rather than
/// the walls it stops at. Looked up only, so the `HashMap` order cannot reach an output.
pub fn bands_on_the_region<'a>(
    bands_on_region: &'a HashMap<i32, HashSet<i32>>,
    region: &FaceGroup,
) -> &'a HashSet<i32> {
    bands_on_region
        .get(&SurfaceBevelPlan::key_of(region))
        .unwrap_or(&NO_BANDS)
}

/// The smallest offset along `inward` at which the segment `a`..`b` stands in front of the chain
/// edge's own span (from `from`, `length` along unit `along`), or infinity where it does not
/// stand in front of it at all.
///
/// Exact rather than sampled. Both segments are read in the chain edge's (along, inward) frame;
/// the other segment is clipped to the span the chain edge occupies along that frame's first axis,
/// and the offset is linear along it, so its smallest value over the clipped part is at one of the
/// two clipped ends. Two answers are deliberately not offsets: a segment sharing no more than a
/// corner's width of the span, and a segment lying ON the edge's line, which are the chain edge
/// itself and anything collinear with it.
pub fn offset_to_segment(
    from: Vector3,
    along: Vector3,
    length: f64,
    inward: Vector3,
    a: Vector3,
    b: Vector3,
) -> f64 {
    let tolerance = f64::max(length, 1.0) * RELATIVE_TOLERANCE;

    // How much of the span the two have to SHARE before they face each other at all is a CORNER's
    // width and not a rounding: a rim that leaves the chain edge's own corner is written in
    // floats, and the stretch of it below the corner overlaps the span by a millionth or so
    // without standing in front of anything. Read at the finer ruler that overlap is a wall the
    // width of a corner, which on the picture hanger's gusset says a 20 mm face is 0.4 mm wide.
    let sharing = f64::max(length, 1.0) * VERTEX_TOLERANCE;

    let along_a = (a - from).dot(along);
    let along_b = (b - from).dot(along);
    let offset_a = (a - from).dot(inward);
    let offset_b = (b - from).dot(inward);

    // C#'s Math.Max/Math.Min: NaN in either argument gives NaN.
    let low = csharp_max(csharp_min(along_a, along_b), 0.0);
    let high = csharp_min(csharp_max(along_a, along_b), length);

    let shared = high - low;
    if shared <= sharing || shared.is_nan() {
        return f64::INFINITY;
    }

    let nearest = csharp_min(
        offset_at(along_a, offset_a, along_b, offset_b, low),
        offset_at(along_a, offset_a, along_b, offset_b, high),
    );

    if nearest > tolerance {
        nearest
    } else {
        f64::INFINITY
    }
}

/// The offset of the segment at one place along the frame's first axis, interpolated.
pub fn offset_at(along_a: f64, offset_a: f64, along_b: f64, offset_b: f64, at: f64) -> f64 {
    let span = along_b - along_a;

    if span == 0.0 {
        return csharp_min(offset_a, offset_b);
    }

    offset_a + ((offset_b - offset_a) * ((at - along_a) / span))
}

/// C#'s `Math.Min`, which returns NaN when either argument is NaN (Rust's `f64::min` returns the
/// other one).
fn csharp_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}

/// C#'s `Math.Max`, NaN-propagating like [`csharp_min`].
fn csharp_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}
