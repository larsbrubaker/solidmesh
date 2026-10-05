//! Sewing a cut's new vertices into the faces whose sides they stand on. Ports
//! `Bevel/Surface/SurfaceSplitSewing.cs` (C# `internal static class`; a module of free functions
//! here). The sewing itself (`SewInTheSplits`) lands with a later package; what is here
//! is the one reading of "on this side" that the whole build shares ([`stands_on_the_side`]),
//! which the fan ([`super::surface_fan`]) asks of its diagonals too, and the two per-face readings
//! the sewing is built from ([`ring_with_the_splits_in`], [`the_neighbours_carry_its_middle`]), and
//! the lobes a ring that revisits a vertex falls into ([`lobes`]).

use vector_math::Vector3;

use super::{surface_fan, surface_ring, SurfaceSplitsOnASegment, SurfaceVertexPool};

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

/// C# `TheNeighboursCarryItsMiddle`: whether the faces either side of a lobe that covers nothing
/// already carry its middle corners - which is what decides whether dropping the lobe closes a
/// seam or opens one.
///
/// A zero-area lobe [a, b, c] with b on the line from a to c emits a-b, b-c and c-a. Dropping it
/// pairs its neighbours only if the face across the long side a-c is split at b as well, so that
/// it emits b-a and c-b rather than the one long c-a. That is exactly the stretch's own reading:
/// the neighbour's side from a to c is read by the same key in the same pass
/// ([`SurfaceSplitsOnASegment`]), so if the reading names b, the neighbour carries b.
///
/// Measured three ways. On the lobe wall's rim at 0.1 the lobe [189, 482, 475] has its far pair
/// 0.0557 apart and the reading of that stretch returns [475] - the middle corner named, the
/// neighbours split at it, and dropping the lobe takes the third face off the edge 189-475. At 0.02
/// the same. But on the Round Side at radius 6 the needle [408, 415, 598] is 24.2 mm long and five
/// hundredths of a thousandth wide, and the reading of its long side returns NOTHING: no neighbour
/// is split at 415, and dropping it left three edges open. So it is emitted - a triangle with no
/// area but with its edges answered is sound, and a hole is not.
///
/// The corners are checked for BEING named rather than for their order: every corner of such a
/// lobe stands on one line, and the reading is sorted along that line, so a corner it names is
/// named in its place.
pub fn the_neighbours_carry_its_middle(
    pool: &SurfaceVertexPool,
    standing_on: &mut SurfaceSplitsOnASegment<'_>,
    lobe: &[i32],
) -> bool {
    // The SAME pair CoversNothing measured the lobe's flatness against (SurfaceFan.FarthestApart),
    // so the stretch asked about here is the line the lobe was found to lie along.
    let (one, two, _) = surface_fan::farthest_apart(lobe, pool);

    let named = standing_on.between(lobe[one], lobe[two]);

    for (corner, &vertex) in lobe.iter().enumerate() {
        if corner == one || corner == two {
            continue;
        }

        if !named.contains(&vertex) {
            return false;
        }
    }

    true
}

/// C# `RingWithTheSplitsIn`: one face as a ring with every split standing part way along one of
/// its sides inserted, each in its place along that side. (C#'s unused `pool` parameter is
/// dropped; the reading carries the pool it measures with.)
///
/// A side is read as a STRETCH between its two corners ([`SurfaceSplitsOnASegment`]), so the
/// answer is the same one the face across that stretch gets. A split that is already a corner of
/// this ring is left out: put in as well it would make the ring visit one vertex twice, which is
/// the pinch `Lobes` is left to take apart.
pub fn ring_with_the_splits_in(
    corners: &[i32; 3],
    standing_on: &mut SurfaceSplitsOnASegment<'_>,
) -> Vec<i32> {
    let mut ring = Vec::with_capacity(3);

    for corner in 0..corners.len() {
        ring.push(corners[corner]);

        for on in standing_on.between(corners[corner], corners[(corner + 1) % corners.len()]) {
            if on == corners[0] || on == corners[1] || on == corners[2] {
                continue;
            }

            ring.push(on);
        }
    }

    surface_ring::without_repeats(&ring)
}

/// The LOBES of a ring that comes back to a vertex it has already visited: each return closes
/// what was walked since as a lobe of its own (three corners or more), the walk carrying on from
/// the vertex it came back to; what is left is the last lobe.
pub fn lobes(ring: &[i32]) -> Vec<Vec<i32>> {
    let mut lobes = Vec::new();
    let mut walked: Vec<i32> = Vec::with_capacity(ring.len());

    for &vertex in ring {
        let Some(already) = walked.iter().position(|&v| v == vertex) else {
            walked.push(vertex);
            continue;
        };

        // Back where it has been: everything since then is a lobe of its own.
        let lobe = walked[already..].to_vec();

        if lobe.len() >= 3 {
            lobes.push(lobe);
        }

        walked.truncate(already + 1);
    }

    if walked.len() >= 3 {
        lobes.push(walked);
    }

    lobes
}
