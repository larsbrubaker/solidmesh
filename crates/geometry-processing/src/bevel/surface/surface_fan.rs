//! Turning a ring of vertices into triangles, and turning three vertices into a triangle that
//! faces a given way. Ports `Bevel/Surface/SurfaceFan.cs` (C# `internal static class`; a module of
//! free functions here). What phase 3 needs of a triangulator, and no more: every ring it is
//! handed is a convex facet, or a convex facet cut by half spaces, with points added along its
//! edges. [`super::surface_ring_ears`] covers the rings no single apex can see whole.
//!
//! WHICH vertex the fan runs from is the whole difficulty. A ring carries points that were clipped
//! into its edges, and those are collinear with the corners either side of them: fan from a corner
//! of such an edge and the triangle over that point has no area, is dropped as nothing, and the
//! point ends up in no triangle at all - which is exactly the T junction the clip was careful to
//! make a shared vertex. So the apex is a vertex that sees every triangle of its own fan with area
//! in it.
//!
//! The walkback fans its cut facets from here too: a fan that drops a point the other keeps is a T
//! junction on a seam both of them sew, so there is one copy of the rule.
//!
//! C#'s overloads get their own names: `Emit(ring, ..)` is [`emit`], `EmitFacing(ring, ..)` is
//! [`emit_facing`], the private `Emit(ring, .., Vector3? facing)` is `emit_with`, and the
//! three-vertex `EmitFacing(pool, faces, first, second, third, reference)` is
//! [`emit_triangle_facing`].

use vector_math::Vector3;

use super::surface_split_sewing;
use super::SurfaceVertexPool;

/// Twice the area below which a triangle is dropped as covering nothing - three points of one
/// straight rim, which a fan over a clipped facet makes by construction.
pub const SMALLEST_TRIANGLE: f64 = 1e-9;

/// How far from square to the reference a ring's own normal has to read before that reading is
/// worth following: a hundredth of a right angle, which a ring with any face at all to it clears
/// easily and a ring standing edge-on to the reference cannot.
const SQUARE_TO_THE_REFERENCE: f64 = 0.01;

/// How much of the two facings' product a triangle has to agree by before its agreement is
/// evidence of anything: a RELATIVE floor, because both sides of the comparison are areas and so
/// scale as the square of the part's size. An absolute one would call every triangle of a
/// tenth-millimetre ring folded and none of a metre-wide one.
const AGREES_WITH_THE_RING: f64 = 1e-9;

/// C# `Emit(ring, pool, faces)`: triangles for `ring`, in its own order, appended to `faces`.
/// Takes the pool mutably because a ring no vertex of which can fan it is fanned from a new
/// middle vertex (`fan_from_the_middle`).
pub fn emit(ring: &[i32], pool: &mut SurfaceVertexPool, faces: &mut Vec<(i32, i32, i32)>) {
    emit_with(ring, pool, faces, None);
}

/// C# `EmitFacing(ring, pool, faces, facing)`: the same, with every triangle wound to face
/// `facing` rather than to follow the ring. What the strip needs: its rings are built rail by rail
/// from lines, with no winding of their own to follow, and every triangle of one lies in the same
/// plane.
pub fn emit_facing(
    ring: &[i32],
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    facing: Vector3,
) {
    emit_with(ring, pool, faces, Some(facing));
}

/// C# `EmitCoveringNothing`: a ring the CALLER knows covers nothing, emitted as needles whose
/// edges are exactly the ring's - see `fan_from_a_farthest_corner` for why that closes rather than
/// opens a seam.
///
/// Asked for by a caller that knows the ring lies on one line from the way it was built rather
/// than from measuring it: a strip quad whose two rails BOTH stood still is the chord and whatever
/// stands along it, however far the points making it up happen to drift off that line in floats
/// (`SurfaceBevelStitch`). [`emit`] never gets as far as asking [`covers_nothing`] about such a
/// ring: `apex_to_fan_from` finds it an apex first, because what stands between a triangle and
/// nothing there is [`SMALLEST_TRIANGLE`], an ABSOLUTE floor on twice the area - and a needle a
/// ten-millionth of a millimetre wide along a chord half a millimetre long has 6.7e-8 of it,
/// seventy times that floor. So it is wound by a normal that is numerical noise, and only the
/// caller knows better.
pub fn emit_covering_nothing(
    ring: &[i32],
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
) {
    if ring.len() < 3 {
        return;
    }

    if ring.len() == 4 && (ring[1] == ring[3] || ring[0] == ring[2]) {
        // Four corners visiting only three vertices, the repeat standing OPPOSITE itself: the ring
        // walks out along the line and back over the same two sides. Both of its triangles are then
        // either the same three vertices laid down twice or three with no width between them,
        // depending which corner the fan runs from, and skipping them is right either way - the two
        // sides that repeat already pair with each other.
        return;
    }

    fan_from_a_farthest_corner(ring, pool, faces);
}

/// C#'s private `Emit(ring, pool, faces, Vector3? facing)`, which both public emits forward to.
fn emit_with(
    ring: &[i32],
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    facing: Option<Vector3>,
) {
    if ring.len() < 3 {
        return;
    }

    let apex = apex_to_fan_from(ring, pool);

    if apex < 0 {
        if covers_nothing(ring, pool) {
            // A ring the part cannot tell off one line has no middle worth minting, and minting one
            // is a fold: the neighbours across its sides know nothing of the new point. It is emitted
            // as it stands instead.
            fan_from_a_farthest_corner(ring, pool, faces);
            return;
        }

        // Nothing in the ring can see all of it without a flat triangle - which a convex ring should
        // not manage, but a sliver left by a cut through a corner can. Its own middle can, always.
        fan_from_the_middle(ring, pool, faces, facing);
        return;
    }

    let apex = apex as usize;

    // ONE ring is ONE piece of surface, so which way round it goes is read ONCE - from the ring's own
    // Newell normal against the reference - and every triangle of the fan then follows the ring.
    // Read triangle by triangle instead, a fan over a ring that is not flat can have one of its
    // triangles standing nearly SQUARE to the reference and be wound by the sign of a dot product
    // that is numerical noise: at the raked bore's edge 30 the last column pinches onto one wall
    // vertex the two stations share, and its middle triangle - three points of the mitre plane, its
    // normal along x where the ring's is along y - came back flipped, leaving four of the ring's own
    // edges carrying two faces walking them the same way. Where the ring itself reads square to the
    // reference there is nothing to follow and each triangle is asked on its own, as before.
    let ring_wound = match facing {
        Some(facing)
            if newell_normal_of(ring, pool)
                .get_normal()
                .dot(facing.get_normal())
                .abs()
                > SQUARE_TO_THE_REFERENCE =>
        {
            Some(newell_normal_of(ring, pool).dot(facing) >= 0.0)
        }
        _ => None,
    };

    let count = ring.len();
    let mut i = 1;
    while i + 1 < count {
        let first = ring[apex];
        let second = ring[(apex + i) % count];
        let third = ring[(apex + i + 1) % count];

        if let Some(wound) = ring_wound {
            if is_worth_keeping(pool, first, second, third) {
                faces.push(if wound {
                    (first, second, third)
                } else {
                    (first, third, second)
                });
            }
        } else if let Some(facing) = facing {
            emit_triangle_facing(pool, faces, first, second, third, facing);
        } else if is_worth_keeping(pool, first, second, third) {
            faces.push((first, second, third));
        }

        i += 1;
    }
}

/// C# `CoversNothing`: whether a ring covers nothing - every corner of it standing within the
/// ruler of the line through its two farthest apart, which is three points of one straight rim
/// however many of them there are.
///
/// Nothing is emitted for such a ring, and nothing is lost by that. Its corners are all corners of
/// the faces around it, and the edges those faces walk along it PAIR WITH EACH OTHER across it -
/// measured on the lobe wall's rim at radius 0.1, where the sliver (189, 482, 475) lay between
/// faces already walking 189-475 and 475-189, 475-482 and 482-475, 482-564 and 564-482. Emitting a
/// triangle there is what puts a third face on one of those edges.
///
/// Read against the pool's OWN ruler rather than an area: a sliver five ten-thousandths long and
/// one ten-thousandth wide has plenty of area for any absolute floor, and its thinness - an
/// altitude over a longest side - is a fifth, which is no needle at all. What makes it nothing is
/// that the part cannot tell its corners off one line, which is exactly what the ruler measures.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn covers_nothing(ring: &[i32], pool: &SurfaceVertexPool) -> bool {
    let (from, to, apart) = farthest_apart(ring, pool);

    if !(apart > 0.0) {
        return true;
    }

    let at = pool.position_of(ring[from]);
    let run = (pool.position_of(ring[to]) - at) / apart;

    for &corner in ring {
        let along = pool.position_of(corner) - at;

        if (along - (run * along.dot(run))).length() >= pool.near_enough() {
            return false;
        }
    }

    true
}

/// C# `FanFromAFarthestCorner`: a ring that covers nothing, fanned from one of its two farthest
/// apart corners - zero-area triangles, every side of the ring used exactly once, and no vertex
/// minted.
///
/// Emitted whatever its area, because its EDGES are what close the shell - the smallest triangle
/// is a reading about a triangle worth keeping for its own sake, and these are kept for the
/// neighbours' sake. Fanned from a farthest corner rather than any corner so that the fan's
/// diagonals lie along the ring's own line rather than doubling back over it; the diagonals cancel
/// in pairs either way, which is what makes the neighbours across each side pair with these
/// exactly as they would have with the ring as one face.
///
/// Where it is reached: a source mesh carrying a zero-area NEEDLE triangle whose line is the plane
/// a band ends on - "15mm Bevel Top Edge.mcx", whose top face carries (0,20,20), (10,10,20),
/// (20,0,20) along x + y = 20. The knit inserts the band end's tangent point into the needle's
/// side, so what reaches the fan is four corners of one line. Minting a middle for that left the
/// ring's sides claimed by slivers wound against each other and a vertex no neighbour knew.
///
/// DROPPING the ring is not this pass's decision: whether the faces either side carry the ring's
/// middle corners is what says a drop closes a seam rather than opening one, and only the sewing
/// has the segment reading that knows (`SurfaceSplitSewing`). What the fan owes it is a needle
/// whose edges are exactly the ring's.
fn fan_from_a_farthest_corner(
    ring: &[i32],
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
) {
    let (from, _, _) = farthest_apart(ring, pool);
    let count = ring.len();

    let mut i = 1;
    while i + 1 < count {
        let first = ring[from];
        let second = ring[(from + i) % count];
        let third = ring[(from + i + 1) % count];

        // A ring visiting one vertex twice is a pinch, which the sewing splits into lobes before it
        // gets here; a triangle on two of one vertex is no edge of anything either way.
        if first != second && second != third && third != first {
            faces.push((first, second, third));
        }

        i += 1;
    }
}

/// C# `FarthestApart`: the two corners of a ring that stand farthest apart, as their places in
/// it and how far apart they are - the pair that names the line a ring covering nothing lies
/// along.
///
/// One reading, shared: [`covers_nothing`] measures every corner against the line through this
/// pair, and the sewing asks the same pair's own stretch whether the faces beside a lobe carry its
/// middle (`SurfaceSplitSewing`). The two have to be talking about the same line.
pub fn farthest_apart(ring: &[i32], pool: &SurfaceVertexPool) -> (usize, usize, f64) {
    let mut from = 0;
    let mut to = 0;
    let mut apart = 0.0;

    for first in 0..ring.len() {
        for second in first + 1..ring.len() {
            let between = (pool.position_of(ring[second]) - pool.position_of(ring[first])).length();

            if between > apart {
                apart = between;
                from = first;
                to = second;
            }
        }
    }

    (from, to, apart)
}

/// C# `EmitFacing(pool, faces, first, second, third, reference)`: the triangle on three vertices,
/// wound so that it faces the same way as `reference`, appended to `faces`. Nothing is appended for
/// three vertices with no area between them.
pub fn emit_triangle_facing(
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    first: i32,
    second: i32,
    third: i32,
    reference: Vector3,
) {
    if !is_worth_keeping(pool, first, second, third) {
        return;
    }

    let from = pool.position_of(first);
    let normal = (pool.position_of(second) - from).cross(pool.position_of(third) - from);

    faces.push(if normal.dot(reference) >= 0.0 {
        (first, second, third)
    } else {
        (first, third, second)
    });
}

/// C# `IsWorthKeeping`: whether three vertices make a triangle with area in it rather than a flat
/// nothing.
pub fn is_worth_keeping(pool: &SurfaceVertexPool, first: i32, second: i32, third: i32) -> bool {
    if first == second || second == third || third == first {
        return false;
    }

    has_area(
        pool.position_of(first),
        pool.position_of(second),
        pool.position_of(third),
    )
}

/// C# `ApexToFanFrom`: which vertex to fan from - of those that see the whole ring with area in
/// every triangle, the one whose THINNEST triangle is the fattest; -1 where none does.
///
/// The area test alone is not enough, and a cut band on a curved run is where that shows. Three
/// points of one straight rim are collinear in the arithmetic that made them and very slightly not
/// collinear once they are float positions again, so a fan from the wrong end of such a rim keeps
/// a needle a hundredth of a micron across instead of dropping it. Every apex that can fan the ring
/// covers it with the same NUMBER of triangles, so choosing between them costs nothing but this
/// walk.
///
/// Nor is it enough to have area: a triangle of the fan has to lie the way the RING does. A ring
/// with a reflex vertex in it - the bent end pentagon a chamfer strip mitres to - fanned from a
/// vertex the reflex corner hides part of the ring from reaches outside the ring, folding one
/// triangle back over its neighbours with plenty of area in it. So each triangle is required to
/// agree with the ring's own Newell normal, which is the sum of exactly those triangles'
/// contributions. On a convex ring every triangle agrees already and nothing changes.
fn apex_to_fan_from(ring: &[i32], pool: &SurfaceVertexPool) -> i32 {
    let mut best = -1;
    let mut best_worst = 0.0;
    let mut best_is_clean = false;
    let ring_faces = newell_normal_of(ring, pool);
    let count = ring.len();

    for apex in 0..count {
        // A fan from here takes no diagonal THROUGH another corner of the ring: a diagonal with a
        // corner standing on it hides a T junction on itself, since the face across that corner
        // names it and this one spans it. Preferred over a fatter fan that does, and only ranked by
        // thinness among equals - if no apex of the ring manages it, the thinnest-fattest is still
        // taken, which is the answer this had before and no worse than it.
        let clean = no_diagonal_carries_a_corner(ring, pool, apex);

        if best_is_clean && !clean {
            continue;
        }

        let mut worst = f64::MAX;

        let mut i = 1;
        while i + 1 < count && worst > 0.0 {
            let first = ring[apex];
            let second = ring[(apex + i) % count];
            let third = ring[(apex + i + 1) % count];

            worst = worst.min(
                if is_worth_keeping(pool, first, second, third)
                    && lies_the_way_the_ring_does(pool, first, second, third, ring_faces)
                {
                    thinness(pool, first, second, third)
                } else {
                    0.0
                },
            );

            i += 1;
        }

        if worst > best_worst || (clean && !best_is_clean && worst > 0.0) {
            best_worst = worst;
            best = apex as i32;
            best_is_clean = clean;
        }
    }

    best
}

/// C# `NoDiagonalCarriesACorner`: whether a fan from this apex takes no diagonal with another
/// corner of the ring standing on it. Only the diagonals are asked about - the fan's other sides
/// are sides of the ring itself, and a corner standing on one of those was put there by the sewing
/// on purpose.
fn no_diagonal_carries_a_corner(ring: &[i32], pool: &SurfaceVertexPool, apex: usize) -> bool {
    let count = ring.len();

    let mut i = 2;
    while i + 1 < count {
        let from = pool.position_of(ring[apex]);
        let to = pool.position_of(ring[(apex + i) % count]);

        for &corner in ring {
            if corner == ring[apex] || corner == ring[(apex + i) % count] {
                continue;
            }

            if surface_split_sewing::stands_on_the_side(pool, from, to, pool.position_of(corner)).0
            {
                return false;
            }
        }

        i += 1;
    }

    true
}

/// C# `NewellNormalOf`: which way a ring lies as a whole, by Newell's sum over its edges - exact
/// for a planar ring however it is wound, and nothing (a zero vector) for one that covers no area
/// at all.
pub fn newell_normal_of(ring: &[i32], pool: &SurfaceVertexPool) -> Vector3 {
    let mut summed = Vector3::ZERO;

    for i in 0..ring.len() {
        summed += pool
            .position_of(ring[i])
            .cross(pool.position_of(ring[(i + 1) % ring.len()]));
    }

    summed
}

/// C# `LiesTheWayTheRingDoes`: whether a fan triangle faces the same way as the ring it came
/// from. A ring with no direction of its own answers yes, which leaves the area test on its own
/// where there is nothing better - and so does a triangle whose agreement is too small to be about
/// anything but rounding, since a triangle that really is folded back disagrees by the whole of
/// the product and not by a billionth of it.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn lies_the_way_the_ring_does(
    pool: &SurfaceVertexPool,
    first: i32,
    second: i32,
    third: i32,
    ring_faces: Vector3,
) -> bool {
    if !(ring_faces.length_squared() > 0.0) {
        return true;
    }

    let from = pool.position_of(first);
    let facing = (pool.position_of(second) - from).cross(pool.position_of(third) - from);

    facing.dot(ring_faces) > -(facing.length() * ring_faces.length() * AGREES_WITH_THE_RING)
}

/// C# `Thinness`: how far a triangle is from a needle, as its smallest altitude over its longest
/// edge - zero for three points on a line and about 0.87 for an equilateral one, and free of the
/// part's scale.
fn thinness(pool: &SurfaceVertexPool, first: i32, second: i32, third: i32) -> f64 {
    let at = pool.position_of(first);
    let to = pool.position_of(second);
    let last = pool.position_of(third);

    let longest = (to - at)
        .length()
        .max((last - to).length().max((at - last).length()));

    if longest > 0.0 {
        (to - at).cross(last - at).length() / (longest * longest)
    } else {
        0.0
    }
}

/// C# `FanFromTheMiddle`: the ring fanned from a new vertex at its own middle, which every ring
/// can be seen entirely from.
///
/// Only reached by a ring no vertex of which can fan it, and that takes a collinear triple at
/// EVERY vertex - which on a ring means the whole of it lies on one line and covers nothing. So in
/// practice this emits no triangle, and the middle is worked out before it is added to the pool so
/// that a ring covering nothing leaves nothing behind it either.
fn fan_from_the_middle(
    ring: &[i32],
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    facing: Option<Vector3>,
) {
    let mut summed = Vector3::ZERO;

    for &vertex in ring {
        summed += pool.position_of(vertex);
    }

    let count = ring.len();
    let middle = summed / count as f64;
    let mut worth = false;

    let mut i = 0;
    while i < count && !worth {
        worth = has_area(
            middle,
            pool.position_of(ring[i]),
            pool.position_of(ring[(i + 1) % count]),
        );
        i += 1;
    }

    if !worth {
        return;
    }

    let apex = pool.add(middle);

    for i in 0..count {
        let second = ring[i];
        let third = ring[(i + 1) % count];

        if let Some(facing) = facing {
            // The caller asked for triangles facing a given way and gets them here as much as from
            // the ordinary fan: a ring the strip built has no winding of its own to follow, so a
            // triangle wound to follow it is wound at random.
            emit_triangle_facing(pool, faces, apex, second, third, facing);
        } else if is_worth_keeping(pool, apex, second, third) {
            faces.push((apex, second, third));
        }
    }
}

/// C# `HasArea`: whether three positions have area between them rather than standing on one line.
fn has_area(first: Vector3, second: Vector3, third: Vector3) -> bool {
    (second - first).cross(third - first).length() > SMALLEST_TRIANGLE
}
