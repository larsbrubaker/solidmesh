//! Any planar ring cut into triangles by clipping EARS off it: the triangulation anything needs
//! where no single vertex of the ring sees the whole of it and [`super::surface_fan`]'s fan
//! therefore cannot cover it once. Ports `Bevel/Surface/SurfaceRingEars.cs` (C# `internal static
//! class`; a module of free functions here).
//!
//! Written for `SurfaceEndFollowsTheWalkedChords`: a rolled concave section caves in both ways at
//! once - the arc one way, and on a boss the wall's chords the other - so it is star shaped about
//! nothing at all. Everything the fan knows about apexes still applies and is not repeated here;
//! what is different is only that a ring can need more than one.
//!
//! It asks nothing of what the ring IS, so the same tool covers an extended standing feature's own
//! wall (`SurfaceStandingFeatureFaces`), which caves in where the trace it runs out on bends back
//! inside the face. That is why it is named for the ring rather than for the section it was first
//! built for.

use vector_math::Vector3;

use super::surface_fan;
use super::SurfaceVertexPool;

/// C# `TryTriangulateTheRing`: the ring as triangles, by clipping EARS off it, or false where it
/// is no ring this can finish - which the caller has to read as a refusal rather than publish what
/// was emitted. C#'s optional `roundTipsFirst` defaults to false.
///
/// A fan will not do here and [`super::surface_fan`] says why: it covers a ring exactly once only
/// from a vertex that sees the whole of it. A rolled concave section has two sides that cave in -
/// the arc, and the wall's chords, which on a boss bow INTO the section - and between them they can
/// leave no such vertex at all, at which point the fan falls back to the ring's middle and that
/// middle can stand outside the ring. On the half disc boss that is a cap folded over itself:
/// closed, manifold, and carrying a tenth of a cubic millimetre per millimetre that is not there.
///
/// Ear clipping asks less: a corner whose triangle lies the way the ring does and holds no other
/// vertex of the ring is cut off, and a SIMPLE polygon always has one. Points standing part way
/// along a straight side are never cut off as a tip - their own triangle has no area - so each is
/// swallowed by a neighbour's ear with area in it, which is the T junction the fan was careful
/// about.
///
/// Simple is CHECKED rather than assumed, and running out of ears is a refusal rather than a
/// fallback. The theorem is about simple polygons; a ring that crosses itself has no triangulation
/// to find, and a partial one published is exactly the fold this was written to stop.
pub fn try_triangulate_the_ring(
    ring: &[i32],
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    plane_normal: Vector3,
    round_tips_first: bool,
) -> bool {
    if ring.len() < 3 || !it_is_simple(ring, pool, plane_normal) {
        return false;
    }

    // Which way the ring as a whole lies, so that "a corner pointing outwards" is read the same way
    // round whichever order it was handed in.
    let facing = if newell_normal_of(ring, pool).dot(plane_normal) >= 0.0 {
        plane_normal
    } else {
        -plane_normal
    };
    let mut left: Vec<i32> = ring.to_vec();

    while left.len() > 3 {
        let mut clipped = false;

        let mut pass = if round_tips_first { 0 } else { 1 };
        while pass < 2 && !clipped {
            for at in 0..left.len() {
                let before = left[(at + left.len() - 1) % left.len()];
                let tip = left[at];
                let after = left[(at + 1) % left.len()];

                if !is_an_ear(&left, at, pool, facing)
                    || (pass == 0 && !it_stands_off_its_chord(&left, at, pool))
                {
                    continue;
                }

                surface_fan::emit_triangle_facing(pool, faces, before, tip, after, plane_normal);
                left.remove(at);
                clipped = true;
                break;
            }

            pass += 1;
        }

        if !clipped {
            // No ear at all. On a ring that is still SIMPLE and still lying the way it started, that
            // is not a ring with no triangulation - it is a remainder every corner of which is a
            // sliver, under the ears' own floor or with a convexity that reads as noise. Fan it.
            return the_remainder_fans_from_its_own_first_vertex(
                &left,
                pool,
                faces,
                plane_normal,
                facing,
            );
        }
    }

    surface_fan::emit_triangle_facing(pool, faces, left[0], left[1], left[2], plane_normal);
    true
}

/// C# `TheRemainderFansFromItsOwnFirstVertex`: the last resort where the ear walk runs out - a
/// remainder whose every corner is a sliver is covered by a FAN from one of its own vertices, or
/// refused where there is nothing to cover.
///
/// A ring that is simple and wound the way it started is a region with a triangulation whatever
/// its area - a slit thin quad is two triangles - and the ears can still fail to find it, because
/// an ear has to clear [`surface_fan::SMALLEST_TRIANGLE`] and to read convex by more than float
/// noise, and on a remainder of four all but collinear points no corner does either. Refusing
/// there leaves a HOLE in a result that was going to be closed, which is worse than a triangle with
/// almost no area in it.
///
/// The fan is taken from the remainder's LOWEST POOL INDEX rather than from wherever the walk
/// happened to stop, so the same ring gives the same triangles whichever end it was handed in from
/// and wherever the part stands. Its signed areas sum to the ring's own whatever the apex sees,
/// and each of the ring's directed edges is used once, so zero area triangles are kept rather than
/// dropped: dropping one would leave its two boundary edges uncovered.
///
/// What is refused is a ring with NO AREA TO THE LATTICE - four points collinear to the float grid
/// the pool keeps its positions on. There the ring is a line, not a region, and every fanned
/// triangle would be noise (`SurfaceRingEarsBridgedRingTests.ARingCollinearToTheLatticeIsStillRefused`).
/// A repeated vertex is refused for the same reason the caller cares: a bridge's two ends left in
/// one remainder would have the fan use one directed edge twice, which is a fold.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn the_remainder_fans_from_its_own_first_vertex(
    left: &[i32],
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    plane_normal: Vector3,
    facing: Vector3,
) -> bool {
    if !it_is_simple(left, pool, plane_normal) || it_repeats_a_vertex(left) {
        return false;
    }

    let twice_area = newell_normal_of(left, pool).dot(facing);

    if twice_area < 0.0 || !(twice_area > twice_the_smallest_area_the_lattice_can_hold(left, pool))
    {
        // Folded back on itself, or a line rather than a region.
        return false;
    }

    let mut from = 0;

    for at in 1..left.len() {
        if left[at] < left[from] {
            from = at;
        }
    }

    // Added here rather than through surface_fan::emit_triangle_facing, which drops anything under
    // SMALLEST_TRIANGLE: on this remainder EVERY triangle is under it, and dropping one leaves its
    // two boundary edges uncovered, which is the hole this whole fallback is against. Wound to the
    // ring's own way round, turned to face the plane the caller asked for.
    let the_ring_lies_the_way_the_caller_asked = facing.dot(plane_normal) >= 0.0;

    let mut step = 1;
    while step + 1 < left.len() {
        let second = left[(from + step) % left.len()];
        let third = left[(from + step + 1) % left.len()];

        faces.push(if the_ring_lies_the_way_the_caller_asked {
            (left[from], second, third)
        } else {
            (left[from], third, second)
        });

        step += 1;
    }

    true
}

/// C# `ItRepeatsAVertex`: whether any pool index stands in the ring twice - the two sides of a
/// bridge, say.
fn it_repeats_a_vertex(ring: &[i32]) -> bool {
    for at in 0..ring.len() {
        for other in at + 1..ring.len() {
            if ring[at] == ring[other] {
                return true;
            }
        }
    }

    false
}

/// C# `TwiceTheSmallestAreaTheLatticeCanHold`: twice the area below which a ring is a LINE rather
/// than a region - its own extent squared taken down to the round off of the arithmetic that
/// measured it. A sliver is NOT a line - a remainder thinner than the ears' floor still has to be
/// covered - so this floor sits far below that floor.
fn twice_the_smallest_area_the_lattice_can_hold(ring: &[i32], pool: &SurfaceVertexPool) -> f64 {
    let mut low = pool.position_of(ring[0]);
    let mut high = low;

    for &at in ring {
        let position = pool.position_of(at);
        low = Vector3::component_min(low, position);
        high = Vector3::component_max(high, position);
    }

    let extent = (high - low).length();

    // Only what ROUNDING can manufacture is refused, not everything thin: the remainder this fallback
    // was written for is thinner than the ears' own floor and is still a region the result needs
    // covered. Double arithmetic over a reach of this extent carries about fifteen figures, so an
    // area below that is a line read through round off and nothing else.
    extent * extent * 1e-15
}

/// C# `ItIsSimple`: whether a ring crosses nothing of itself - every pair of sides that do not
/// share an end stands clear of the other, READ IN THE RING'S OWN PLANE. Over a handful of sides,
/// so the pairwise walk costs nothing worth saving.
///
/// A ring may be WEAKLY simple - a bridged one is. A flat piece whose outline was several rings is
/// threaded onto one by running out to each inner ring along the piece's own edges and back along
/// the same ones, a slit of no width, and the two sides of that slit lie exactly on top of one
/// another. They are one edge run both ways: the SAME two vertices, in either order, which is how
/// they are told from a ring that genuinely doubles back over itself. Read by index and never by
/// position, because two sides that merely happen to lie on one another are the fold this refuses.
///
/// The plane is the CALLER'S, not the pair's own. Two sides that face one another across the ring
/// run all but opposite ways, so the cross product of their directions is almost nothing, and
/// whatever the ring stands off flat by is then the whole of it. A ring here is only NEARLY flat
/// (a mouth cut through a tessellated wall stands a twentieth of a millimetre off its piece's
/// plane), so the pair's own normal is not a plane at all, and the one the caller already named
/// is (`TwoSidesFacingOneAnotherAcrossANearlyFlatRingDoNotCross`).
fn it_is_simple(ring: &[i32], pool: &SurfaceVertexPool, plane_normal: Vector3) -> bool {
    let count = ring.len();

    for side in 0..count {
        let from = pool.position_of(ring[side]);
        let to = pool.position_of(ring[(side + 1) % count]);

        for other in side + 1..count {
            if (other + 1) % count == side || other == (side + 1) % count {
                continue;
            }

            if ring[side] == ring[(other + 1) % count] && ring[(side + 1) % count] == ring[other] {
                // The two sides of a slit, which is a bridge drawn out and back, not a crossing.
                continue;
            }

            if they_cross_in(
                plane_normal,
                from,
                to,
                pool.position_of(ring[other]),
                pool.position_of(ring[(other + 1) % count]),
                pool.near_enough(),
            ) {
                return false;
            }
        }
    }

    true
}

/// C# `TheyCrossIn`: whether two segments that share no end cross, READ IN THE PLANE THE CALLER
/// NAMES - each straddles the other's line by more than a hair, which is the reading that lets two
/// sides TOUCH along a rounding without being read as crossing. The plane is always the caller's
/// own - a plane made of the two segments' own directions is useless where they run all but
/// parallel, since then it is made of their noise and a plain rectangle can read as self-crossing
/// (see `it_is_simple`).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn they_cross_in(
    plane_normal: Vector3,
    from: Vector3,
    to: Vector3,
    other_from: Vector3,
    other_to: Vector3,
    near_enough: f64,
) -> bool {
    let along = to - from;
    let other_along = other_to - other_from;

    if !(along.length_squared() > 0.0)
        || !(other_along.length_squared() > 0.0)
        || !(plane_normal.length_squared() > 0.0)
    {
        return false;
    }

    let facing = plane_normal.get_normal();

    straddles(from, along, other_from, other_to, facing, near_enough)
        && straddles(other_from, other_along, from, to, facing, near_enough)
}

/// C# `Straddles`: whether the two ends stand on opposite sides of the line, both by more than a
/// hair.
fn straddles(
    from: Vector3,
    along: Vector3,
    first: Vector3,
    second: Vector3,
    facing: Vector3,
    near_enough: f64,
) -> bool {
    let reach = near_enough * along.length();
    let at_first = (first - from).cross(along).dot(facing);
    let at_second = (second - from).cross(along).dot(facing);

    (at_first > reach && at_second < -reach) || (at_first < -reach && at_second > reach)
}

/// C# `ItStandsOffItsChord`: whether the corner at `at` is a corner the ring's OWN RULER can see -
/// its tip stands further off the chord its two neighbours span than the nearness everything else
/// here is read with. A tip that does not is FLAT, and which way a flat tip points is float noise:
/// the same corner of the same ring reads convex where the part was made and reflex once the part
/// is moved.
fn it_stands_off_its_chord(ring: &[i32], at: usize, pool: &SurfaceVertexPool) -> bool {
    let before = pool.position_of(ring[(at + ring.len() - 1) % ring.len()]);
    let tip = pool.position_of(ring[at]);
    let after = pool.position_of(ring[(at + 1) % ring.len()]);

    (tip - before).cross(after - tip).length() > pool.near_enough() * (after - before).length()
}

/// C# `IsAnEar`: whether the corner at `at` can be cut off - its triangle has area, points the way
/// the ring lies, and holds no other vertex of the ring.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn is_an_ear(ring: &[i32], at: usize, pool: &SurfaceVertexPool, facing: Vector3) -> bool {
    let count = ring.len();
    let before = pool.position_of(ring[(at + count - 1) % count]);
    let tip = pool.position_of(ring[at]);
    let after = pool.position_of(ring[(at + 1) % count]);

    let normal = (tip - before).cross(after - tip);

    if normal.dot(facing) <= 0.0 || !(normal.length() > surface_fan::SMALLEST_TRIANGLE) {
        return false;
    }

    // By IDENTITY, not by place in the ring: on a spliced ring the ear's own vertices stand in the
    // ring more than once - the two sides of the slit - and that second occurrence is the same point,
    // not a foreign one standing on the ear.
    for &other in ring {
        if other == ring[at]
            || other == ring[(at + count - 1) % count]
            || other == ring[(at + 1) % count]
        {
            continue;
        }

        if is_inside_or_on_the_rim(pool.position_of(other), before, tip, after, facing) {
            return false;
        }
    }

    true
}

/// C# `IsInsideOrOnTheRim`: whether a point of the plane stands inside the triangle OR ON ITS
/// RIM, read with no ruler at all, so only a point exactly on an edge to the arithmetic counts as
/// on it.
///
/// On the rim counts as inside because of the SLIT. A spliced ring runs out to an inner ring along
/// a zero width slit and back along the same edge, and the slit's vertices come to stand exactly on
/// other corners' triangle edges. Cutting such a corner off cuts ACROSS the slit and leaves the
/// slit's other side - and the inner ring hanging off it - inside the ear: a triangle laid over a
/// hole (`SurfaceRingEarsBridgedRingTests.AHoleVertexStandingOnAnEarsEdgeIsNoEar`).
///
/// No RULER, because a ruler here reads the other way round: the arc's own samples sit a rounding
/// off the chords either side of them, and a nearness wide enough to call a slit vertex "on the
/// edge" calls those samples on it too and vetoes every ear there is.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn is_inside_or_on_the_rim(
    at: Vector3,
    first: Vector3,
    second: Vector3,
    third: Vector3,
    facing: Vector3,
) -> bool {
    for (from, to) in [(first, second), (second, third), (third, first)] {
        let along = to - from;

        if !(along.length_squared() > 0.0) {
            return false;
        }

        if (at - from).cross(along).dot(facing) > 0.0 {
            return false;
        }
    }

    true
}

/// C# `NewellNormalOf` (this class's own, a fan sum from the first vertex rather than
/// [`surface_fan::newell_normal_of`]'s edge sum): the direction a ring as a whole lies in - the sum
/// of its own triangles' contributions.
fn newell_normal_of(ring: &[i32], pool: &SurfaceVertexPool) -> Vector3 {
    let mut normal = Vector3::ZERO;
    let first = pool.position_of(ring[0]);

    let mut i = 1;
    while i + 1 < ring.len() {
        normal += (pool.position_of(ring[i]) - first).cross(pool.position_of(ring[i + 1]) - first);
        i += 1;
    }

    normal
}
