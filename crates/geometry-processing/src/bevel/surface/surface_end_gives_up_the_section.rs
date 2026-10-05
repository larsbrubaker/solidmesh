//! A convex chain end whose face beyond gives the section up. Ports
//! `Bevel/Surface/SurfaceEndGivesUpTheSection.cs` (C# `internal static class`; a module of free
//! functions here). What is here is the sewing of the faces the subtraction did not rebuild to the
//! vertices it made ([`sew_the_rim`]), the half planes the section is bounded by and the pieces a
//! face leaves outside them ([`pieces_outside_of`], [`pieces_left_of`]); the subtraction itself
//! (`TryGiveItUp`) and its other readings land with later packages.

use std::collections::HashSet;

use vector_math::Vector3;

use super::{surface_cut_clip, surface_fan, SurfaceVertexPool};

/// C# `SewTheRim`: splits any face the subtraction did NOT rebuild wherever one of the vertices it
/// made stands part way along an edge of that face, so the rim sews by index rather than by
/// coincidence. `untouched` names faces of `rebuilt` by index; the vertices the subtraction made are
/// the pool's from `the_first_new_vertex` on.
///
/// The clips are by whole PLANES, and a plane that bounds the section over the end face carries on
/// over the rest of it. Two of the three - the wedge's own two sides - run along the face's own rim
/// and so carry on outside it, where they cost nothing; but a rim that TURNS leaves the line inside
/// the face again beyond the turn, and there the clip lands a vertex on the face's rim with nothing
/// on the far side of that rim split to match - on the C# fixture, the lower left crease carried on
/// past the nose's tip meets the cube's back edge at (0, 0, 13.333): three open edges, measured.
///
/// Splitting the neighbour is safe wherever it fires, wanted or not: a triangle cut at a point that
/// really is on one of its edges covers the same ground it did, and the face on the other side of
/// that edge is cut at the same point by the same test. What it cannot do is invent a T junction.
pub fn sew_the_rim(
    pool: &mut SurfaceVertexPool,
    rebuilt: &mut Vec<(i32, i32, i32)>,
    untouched: &[i32],
    the_first_new_vertex: i32,
) {
    if the_first_new_vertex >= pool.count() {
        return;
    }

    let mut split: Vec<(i32, Vec<i32>)> = Vec::new();

    for &face in untouched {
        let (v0, v1, v2) = rebuilt[face as usize];
        let corners = [v0, v1, v2];
        let mut ring = Vec::with_capacity(3);
        let mut any_standing = false;

        for side in 0..3 {
            ring.push(corners[side]);

            let standing = standing_on(
                pool,
                corners[side],
                corners[(side + 1) % 3],
                the_first_new_vertex,
            );

            any_standing |= !standing.is_empty();
            ring.extend(standing);
        }

        if any_standing {
            split.push((face, ring));
        }
    }

    if split.is_empty() {
        return;
    }

    // Emitted at the end and the originals left as degenerate nothings, because the faces this walks
    // are named by index and rewriting one in place would renumber the rest. Only looked up, so a
    // `HashSet`'s order cannot reach the output.
    let mut dropped = HashSet::new();

    for (face, ring) in &split {
        dropped.insert(*face);
        surface_fan::emit(ring, pool, rebuilt);
    }

    let kept: Vec<(i32, i32, i32)> = rebuilt
        .iter()
        .enumerate()
        .filter(|(face, _)| !dropped.contains(&(*face as i32)))
        .map(|(_, &triangle)| triangle)
        .collect();

    *rebuilt = kept;
}

/// C# `StandingOn`: the vertices the subtraction made that stand strictly between `from` and
/// `to`, in order along that run. C#'s `List.Sort` is unstable and this sort is stable; two pool
/// vertices at one place along one line within the ruler would be one weld, so the tie does not
/// arise.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn standing_on(
    pool: &SurfaceVertexPool,
    from: i32,
    to: i32,
    the_first_new_vertex: i32,
) -> Vec<i32> {
    let start = pool.position_of(from);
    let run = pool.position_of(to) - start;
    let mut found: Vec<(f64, i32)> = Vec::new();

    if !(run.length_squared() > 0.0) {
        return Vec::new();
    }

    for vertex in the_first_new_vertex..pool.count() {
        if vertex == from || vertex == to {
            continue;
        }

        let at = pool.position_of(vertex) - start;
        let where_ = at.dot(run) / run.length_squared();

        if where_ > 0.0 && where_ < 1.0 && (at - (run * where_)).length() < pool.near_enough() {
            found.push((where_, vertex));
        }
    }

    // Every find is strictly inside (0, 1), so no NaN reaches the comparison.
    found.sort_by(|first, second| first.0.total_cmp(&second.0));

    found.into_iter().map(|(_, vertex)| vertex).collect()
}

/// How far into the end face the chain has to lean for the face to stand BEHIND the end, as a
/// cosine - a dimensionless floor.
pub const LEANS_INTO_THE_FACE: f64 = 1e-6;

/// Whether the end face (facing `end_face_outward`) stands behind the chain's end, which leans
/// `into_the_chain`.
pub fn the_face_is_behind_it(into_the_chain: Vector3, end_face_outward: Vector3) -> bool {
    into_the_chain.dot(end_face_outward) > LEANS_INTO_THE_FACE
}

/// Whether the section swallows `face` whole: every corner inside every half plane of
/// `the_wedge` and behind at least one chord of `the_bulge` (whose half spaces are the chords
/// read AWAY from the corner, so the corner side of a chord is the one this reads as outside),
/// to the cut's own ruler.
pub fn the_section_swallows_it(
    pool: &SurfaceVertexPool,
    face: (i32, i32, i32),
    the_wedge: &[(Vector3, Vector3)],
    the_bulge: &[(Vector3, Vector3)],
) -> bool {
    let on_the_cut = surface_cut_clip::on_the_cut(pool);

    for vertex in [face.0, face.1, face.2] {
        let at = pool.position_of(vertex);

        for &(point, inward) in the_wedge {
            if (at - point).dot(inward) < -on_the_cut {
                return false;
            }
        }

        let behind_a_chord = the_bulge.iter().fold(false, |behind, &(point, inward)| {
            behind | ((at - point).dot(inward) < on_the_cut)
        });

        if !behind_a_chord {
            return false;
        }
    }

    true
}

/// The pieces of `ring` the section leaves standing: those outside the wedge's half planes, and
/// whatever of the inside stands beyond the bulge's chords.
pub fn pieces_left_of(
    ring: &[i32],
    pool: &mut SurfaceVertexPool,
    the_wedge: &[(Vector3, Vector3)],
    the_bulge: &[(Vector3, Vector3)],
) -> Vec<Vec<i32>> {
    let (mut pieces, inside) = pieces_outside_of(ring, pool, the_wedge, None);

    let Some(mut inside) = inside else {
        return pieces;
    };

    for &(point, inward) in the_bulge {
        inside = surface_cut_clip::clip_by(&inside, pool, point, inward, None, None);

        if inside.len() < 3 {
            return pieces;
        }
    }

    pieces.push(inside);
    pieces
}

/// The pieces of `ring` outside each half plane of `the_region` in turn, clipping what is left
/// inside as it goes, and that inside (`None` once it is less than a triangle). `snap_to` is
/// passed to every clip.
pub fn pieces_outside_of(
    ring: &[i32],
    pool: &mut SurfaceVertexPool,
    the_region: &[(Vector3, Vector3)],
    snap_to: Option<&[i32]>,
) -> (Vec<Vec<i32>>, Option<Vec<i32>>) {
    let mut pieces = Vec::with_capacity(the_region.len() + 1);

    let mut inside = ring.to_vec();

    for &(point, inward) in the_region {
        let outside = surface_cut_clip::clip_by(&inside, pool, point, -inward, snap_to, None);

        if outside.len() >= 3 {
            pieces.push(outside);
        }

        inside = surface_cut_clip::clip_by(&inside, pool, point, inward, snap_to, None);

        if inside.len() < 3 {
            return (pieces, None);
        }
    }

    (pieces, Some(inside))
}

/// The half plane in the plane facing `plane_normal` bounded by the line `from`..`to`, as a point
/// and the unit inward direction toward `inside`; `None` where the line has no length, lies along
/// the normal, or `inside` stands on it.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_half_plane(
    from: Vector3,
    to: Vector3,
    plane_normal: Vector3,
    inside: Vector3,
) -> Option<(Vector3, Vector3)> {
    let along = to - from;

    if !(along.length_squared() > 0.0) {
        return None;
    }

    let inward = plane_normal.cross(along);

    if !(inward.length_squared() > 0.0) {
        return None;
    }

    let inward = inward.get_normal();

    let reach = (inside - from).dot(inward);

    if reach == 0.0 {
        return None;
    }

    Some((from, if reach > 0.0 { inward } else { -inward }))
}
