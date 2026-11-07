//! A chain end whose section the walk rolled along a curved wall: the cap follows the chords the
//! walk passed rather than one straight line. A module of free functions. What is here is the
//! concave reading - the section added as the bead's own end cap ([`try_cap_with_the_section`]),
//! ear clipped by [`super::surface_ring_ears`]; the walk of the chords the cap follows
//! ([`the_chords_the_cap_follows`]) and the rim, wedge and bulge readings the convex section is
//! made of, and the convex reading itself ([`try_take_the_section`]). The rim is an `IndexMap` and
//! every set an `IndexSet`, because which neighbour the walk tries first and which corner it starts
//! at follow insertion order, which must be the same on every run.

use indexmap::{IndexMap, IndexSet};
use vector_math::{ieee_math, Vector3};

use super::{
    surface_cut_clip, surface_end_cut, surface_end_notch, surface_fan, surface_ring,
    surface_ring_ears, surface_split_sewing, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// The section added as the bead's own end cap, bounded on the wall
/// side by the chords the walk passed - the concave reading of the same section the convex end
/// gives up. `Ok` carries what stands along the profile (nothing stands part way along any chord,
/// so one empty list per chord); `Err` is why not, with `faces` as it was handed in.
///
/// `profile` is the strip's end ring, rail by rail, already in the pool; `wall_end` names which end
/// of it stands on the wall; `corner` is where the chain ends; `plane_normal` is the end face's
/// outward normal, which the cap is wound to face; `walked` is the rim vertices between the corner
/// and the wall foot, in that order.
///
/// EAR CLIPPED as one ring rather than fanned from the corner. A section between two planes is
/// star shaped about its corner and a fan from there covers it exactly once; a rolled one need not
/// be star shaped about anything at all - the arc caves in one way and, on a boss, the wall's
/// chords cave in the other - so a fan either folds or falls back to a middle that stands outside
/// the ring.
///
/// The ring carries whatever the walkback's end cut left standing part way along the section's ONE
/// straight side as well ([`surface_end_cut::standing_between`]), for the same reason the unrolled
/// cap does: a point on that side in no triangle with area is a T junction.
#[allow(clippy::too_many_arguments)]
pub fn try_cap_with_the_section(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    profile: &[i32],
    wall_end: i32,
    corner: Vector3,
    plane_normal: Vector3,
    walked: &[i32],
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let apex = pool.add(corner);
    let flat_end = if wall_end == 0 { profile.len() - 1 } else { 0 };
    let before = faces.len();

    // Round the section: out along the straight side to the flat foot, along the profile to the wall
    // foot, and back to the corner the way the wall's own rim runs.
    let mut ring = Vec::with_capacity(profile.len() + walked.len() + 3);
    ring.push(apex);

    ring.extend(surface_end_cut::standing_between(
        pool,
        &surface_end_cut::vertices_of(faces),
        apex,
        profile[flat_end],
    ));

    if flat_end == 0 {
        ring.extend(profile.iter().copied());
    } else {
        ring.extend(profile.iter().rev().copied());
    }

    ring.extend(walked.iter().rev().copied());

    if !surface_ring_ears::try_triangulate_the_ring(
        &surface_ring::without_repeats(&ring),
        pool,
        faces,
        plane_normal,
        false,
    ) || faces.len() == before
    {
        // A ring the clipper cannot finish, or a section covering no area: either way there is no cap
        // here, and half a cap is worse than none - what it leaves is a fold that closes, is manifold
        // and carries volume nobody asked for. Everything this put in comes back out and the end is
        // refused by name.
        faces.truncate(before);

        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        ));
    }

    // Nothing stands part way along a chord: this face was built from the profile's own vertices, and
    // nothing of the mesh was cut to make room for it.
    Ok(vec![Vec::new(); profile.len() - 1])
}

/// How many rim edges the walk from the corner to the wall's foot may take before it is read as
/// going nowhere.
pub const THE_FURTHEST_WALK: i32 = 64;

/// The rim vertices the cap walks past from the `corner` to the wall's foot (`profile[wall_end]`)
/// along the rim of the end faces `in_the_end` of `faces`: walked BOTH ways round the rim from the
/// corner, the SHORTER arrival being the wall's side of the section. `None` where the corner has
/// no rim of its own, the foot is on the corner's own rim edge (nothing was walked past: the
/// unrolled section), or neither way arrives.
pub fn the_chords_the_cap_follows(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    in_the_end: &IndexSet<i32>,
    profile: &[i32],
    wall_end: usize,
    corner: Vector3,
) -> Option<Vec<i32>> {
    let wall_foot = pool.position_of(profile[wall_end]);
    let mut reach = pool.near_enough();

    for &vertex in profile {
        reach = ieee_math::max_f64(
            reach,
            (pool.position_of(vertex) - corner).length() + pool.near_enough(),
        );
    }

    let rim = the_rim(faces, in_the_end);

    let here = try_the_corners_own_rim(pool, &rim, corner)?;

    let mut shortest: Option<Vec<i32>> = None;

    for &neighbour in &rim[&here] {
        let mut chain = Vec::new();
        let mut seen = IndexSet::new();
        seen.insert(here);
        let mut previous = here;
        let mut at = neighbour;

        for _ in 0..THE_FURTHEST_WALK {
            // Arrival is by POSITION against a rim vertex the walkback made.
            if (pool.position_of(at) - wall_foot).length() < pool.near_enough() {
                if chain.is_empty() {
                    // The foot is on the corner's own rim edge: nothing was walked past, so this
                    // is the unrolled section however the plan read the edge.
                    return None;
                }

                if shortest
                    .as_ref()
                    .is_none_or(|best| chain.len() < best.len())
                {
                    shortest = Some(chain);
                }

                break;
            }

            if !seen.insert(at) || (pool.position_of(at) - corner).length() > reach {
                break;
            }
            let Some(next) = rim.get(&at).filter(|next| next.len() == 2) else {
                // Back on itself, off the end of the section, or a rim that forks: not this walk.
                break;
            };

            chain.push(at);

            let onward = if next[0] == previous {
                next[1]
            } else {
                next[0]
            };

            previous = at;
            at = onward;
        }
    }

    shortest
}

/// The rim of the end faces: every directed side of a face in `in_the_end` whose reverse is not
/// also a side, joined both ways, in the order the faces name them.
pub fn the_rim(faces: &[(i32, i32, i32)], in_the_end: &IndexSet<i32>) -> IndexMap<i32, Vec<i32>> {
    let mut used: IndexSet<(i32, i32)> = IndexSet::new();

    for &face in in_the_end {
        let (v0, v1, v2) = faces[face as usize];
        let corners = [v0, v1, v2];

        for side in 0..3 {
            used.insert((corners[side], corners[(side + 1) % 3]));
        }
    }

    let mut rim = IndexMap::new();

    for &(from, to) in &used {
        if used.contains(&(to, from)) || from == to {
            continue;
        }

        join(&mut rim, from, to);
        join(&mut rim, to, from);
    }

    rim
}

/// Adds `to` to `from`'s rim neighbours, once.
pub fn join(rim: &mut IndexMap<i32, Vec<i32>>, from: i32, to: i32) {
    let joined = rim.entry(from).or_insert_with(|| Vec::with_capacity(2));

    if !joined.contains(&to) {
        joined.push(to);
    }
}

/// The ONE rim vertex standing at `corner`, with exactly two rim neighbours; `None` where none
/// does, or more than one, or it forks.
pub fn try_the_corners_own_rim(
    pool: &SurfaceVertexPool,
    rim: &IndexMap<i32, Vec<i32>>,
    corner: Vector3,
) -> Option<i32> {
    let mut here = None;

    for (&vertex, joined) in rim {
        if (pool.position_of(vertex) - corner).length() < pool.near_enough() {
            if here.is_some() || joined.len() != 2 {
                return None;
            }

            here = Some(vertex);
        }
    }

    here
}

/// The half planes, in the plane facing `plane_normal`, each side of the closed `outline` bounds,
/// facing its middle; `None` where a side has no length or its middle stands on one.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
pub fn try_the_wedge(
    outline: &[Vector3],
    plane_normal: Vector3,
) -> Option<Vec<(Vector3, Vector3)>> {
    let mut the_wedge = Vec::with_capacity(outline.len());

    let mut middle = Vector3::ZERO;
    let count = outline.len() as f64;

    for &at in outline {
        middle += at / count;
    }

    for side in 0..outline.len() {
        let from = outline[side];
        let along = outline[(side + 1) % outline.len()] - from;

        if !(along.length_squared() > 0.0) {
            return None;
        }

        let inward = plane_normal.cross(along);

        if !(inward.length_squared() > 0.0) {
            return None;
        }

        let inward = inward.get_normal();

        let reach = (middle - from).dot(inward);

        if reach == 0.0 {
            return None;
        }

        the_wedge.push((from, if reach > 0.0 { inward } else { -inward }));
    }

    Some(the_wedge)
}

/// Whether every point of `outline` stands inside every half plane of `the_wedge`, to the pool's
/// ruler.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // Negated, so a NaN distance passes.
pub fn it_is_convex(
    outline: &[Vector3],
    the_wedge: &[(Vector3, Vector3)],
    pool: &SurfaceVertexPool,
) -> bool {
    outline.iter().all(|&at| {
        the_wedge
            .iter()
            .all(|&(point, inward)| !((at - point).dot(inward) < -pool.near_enough()))
    })
}

/// Whether `corner` stands outside some half plane of `the_bulge`, to the pool's ruler.
pub fn the_corner_is_outside_the_bulge(
    corner: Vector3,
    the_bulge: &[(Vector3, Vector3)],
    pool: &SurfaceVertexPool,
) -> bool {
    the_bulge
        .iter()
        .any(|&(point, inward)| (corner - point).dot(inward) < -pool.near_enough())
}

/// Whether every vertex of `profile` stands inside every half plane of `the_wedge`, to the
/// pool's ruler.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // Negated, so a NaN distance passes.
pub fn it_holds_the_profile(
    pool: &SurfaceVertexPool,
    profile: &[i32],
    the_wedge: &[(Vector3, Vector3)],
) -> bool {
    profile.iter().all(|&vertex| {
        let at = pool.position_of(vertex);

        the_wedge
            .iter()
            .all(|&(point, inward)| !((at - point).dot(inward) < -pool.near_enough()))
    })
}

/// The half planes each chord of `profile` bounds in the plane facing `plane_normal`, facing the
/// profile's own middle - or, for a profile with no bulge to it (a chamfer's one chord, whose
/// middle is ON it), facing away from `corner`, which leaves the whole polygon as the section.
/// Chords of no length, or along the normal, are skipped.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
pub fn the_bulge(
    pool: &SurfaceVertexPool,
    profile: &[i32],
    corner: Vector3,
    plane_normal: Vector3,
) -> Vec<(Vector3, Vector3)> {
    let mut the_bulge = Vec::with_capacity(profile.len().saturating_sub(1));
    let mut middle = Vector3::ZERO;
    let count = profile.len() as f64;

    for &vertex in profile {
        middle += pool.position_of(vertex) / count;
    }

    for i in 0..profile.len().saturating_sub(1) {
        let from = pool.position_of(profile[i]);
        let along = pool.position_of(profile[i + 1]) - from;

        if !(along.length_squared() > 0.0) {
            continue;
        }

        let inward = plane_normal.cross(along);

        if !(inward.length_squared() > 0.0) {
            continue;
        }

        let inward = inward.get_normal();

        let mut leaning = (middle - from).dot(inward);

        if leaning.abs() < pool.near_enough() {
            leaning = -(corner - from).dot(inward);
        }

        the_bulge.push((from, if leaning > 0.0 { inward } else { -inward }));
    }

    the_bulge
}

/// The pieces of `ring` the section leaves standing: those outside the wedge's half planes, and
/// whatever of the inside stands beyond the bulge's chords.
pub fn pieces_left_of(
    ring: &[i32],
    pool: &mut SurfaceVertexPool,
    the_wedge: &[(Vector3, Vector3)],
    the_bulge: &[(Vector3, Vector3)],
) -> Vec<Vec<i32>> {
    let mut pieces = Vec::with_capacity(the_wedge.len() + 1);
    let mut inside = ring.to_vec();

    for &(point, inward) in the_wedge {
        let outside = surface_cut_clip::clip_by(&inside, pool, point, -inward, None, None);

        if outside.len() >= 3 {
            pieces.push(outside);
        }

        inside = surface_cut_clip::clip_by(&inside, pool, point, inward, None, None);

        if inside.len() < 3 {
            return pieces;
        }
    }

    for &(point, inward) in the_bulge {
        inside = surface_cut_clip::clip_by(&inside, pool, point, inward, None, None);

        if inside.len() < 3 {
            return pieces;
        }
    }

    pieces.push(inside);
    pieces
}

/// Whether the section swallows `face` whole: every corner inside every half plane of
/// `the_wedge` and behind at least one chord of `the_bulge`, to the cut's own ruler.
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

/// The convex reading - the section, bounded on the wall side by the
/// chords the walk passed (`walked`, the rim vertices between the corner and the wall foot), taken
/// out of the end cap (`in_the_end` names its faces) as the polygon its outline names less the bulge
/// the arc puts back. Returns what the cut left standing part way along each chord of the profile,
/// or why the section could not be taken off.
#[allow(clippy::too_many_arguments)]
pub fn try_take_the_section(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    profile: &[i32],
    wall_end: usize,
    corner: Vector3,
    plane_normal: Vector3,
    in_the_end: &IndexSet<i32>,
    walked: &[i32],
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        )
    };
    let flat_foot = pool.position_of(profile[if wall_end == 0 { profile.len() - 1 } else { 0 }]);
    let wall_foot = pool.position_of(profile[wall_end]);

    // The section's own outline, written from the FLAT FOOT round, the chord across the profile's
    // two feet LAST.
    let mut outline = Vec::with_capacity(walked.len() + 3);
    outline.push(flat_foot);
    outline.push(corner);
    outline.extend(walked.iter().map(|&v| pool.position_of(v)));
    outline.push(wall_foot);

    let Some(the_wedge) = try_the_wedge(&outline, plane_normal) else {
        return Err(not_supported());
    };
    if !it_is_convex(&outline, &the_wedge, pool) || !it_holds_the_profile(pool, profile, &the_wedge)
    {
        // A wall that turns the other way, or a section standing outside its own outline.
        return Err(not_supported());
    }

    let the_bulge = the_bulge(pool, profile, corner, plane_normal);
    if the_bulge.is_empty() || !the_corner_is_outside_the_bulge(corner, &the_bulge, pool) {
        return Err(not_supported());
    }

    // Everything the clips add to the pool from here is a point the notch made.
    let split_from = pool.vertices().len() as i32;
    let mut rebuilt = Vec::with_capacity(faces.len());
    let mut cut_faces = Vec::new();

    for (face, &the_face) in faces.iter().enumerate() {
        if !in_the_end.contains(&(face as i32)) {
            rebuilt.push(the_face);
            continue;
        }

        let started_at = rebuilt.len();
        let (v0, v1, v2) = the_face;
        for piece in pieces_left_of(&[v0, v1, v2], pool, &the_wedge, &the_bulge) {
            let snapped =
                surface_end_notch::snapped_to_the_profile(&piece, profile, pool, split_from);
            surface_fan::emit(&snapped, pool, &mut rebuilt);
        }

        if rebuilt.len() == started_at
            && !the_section_swallows_it(pool, the_face, &the_wedge, &the_bulge)
        {
            // A NEEDLE: dropping it would lose the only use its edges had.
            rebuilt.push(the_face);
            continue;
        }

        cut_faces.extend(started_at as i32..rebuilt.len() as i32);
    }

    // Read BEFORE the sew, which drops the faces it splits and appends their pieces.
    let along_the_profile = surface_end_cut::on_the_profile(pool, &rebuilt, &cut_faces, profile);

    surface_split_sewing::sew_in_the_splits_since_unchecked(pool, &mut rebuilt, split_from);

    faces.clear();
    faces.extend(rebuilt);

    Ok(along_the_profile)
}
