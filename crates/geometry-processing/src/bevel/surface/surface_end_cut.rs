//! Cutting a chain end's profile out of the face beyond its corner. Ports
//! `Bevel/Surface/SurfaceEndCut.cs` (C# `internal static class`; a module of free functions here).
//! What is here are the readings the end caps share with the cut: the vertices standing along a
//! straight run ([`standing_between`]), the vertices a face list uses ([`vertices_of`]), the
//! vertices standing on each chord of the profile ([`on_the_profile`]) and the end face the profile
//! lies in ([`try_find_the_end_plane`]), and the cut itself ([`try_cut`]).

use indexmap::IndexSet;
use vector_math::Vector3;

use std::collections::HashMap;

use super::super::{BevelAdvisory, BevelAdvisoryCode};
use super::{
    surface_cut_clip, surface_end_follows_the_walked_chords, surface_end_gains_the_section,
    surface_end_gives_up_the_section, surface_end_notch, surface_fan, surface_ring,
    surface_split_sewing, SurfaceEndGainsTheSectionReading, SurfaceFacetCut, SurfaceLine,
    SurfaceVertexPool,
};

/// C# `StandingBetween`: the vertices standing part way along the straight run from `from` to
/// `to`, in order along it - what the walkback's own cut left on one side of the section, gathered
/// so the cap can sew to it. `standing` is C#'s `IReadOnlyCollection<int>`, read in its own order.
///
/// Read over the mesh's own vertices rather than off a named few, because the points wanted here
/// were made by the WALKBACK - in whichever region's facets its end cut crossed - and this step has
/// no list of those.
///
/// What it takes is therefore ANY vertex of the mesh within [`SurfaceVertexPool::near_enough`] of
/// the 3D segment, not only the ones that cut made: a vertex elsewhere in the part that happens to
/// fall on this few-millimetre run between the chain's corner and its tangent point would be sewn
/// in too. That false positive fails safe rather than silently - a cap fanned through a point that
/// is not on its boundary leaves an open edge, and the build is refused at the closure gate
/// (`BevelSurfacePartBuilder`, [`super::surface_mesh_closure`]) instead of published.
///
/// C# orders the finds with `List.Sort`, which is unstable; this sort is stable, so two finds at
/// the same place along the run keep `standing`'s order. Two vertices at one place along one line
/// within the ruler are one weld, which the pool does not hand out twice, so the tie does not occur.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn standing_between<'a>(
    pool: &SurfaceVertexPool,
    standing: impl IntoIterator<Item = &'a i32>,
    from: i32,
    to: i32,
) -> Vec<i32> {
    let start = pool.position_of(from);
    let run = pool.position_of(to) - start;
    let mut found: Vec<(f64, i32)> = Vec::new();

    if !(run.length_squared() > 0.0) {
        return Vec::new();
    }

    // The run's own box, opened by the same nearness the test below uses: a vertex outside it cannot
    // pass, and most of the part's vertices are outside it, so this is the whole of the saving.
    let end = start + run;
    let near = pool.near_enough();
    let low = Vector3::component_min(start, end) - Vector3::new(near, near, near);
    let high = Vector3::component_max(start, end) + Vector3::new(near, near, near);

    for &vertex in standing {
        if vertex == from || vertex == to {
            continue;
        }

        let position = pool.position_of(vertex);

        if position.x < low.x
            || position.x > high.x
            || position.y < low.y
            || position.y > high.y
            || position.z < low.z
            || position.z > high.z
        {
            continue;
        }

        let at = position - start;
        let where_ = at.dot(run) / run.length_squared();

        if where_ > 0.0 && where_ < 1.0 && (at - (run * where_)).length() < pool.near_enough() {
            found.push((where_, vertex));
        }
    }

    // C# `first.Where.CompareTo(second.Where)`: the finds are strictly inside (0, 1), so no NaN.
    found.sort_by(|first, second| first.0.total_cmp(&second.0));

    found.into_iter().map(|(_, vertex)| vertex).collect()
}

/// C# `VerticesOf`: every vertex the faces use, once each. An [`IndexSet`] keeps C#'s `HashSet<int>`
/// enumeration order (insertion order, nothing being removed), which reaches
/// [`standing_between`]'s tie order.
pub fn vertices_of(faces: &[(i32, i32, i32)]) -> IndexSet<i32> {
    let mut used = IndexSet::new();

    for face in faces {
        used.insert(face.0);
        used.insert(face.1);
        used.insert(face.2);
    }

    used
}

/// How near two face normals have to agree to be one end face - a dimensionless floor.
pub const FACE_NORMALS_AGREE: f64 = 1e-6;

/// Whether every vertex of `facet` lies on the plane through `point` facing `inward`, to the
/// cut's own ruler.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `>`/`<` tests, written so a NaN reads as C# reads it.
pub fn it_lies_along_the_side(
    pool: &SurfaceVertexPool,
    facet: &[i32],
    point: Vector3,
    inward: Vector3,
) -> bool {
    let on_the_cut = surface_cut_clip::on_the_cut(pool);

    facet
        .iter()
        .all(|&vertex| !((pool.position_of(vertex) - point).dot(inward).abs() > on_the_cut))
}

/// For each chord of `profile`, the vertices of the `cut_faces` of `rebuilt` standing strictly
/// inside it (within the pool's ruler of the chord), ordered along it.
///
/// The candidates are taken in the order the faces name them (C#'s `HashSet<int>` of small ints
/// iterates in that order), and the sort along the chord is stable where C#'s `List.Sort` is
/// not: two vertices at one parameter would be one weld apart, which the pool has already merged.
pub fn on_the_profile(
    pool: &SurfaceVertexPool,
    rebuilt: &[(i32, i32, i32)],
    cut_faces: &[i32],
    profile: &[i32],
) -> Vec<Vec<i32>> {
    let mut candidates = IndexSet::new();

    for &face in cut_faces {
        let (v0, v1, v2) = rebuilt[face as usize];
        candidates.insert(v0);
        candidates.insert(v1);
        candidates.insert(v2);
    }

    let mut found = Vec::with_capacity(profile.len().saturating_sub(1));

    for chord in 0..profile.len().saturating_sub(1) {
        let from = pool.position_of(profile[chord]);
        let run = pool.position_of(profile[chord + 1]) - from;
        let mut along: Vec<(f64, i32)> = Vec::new();

        for &vertex in &candidates {
            if vertex == profile[chord] || vertex == profile[chord + 1] {
                continue;
            }

            let at = pool.position_of(vertex) - from;
            let where_ = at.dot(run) / run.length_squared();

            if where_ > 0.0 && where_ < 1.0 && (at - (run * where_)).length() < pool.near_enough() {
                along.push((where_, vertex));
            }
        }

        along.sort_by(|first, second| first.0.total_cmp(&second.0));

        found.push(along.into_iter().map(|(_, vertex)| vertex).collect());
    }

    found
}

/// The end face the `profile` lies in at `corner`: the faces touching the corner that hold the
/// whole profile, flooded across every face coplanar with them. Returns the plane's normal and
/// the faces of the end (in the order the flood reached them), or `None` where no face at the
/// corner holds the profile or two that do face different ways - there is no one end face to cut
/// then, and which was taken would be down to face order.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_find_the_end_plane(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    profile: &[i32],
    corner: Vector3,
) -> Option<(Vector3, IndexSet<i32>)> {
    let mut plane_normal = Vector3::ZERO;
    let mut seeds: Vec<i32> = Vec::new();

    for (face, &vertices) in faces.iter().enumerate() {
        let corners = corners_of(pool, vertices);

        if !corners
            .iter()
            .any(|&at| (at - corner).length() < pool.near_enough())
        {
            continue;
        }

        let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);

        if !(normal.length_squared() > 0.0) {
            continue;
        }

        let normal = normal.get_normal();

        if !holds_the_profile(pool, profile, corner, normal) {
            continue;
        }

        if !seeds.is_empty() && normal.dot(plane_normal) < 1.0 - FACE_NORMALS_AGREE {
            return None;
        }

        plane_normal = normal;
        seeds.push(face as i32);
    }

    if seeds.is_empty() {
        return None;
    }

    // Everything coplanar, indexed by the vertices it uses, so the flood can walk face to face.
    // Looked up only, so the HashMap's order cannot reach an output.
    let mut faces_at_vertex: HashMap<i32, Vec<i32>> = HashMap::new();

    for (face, &vertices) in faces.iter().enumerate() {
        let corners = corners_of(pool, vertices);

        if corners
            .iter()
            .all(|&at| (at - corner).dot(plane_normal).abs() < pool.near_enough())
        {
            for vertex in [vertices.0, vertices.1, vertices.2] {
                faces_at_vertex.entry(vertex).or_default().push(face as i32);
            }
        }
    }

    // C#'s Stack seeded with the seeds in order: the last seed is walked first.
    let mut walking = seeds;
    let mut in_the_end = IndexSet::new();

    while let Some(face) = walking.pop() {
        if !in_the_end.insert(face) {
            continue;
        }

        let (v0, v1, v2) = faces[face as usize];

        for vertex in [v0, v1, v2] {
            // A face reached from a seed is only in the index while the flood stays on the plane -
            // ask rather than assume, so a seed the coplanar test reads differently is a flood
            // that stops.
            let Some(neighbours) = faces_at_vertex.get(&vertex) else {
                continue;
            };

            for &neighbour in neighbours {
                if !in_the_end.contains(&neighbour) {
                    walking.push(neighbour);
                }
            }
        }
    }

    Some((plane_normal, in_the_end))
}

/// Whether every vertex of `profile` lies on the plane through `corner` facing `normal`, to the
/// pool's ruler.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `>`/`<` tests, written so a NaN reads as C# reads it.
pub fn holds_the_profile(
    pool: &SurfaceVertexPool,
    profile: &[i32],
    corner: Vector3,
    normal: Vector3,
) -> bool {
    profile.iter().all(|&vertex| {
        !((pool.position_of(vertex) - corner).dot(normal).abs() >= pool.near_enough())
    })
}

/// The three corners of `face`.
pub fn corners_of(pool: &SurfaceVertexPool, face: (i32, i32, i32)) -> [Vector3; 3] {
    [
        pool.position_of(face.0),
        pool.position_of(face.1),
        pool.position_of(face.2),
    ]
}

/// C# `SurfaceEndCut.TryCut`: the notch a chain end's section takes out of the end face beyond its
/// corner, routed by what the end is: a run-out's cap (`runs_out_facing`), a concave end's
/// subtraction or cap, the walked chords along a rolled wall (`rolled_wall_end`), a face that gains
/// the section, or the ordinary notch - the face cut facet by facet by the section's two sides and
/// the profile's chords (all but the chord into `the_die_point`), the pieces knitted and fanned and
/// the cut's new vertices sewn back in. Returns what stands part way along each chord of the
/// profile, or why the end could not be cut.
#[allow(clippy::too_many_arguments)]
pub fn try_cut(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    profile: &[i32],
    corner: Vector3,
    concave: bool,
    into_the_chain: Vector3,
    runs_out_facing: Vector3,
    rolled_wall_end: i32,
    the_die_point: i32,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let not_supported = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![at.into()],
        )
    };
    if profile.len() < 2 {
        return Err(not_supported(corner));
    }

    if runs_out_facing.length_squared() > 0.0 {
        let facing = if concave {
            -runs_out_facing.get_normal()
        } else {
            runs_out_facing.get_normal()
        };
        return surface_end_notch::try_cap_with_the_section(pool, faces, profile, corner, facing);
    }

    let Some((plane_normal, in_the_end)) = try_find_the_end_plane(pool, faces, profile, corner)
    else {
        return Err(not_supported(corner));
    };

    let walls_end = (rolled_wall_end >= 0 && (rolled_wall_end as usize) < profile.len())
        .then_some(rolled_wall_end as usize);

    if concave {
        if surface_end_gives_up_the_section::the_face_is_behind_it(into_the_chain, plane_normal) {
            return surface_end_gives_up_the_section::try_give_it_up(
                pool,
                faces,
                profile,
                corner,
                plane_normal,
                &in_the_end,
            );
        }

        if let Some(wall_end) = walls_end {
            if let Some(along_the_wall) =
                surface_end_follows_the_walked_chords::the_chords_the_cap_follows(
                    pool,
                    faces,
                    &in_the_end,
                    profile,
                    wall_end,
                    corner,
                )
            {
                return surface_end_follows_the_walked_chords::try_cap_with_the_section(
                    pool,
                    faces,
                    profile,
                    rolled_wall_end,
                    corner,
                    plane_normal,
                    &along_the_wall,
                );
            }
        }

        return surface_end_notch::try_cap_with_the_section(
            pool,
            faces,
            profile,
            corner,
            plane_normal,
        );
    }

    if let Some(wall_end) = walls_end {
        if let Some(walked_chords) =
            surface_end_follows_the_walked_chords::the_chords_the_cap_follows(
                pool,
                faces,
                &in_the_end,
                profile,
                wall_end,
                corner,
            )
        {
            return surface_end_follows_the_walked_chords::try_take_the_section(
                pool,
                faces,
                profile,
                wall_end,
                corner,
                plane_normal,
                &in_the_end,
                &walked_chords,
            );
        }
    }

    let mut cuts = Vec::with_capacity(profile.len() - 1);
    for i in 0..profile.len() - 1 {
        let from = pool.position_of(profile[i]);
        let to = pool.position_of(profile[i + 1]);
        let chord = to - from;
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN skips, as the C#'s `!(x > 0)` does
        let no_chord = !(chord.length_squared() > 0.0);
        if no_chord {
            continue;
        }

        let mut away = plane_normal.cross(chord).get_normal();
        let how_far_off = (corner - from).dot(away);

        let is_the_chord_into_the_die_point =
            the_die_point >= 0 && (profile[i] == the_die_point || profile[i + 1] == the_die_point);
        if is_the_chord_into_the_die_point {
            continue;
        }

        if how_far_off.abs() <= pool.near_enough() {
            return Err(not_supported(from));
        }

        if how_far_off > 0.0 {
            away = -away;
        }

        cuts.push(SurfaceFacetCut::with_own_ends(
            SurfaceLine::new(from, chord.get_normal()),
            away,
            vec![profile[i], profile[i + 1]],
        ));
    }

    if cuts.is_empty() {
        return Err(not_supported(corner));
    }

    let sides = surface_end_notch::the_sides_of_the_section(pool, profile, corner, plane_normal);
    if !surface_end_notch::the_section_stands_in_the_wedge(pool, profile, &sides) {
        return Err(not_supported(corner));
    }

    if surface_end_gains_the_section::how_the_face_stands_to_it(
        pool,
        faces,
        &in_the_end,
        profile,
        corner,
        plane_normal,
        &sides,
    ) == SurfaceEndGainsTheSectionReading::TheFaceIsBesideIt
    {
        return surface_end_gains_the_section::try_gain_it(
            pool,
            faces,
            profile,
            corner,
            plane_normal,
        );
    }

    let reach =
        surface_end_notch::the_reach_of_the_section(pool, profile, corner) + pool.near_enough();
    let low = corner - Vector3::new(reach, reach, reach);
    let high = corner + Vector3::new(reach, reach, reach);

    let split_from = pool.vertices().len() as i32;
    let mut rebuilt = Vec::with_capacity(faces.len());
    let mut cut_faces = Vec::new();

    for (face, &the_face) in faces.iter().enumerate() {
        if !in_the_end.contains(&(face as i32))
            || !surface_end_notch::the_section_could_reach(pool, the_face, low, high)
        {
            rebuilt.push(the_face);
            continue;
        }

        let started_at = rebuilt.len();
        let mut pieces = Vec::with_capacity(sides.len() + 1);
        let mut in_the_wedge = vec![the_face.0, the_face.1, the_face.2];

        for (point, inward, ends) in &sides {
            if in_the_wedge.len() < 3 {
                break;
            }

            if !it_lies_along_the_side(pool, &in_the_wedge, *point, *inward) {
                let outside = surface_cut_clip::clip_by(
                    &in_the_wedge,
                    pool,
                    *point,
                    -*inward,
                    None,
                    Some(ends),
                );
                if outside.len() >= 3 {
                    pieces.push(outside);
                }
            }

            in_the_wedge =
                surface_cut_clip::clip_by(&in_the_wedge, pool, *point, *inward, None, Some(ends));
        }

        for cut in &cuts {
            if in_the_wedge.len() < 3 {
                break;
            }

            in_the_wedge = cut.clip(&in_the_wedge, pool);
        }

        if in_the_wedge.len() >= 3 {
            pieces.push(in_the_wedge);
        }

        for piece in surface_ring::knit(pieces, pool) {
            let snapped =
                surface_end_notch::snapped_to_the_profile(&piece, profile, pool, split_from);
            surface_fan::emit(&snapped, pool, &mut rebuilt);
        }

        cut_faces.extend(started_at as i32..rebuilt.len() as i32);
    }

    let along_the_profile = on_the_profile(pool, &rebuilt, &cut_faces, profile);

    surface_split_sewing::sew_in_the_splits_since_unchecked(pool, &mut rebuilt, split_from);

    faces.clear();
    faces.extend(rebuilt);

    Ok(along_the_profile)
}
