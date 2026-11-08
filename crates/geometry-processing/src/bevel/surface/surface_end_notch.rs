//! The NOTCH a chain end's section takes out of the face beyond its corner
//! (free functions). The section's sides,
//! reach and wedge tests, the snap of a cut ring onto the profile, and the cap
//! ([`try_cap_with_the_section`]). The notch itself is cut by `surface_end_gives_up_the_section`
//! and `surface_end_gains_the_section`, which build on these.

use solidmesh_math::{ieee_math, Vector3};

use super::super::{BevelAdvisory, BevelAdvisoryCode};
use super::{surface_end_cut, surface_fan, surface_ring, SurfaceVertexPool};

/// One straight side of the section: a point on it (the corner), the unit direction into the
/// section, and the vertices standing on it by construction (the corner's own vertex where the
/// build stood one there, and the profile's end).
pub type SurfaceEndNotchSide = (Vector3, Vector3, Vec<i32>);

/// The section's two straight sides - from the `corner` to each end of the `profile`, in the plane
/// facing `plane_normal` - each facing the other end. A side the other lies along, or one of no
/// length, is left out.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
pub fn the_sides_of_the_section(
    pool: &SurfaceVertexPool,
    profile: &[i32],
    corner: Vector3,
    plane_normal: Vector3,
) -> Vec<SurfaceEndNotchSide> {
    let mut sides = Vec::with_capacity(2);

    let first = pool.position_of(profile[0]) - corner;
    let last = pool.position_of(profile[profile.len() - 1]) - corner;

    // Each side runs from the corner to one end of the profile, so those two vertices stand ON it
    // by construction - and a vertex the CORNER itself is, where the build has stood one there.
    let at_the_corner = pool.try_find_near(corner, pool.near_enough(), Some(&|_| true));

    for (along, toward, end) in [
        (first, last, profile[0]),
        (last, first, profile[profile.len() - 1]),
    ] {
        if !(along.length_squared() > 0.0) || !(toward.length_squared() > 0.0) {
            continue;
        }

        let inward = plane_normal.cross(along).get_normal();
        let leaning = toward.get_normal().dot(inward);

        if leaning.abs() < 1e-9 {
            // The other side lies along this one, so there is no side of it the section is on.
            continue;
        }

        sides.push((
            corner,
            if leaning > 0.0 { inward } else { -inward },
            match at_the_corner {
                Some(own) => vec![own, end],
                None => vec![end],
            },
        ));
    }

    sides
}

/// How far the section reaches from `corner`: its furthest profile vertex.
pub fn the_reach_of_the_section(pool: &SurfaceVertexPool, profile: &[i32], corner: Vector3) -> f64 {
    let mut reach = 0.0;

    for &vertex in profile {
        let here = (pool.position_of(vertex) - corner).length();
        reach = ieee_math::max_f64(reach, here);
    }

    reach
}

/// Whether every profile vertex stands inside both of the section's `sides`, to the pool's ruler.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // A negated `x < y` refusal, which a NaN passes.
pub fn the_section_stands_in_the_wedge(
    pool: &SurfaceVertexPool,
    profile: &[i32],
    sides: &[SurfaceEndNotchSide],
) -> bool {
    sides.iter().all(|(point, inward, _)| {
        profile.iter().all(|&vertex| {
            !((pool.position_of(vertex) - *point).dot(*inward) < -pool.near_enough())
        })
    })
}

/// Whether `face`'s bounding box overlaps the section's (`low`..`high`).
pub fn the_section_could_reach(
    pool: &SurfaceVertexPool,
    face: (i32, i32, i32),
    low: Vector3,
    high: Vector3,
) -> bool {
    let mut lowest = pool.position_of(face.0);
    let mut highest = lowest;

    for vertex in [face.1, face.2] {
        let at = pool.position_of(vertex);

        lowest = Vector3::component_min(lowest, at);
        highest = Vector3::component_max(highest, at);
    }

    highest.x >= low.x
        && lowest.x <= high.x
        && highest.y >= low.y
        && lowest.y <= high.y
        && highest.z >= low.z
        && lowest.z <= high.z
}

/// `ring` with every vertex from `only_from` on snapped onto the first `profile` vertex within the
/// pool's snapping distance of it, and the repeats that leaves taken out.
pub fn snapped_to_the_profile(
    ring: &[i32],
    profile: &[i32],
    pool: &SurfaceVertexPool,
    only_from: i32,
) -> Vec<i32> {
    let mut snapped = Vec::with_capacity(ring.len());

    for &vertex in ring {
        if vertex < only_from {
            snapped.push(vertex);
            continue;
        }

        let at = pool.position_of(vertex);
        let landed = profile
            .iter()
            .copied()
            .find(|&on| (pool.position_of(on) - at).length() < pool.snap_near())
            .unwrap_or(vertex);

        snapped.push(landed);
    }

    surface_ring::without_repeats(&snapped)
}

/// The section laid as the cap that closes the strip's end, fanned from
/// a new vertex at `corner` and wound to face `plane_normal`. The two end triangles go as rings with
/// whatever the walkback's end cut left on the section's straight sides, so every such point gets a
/// triangle with area. Returns what stands part way along each chord of the profile (nothing,
/// here), or why the cap could not be laid.
pub fn try_cap_with_the_section(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    profile: &[i32],
    corner: Vector3,
    plane_normal: Vector3,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let apex = pool.add(corner);
    let last = profile.len() - 1;
    let before = faces.len();

    // What the walkback's end cut left standing on the section's two straight sides, walked ONCE.
    let standing = surface_end_cut::vertices_of(faces);
    let from_the_corner = surface_end_cut::standing_between(pool, &standing, apex, profile[0]);
    let back_to_the_corner =
        surface_end_cut::standing_between(pool, &standing, profile[last], apex);

    if profile.len() == 2 {
        // A chamfer's section is one triangle, and both of the straight sides belong to it.
        let mut whole = vec![apex];
        whole.extend(&from_the_corner);
        whole.push(profile[0]);
        whole.push(profile[1]);
        whole.extend(&back_to_the_corner);
        surface_fan::emit_facing(&whole, pool, faces, plane_normal);
    } else {
        // A round's section is not convex, so only the two end triangles go as rings.
        let mut first = vec![apex];
        first.extend(&from_the_corner);
        first.push(profile[0]);
        first.push(profile[1]);
        surface_fan::emit_facing(&first, pool, faces, plane_normal);

        for i in 1..profile.len() - 2 {
            surface_fan::emit_triangle_facing(
                pool,
                faces,
                apex,
                profile[i],
                profile[i + 1],
                plane_normal,
            );
        }

        let mut second = vec![profile[last - 1], profile[last]];
        second.extend(&back_to_the_corner);
        second.push(apex);
        surface_fan::emit_facing(&second, pool, faces, plane_normal);
    }

    if faces.len() <= before {
        // An end with no cap is an open mesh: refuse it rather than publish the hole.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        ));
    }

    // Nothing stands part way along a chord: this face was built from the profile's own vertices.
    Ok(vec![Vec::new(); profile.len() - 1])
}
