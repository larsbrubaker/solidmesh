//! The NOTCH a chain end's section takes out of the face beyond its corner
//! (`Bevel/Surface/SurfaceEndNotch.cs`, a static class: free functions here). The section's sides,
//! reach and wedge tests, and the snap of a cut ring onto the profile, land here first; the cap
//! (`TryCapWithTheSection`) and the notch itself land with later packages.

use vector_math::Vector3;

use super::{surface_ring, SurfaceVertexPool};

/// One straight side of the section: a point on it (the corner), the unit direction into the
/// section, and the vertices standing on it by construction (the corner's own vertex where the
/// build stood one there, and the profile's end).
pub type SurfaceEndNotchSide = (Vector3, Vector3, Vec<i32>);

/// The section's two straight sides - from the `corner` to each end of the `profile`, in the plane
/// facing `plane_normal` - each facing the other end. A side the other lies along, or one of no
/// length, is left out.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
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
        // C#'s Math.Max, which passes a NaN through.
        reach = if here.is_nan() || f64::is_nan(reach) {
            f64::NAN
        } else {
            f64::max(reach, here)
        };
    }

    reach
}

/// Whether every profile vertex stands inside both of the section's `sides`, to the pool's ruler.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `x < y` refusal, which a NaN passes.
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
