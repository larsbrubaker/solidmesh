//! A chain end whose section the walk rolled along a curved wall: the cap follows the chords the
//! walk passed rather than one straight line. Ports
//! `Bevel/Surface/SurfaceEndFollowsTheWalkedChords.cs` (C# `internal static class`; a module of free
//! functions here). What is here is the concave reading - the section added as the bead's own end
//! cap ([`try_cap_with_the_section`]), ear clipped by [`super::surface_ring_ears`]; the convex
//! reading (`TryTakeTheSection`) and its rim and wedge helpers land with later packages.

use vector_math::Vector3;

use super::{surface_end_cut, surface_ring, surface_ring_ears, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `TryCapWithTheSection`: the section added as the bead's own end cap, bounded on the wall
/// side by the chords the walk passed - the concave reading of the same section the convex end
/// gives up. `Ok` carries C#'s `alongTheProfile` (nothing stands part way along any chord, so one
/// empty list per chord); `Err` is C#'s `failure`, with `faces` as it was handed in.
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
