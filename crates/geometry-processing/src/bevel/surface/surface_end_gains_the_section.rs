//! The mirror of `SurfaceEndGivesUpTheSection` (`Bevel/Surface/SurfaceEndGainsTheSection.cs`, a
//! static class): a CONVEX end whose face beyond the corner GAINS the section rather than giving it
//! up, because the corner is a REENTRANT vertex of that face and the section stands in the bite
//! beside it rather than in the face at all. Its nested reading lands here
//! ([`SurfaceEndGainsTheSectionReading`]) with the measure that makes it
//! ([`how_the_face_stands_to_it`]), and the cap ([`try_gain_it`]).

/// C# `SurfaceEndGainsTheSection.Reading`: which of the two the end face is - it holds some of the
/// section and is CUT, or it holds none of it and GAINS it.
///
/// Read as the area of the section the face covers, and not the corner's angle: a reentrant vertex
/// is one the face turns through more than half a turn at, which no one facet can be asked about. A
/// face that covers NONE of the section is one the section stands beside, and is gained; a face
/// covering only part of it is the ordinary case and not a fault. And covered means covered OF THE
/// SECTION, not of the wedge, whose two sides are infinite half spaces.
use indexmap::IndexSet;
use vector_math::Vector3;

use super::super::BevelAdvisory;
use super::surface_end_notch::{self, SurfaceEndNotchSide};
use super::SurfaceVertexPool;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceEndGainsTheSectionReading {
    /// The section stands in the end face, which gives it up - `SurfaceEndCut`'s own path.
    #[default]
    TheFaceHoldsIt = 0,

    /// The corner is reentrant and the section stands beside the face, which gains it as a cap.
    TheFaceIsBesideIt,
}

/// Whether the end face (`in_the_end`, faces of `faces`) HOLDS the section between its two
/// `sides`, or stands BESIDE it: the area of the face inside the wedge that the section's chords
/// take off, against a sliver one ruler wide over the section's whole reach. With fewer than two
/// sides there is no wedge, and the face holds it as it always did.
///
/// The faces are summed in the order the end was flooded (`IndexSet`, C#'s insertion-ordered
/// `HashSet`), so the float sum is C#'s.
pub fn how_the_face_stands_to_it(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    in_the_end: &IndexSet<i32>,
    profile: &[i32],
    corner: Vector3,
    plane_normal: Vector3,
    sides: &[SurfaceEndNotchSide],
) -> SurfaceEndGainsTheSectionReading {
    if sides.len() < 2 {
        // No wedge to read the face against: the section is unbounded sideways and the cut is as
        // it always was.
        return SurfaceEndGainsTheSectionReading::TheFaceHoldsIt;
    }

    // How thin a sliver counts as none of the face at all.
    let reach = surface_end_notch::the_reach_of_the_section(pool, profile, corner);
    let sliver = pool.near_enough() * reach;

    // The corner's own box, which is ECONOMY and not the answer: it saves clipping the far side of
    // a large region facet by facet.
    let half = reach + pool.near_enough();
    let bound = Vector3::new(half, half, half);
    let low = corner - bound;
    let high = corner + bound;

    let chords = the_chords_of_the_section(pool, profile, corner, plane_normal);
    let mut covered = 0.0;

    for &face in in_the_end {
        let vertices = faces[face as usize];

        if !surface_end_notch::the_section_could_reach(pool, vertices, low, high) {
            continue;
        }

        let mut in_the_wedge = vec![
            pool.position_of(vertices.0),
            pool.position_of(vertices.1),
            pool.position_of(vertices.2),
        ];

        for (point, inward, _) in sides {
            in_the_wedge = kept_by(&in_the_wedge, *point, *inward, pool.near_enough());

            if in_the_wedge.len() < 3 {
                break;
            }
        }

        if in_the_wedge.len() < 3 {
            continue;
        }

        // What the CUT would leave of this piece: the wedge less the section is what every chord
        // keeps, so what the facet covers OF THE SECTION is what the chords take off it.
        let mut kept_by_the_chords = in_the_wedge.clone();

        for &(point, away) in &chords {
            kept_by_the_chords = kept_by(&kept_by_the_chords, point, away, pool.near_enough());

            if kept_by_the_chords.len() < 3 {
                break;
            }
        }

        covered += area_of(&in_the_wedge)
            - (if kept_by_the_chords.len() >= 3 {
                area_of(&kept_by_the_chords)
            } else {
                0.0
            });
    }

    if covered <= sliver {
        SurfaceEndGainsTheSectionReading::TheFaceIsBesideIt
    } else {
        SurfaceEndGainsTheSectionReading::TheFaceHoldsIt
    }
}

/// The section's chords as half planes: each chord of `profile` (in the plane facing
/// `plane_normal`) with the unit direction AWAY from the `corner`; chords of no length, or that
/// pass within the pool's ruler of the corner, are left out.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn the_chords_of_the_section(
    pool: &SurfaceVertexPool,
    profile: &[i32],
    corner: Vector3,
    plane_normal: Vector3,
) -> Vec<(Vector3, Vector3)> {
    let mut chords = Vec::with_capacity(profile.len().saturating_sub(1));

    for i in 0..profile.len().saturating_sub(1) {
        let from = pool.position_of(profile[i]);
        let chord = pool.position_of(profile[i + 1]) - from;

        if !(chord.length_squared() > 0.0) {
            continue;
        }

        let away = plane_normal.cross(chord).get_normal();
        let how_far_off = (corner - from).dot(away);

        if how_far_off.abs() <= pool.near_enough() {
            continue;
        }

        chords.push((from, if how_far_off > 0.0 { -away } else { away }));
    }

    chords
}

/// `polygon` clipped to the side of the plane through `point` that `inward` faces, to
/// `near_enough`: points within it are kept, and a side is split only where it crosses clearly.
pub fn kept_by(
    polygon: &[Vector3],
    point: Vector3,
    inward: Vector3,
    near_enough: f64,
) -> Vec<Vector3> {
    let mut kept = Vec::with_capacity(polygon.len() + 1);

    for i in 0..polygon.len() {
        let here = polygon[i];
        let next = polygon[(i + 1) % polygon.len()];
        let on_this_side = (here - point).dot(inward);
        let on_that_side = (next - point).dot(inward);

        if on_this_side >= -near_enough {
            kept.push(here);
        }

        if (on_this_side > near_enough && on_that_side < -near_enough)
            || (on_this_side < -near_enough && on_that_side > near_enough)
        {
            kept.push(here + ((next - here) * (on_this_side / (on_this_side - on_that_side))));
        }
    }

    kept
}

/// The unsigned area of the polygon (a fan from its first point).
pub fn area_of(polygon: &[Vector3]) -> f64 {
    let mut twice = Vector3::ZERO;

    for i in 1..polygon.len().saturating_sub(1) {
        twice += (polygon[i] - polygon[0]).cross(polygon[i + 1] - polygon[0]);
    }

    twice.length() / 2.0
}

/// C# `TryGainIt`: the section, which the end face does not hold, laid as the cap that closes the
/// strip's end - [`surface_end_notch::try_cap_with_the_section`].
pub fn try_gain_it(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    profile: &[i32],
    corner: Vector3,
    plane_normal: Vector3,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    surface_end_notch::try_cap_with_the_section(pool, faces, profile, corner, plane_normal)
}
