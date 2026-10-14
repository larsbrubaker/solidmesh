//! The corner where THREE CHAMFER strips meet at one vertex of the part: the three strip planes
//! meet at one point, and each strip ends in a bent ring that runs to it. Free functions. The
//! arriving strips are [`SurfaceCornerPatchArrival`]s, as at a round corner
//! ([`super::surface_corner_patch`]); what is handed back is each strip's end station
//! ([`SurfaceStripStation`]), with the meeting point standing between its two rails.

use vector_math::Vector3;

use super::{SurfaceCornerPatchArrival, SurfaceStrip, SurfaceStripStation, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How far outside its own band across a strip the meeting point may fall and still be read as
/// within it - a thousandth of the band, which is far under the width of any chord the strip is
/// drawn with and far over the rounding of a plane crossing.
const WITHIN_THE_BAND: f64 = 1e-3;

/// Closes a three-chamfer corner - finds the point the three strip planes meet at and ends each
/// strip in the bent ring that runs to it. `Ok` carries each arrival's end station in the arrivals'
/// own order; `Err` is why the corner would not close. The meeting point is added to `pool` once,
/// and shared.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_close(
    pool: &mut SurfaceVertexPool,
    arrivals: &[SurfaceCornerPatchArrival],
    corner: Vector3,
) -> Result<Vec<SurfaceStripStation>, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.into()],
        )
    };

    let mut points = Vec::with_capacity(arrivals.len());
    let mut normals = Vec::with_capacity(arrivals.len());

    for arrival in arrivals {
        if arrival.strip.rail_count() != 2 {
            // A profile with an arc in it is a patch's corner, not a mitre's.
            return Err(not_supported());
        }

        let point = arrival.strip.point_on(0, 0.0);
        let across = arrival.strip.point_on(1, 0.0) - point;
        let normal = across.cross(arrival.strip.along());

        if !(normal.length_squared() > 0.0) {
            return Err(not_supported());
        }

        points.push(point);
        normals.push(normal.get_normal());
    }

    let mut rail_ends = Vec::with_capacity(arrivals.len() * 2);

    for arrival in arrivals {
        rail_ends.push(arrival.on_the_left);
        rail_ends.push(arrival.on_the_right);
    }

    if !rail_ends_pair_up(&rail_ends) {
        // The three bent ends would not chain into one rim: some end of some strip stands where no
        // neighbour's does, so whatever is built here is sewn to itself along only part of its seam.
        return Err(not_supported());
    }

    let Some(meeting) = try_find_where_they_meet(&points, &normals) else {
        // Two of the three planes parallel, or all three through one line: there is no one point for
        // the corner to close on.
        return Err(not_supported());
    };

    let meeting_vertex = pool.add(meeting);

    let mut stations = Vec::with_capacity(arrivals.len());

    for arrival in arrivals {
        stations.push(try_end_the_strip(
            pool,
            arrival,
            meeting,
            meeting_vertex,
            corner,
        )?);
    }

    Ok(stations)
}

/// Whether the strips' rail ends are vertices shared two at a time, each strip
/// leaving off where a neighbour takes up - given as each strip's left end followed by its right.
///
/// The round path has had this check from the start: `SurfaceCornerPatch`'s `TryOrderTheSides`
/// chains its three end rings by VERTEX INDEX and refuses three that will not chain. The mitre needs
/// the same thing for the same reason - the ends are the walkback's own vertices and the bent rings
/// are drawn on them, so two ends at one place under two indices are a crack down the middle of the
/// seam. Index equality says so exactly; nearness would not, being no transitive thing to group by.
pub fn rail_ends_pair_up(rail_ends: &[i32]) -> bool {
    let mut at = 0;
    while at + 1 < rail_ends.len() {
        if rail_ends[at] == rail_ends[at + 1] {
            // One strip's own two ends are one vertex: a strip pinched shut, which pairs up by count
            // and closes nothing.
            return false;
        }

        at += 2;
    }

    for &end in rail_ends {
        let standing = rail_ends.iter().filter(|&&other| other == end).count();

        if standing != 2 {
            return false;
        }
    }

    true
}

/// The one point on all three planes - Cramer over the three normals,
/// which is the same solve `SurfaceCornerPatch` does for a ball centre and fails the same way when
/// the three planes have no point between them.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn try_find_where_they_meet(points: &[Vector3], normals: &[Vector3]) -> Option<Vector3> {
    let determinant = normals[0].dot(normals[1].cross(normals[2]));

    if !(determinant.abs() > 1e-6) {
        return None;
    }

    let wanted = Vector3::new(
        normals[0].dot(points[0]),
        normals[1].dot(points[1]),
        normals[2].dot(points[2]),
    );

    let first = normals[1].cross(normals[2]);
    let second = normals[2].cross(normals[0]);
    let third = normals[0].cross(normals[1]);

    Some(((first * wanted.x) + (second * wanted.y) + (third * wanted.z)) / determinant)
}

/// One strip's bent end - its two rails stop at the vertices the walkback left
/// on their own tangent lines, which are where the two neighbouring planes cross those rails, and
/// the meeting point stands between them as a point the stitch sews the last quad to.
///
/// The rails are not crossed with anything here. The end of rail 0 is the point where the neighbour
/// sharing that face cuts it, and the walkback has already put a vertex there. Reading it off that
/// vertex rather than solving for it again is the construction's own no-crack rule - and it is
/// CHECKED against the rail, so a corner where those two are not the same point is a refusal rather
/// than a seam a rounding apart.
fn try_end_the_strip(
    pool: &SurfaceVertexPool,
    arrival: &SurfaceCornerPatchArrival,
    meeting: Vector3,
    meeting_vertex: i32,
    corner: Vector3,
) -> Result<SurfaceStripStation, BevelAdvisory> {
    let rail = vec![arrival.on_the_left, arrival.on_the_right];
    let mut param = vec![0.0; 2];

    for side in 0..2 {
        let standing = pool.position_of(rail[side]);

        param[side] = arrival.strip.param_of(standing);

        if (arrival.strip.point_on(side as i32, param[side]) - standing).length()
            > pool.near_enough()
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![standing.into()],
            ));
        }
    }

    if !is_within_the_band(&arrival.strip, meeting, pool.near_enough()) {
        // The three planes meet somewhere this strip does not reach - unequal setbacks, or a corner
        // whose faces lean so far that one strip's band is crossed outside itself. The bent end would
        // run off the side of the strip it belongs to, so there is no mitre here to build.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.into()],
        ));
    }

    Ok(SurfaceStripStation::new(
        rail,
        param,
        vec![vec![meeting_vertex]],
    ))
}

/// Whether a point stands on the strip's plane and between its two rails -
/// taken in the strip's own frame, across and along, which are independent but not square to each
/// other on an oblique corner.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn is_within_the_band(strip: &SurfaceStrip, point: Vector3, near_enough: f64) -> bool {
    let origin = strip.point_on(0, 0.0);
    let across = strip.point_on(1, 0.0) - origin;
    let along = strip.along();
    let reaching = point - origin;

    let squared = across.dot(across);
    let leaning = across.dot(along);
    let determinant = (squared * along.dot(along)) - (leaning * leaning);

    if !(determinant > 0.0) {
        return false;
    }

    let over =
        ((reaching.dot(across) * along.dot(along)) - (leaning * reaching.dot(along))) / determinant;
    let down = ((squared * reaching.dot(along)) - (leaning * reaching.dot(across))) / determinant;

    if (reaching - (across * over) - (along * down)).length() > near_enough {
        // Off the strip's own plane altogether, which the three-plane solve should make impossible -
        // and does, so this is the arithmetic saying so rather than a case to handle.
        return false;
    }

    over > -WITHIN_THE_BAND && over < 1.0 + WITHIN_THE_BAND
}
