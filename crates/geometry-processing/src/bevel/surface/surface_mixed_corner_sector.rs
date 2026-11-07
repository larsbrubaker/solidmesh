//! Where a mixed corner's cap sector closes: the apex the cap's fan is drawn from - the outer
//! rail's centre, or, where the corner hands over at a mitre, the crossing of the handover plane's
//! trace with the cap's radius.

use vector_math::Vector3;

use super::SurfaceMixedCorner;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How near the axis, relative to the radius, a crossing is taken as the axis.
pub const ON_THE_AXIS: f64 = 1e-9;

/// The crossing of the radius through the last station on the cap with the trace of the plane the
/// ring is cut at, through the ring's own cap vertex. A crossing within [`ON_THE_AXIS`] of the axis
/// is the axis itself.
pub fn try_apex(
    corner: &SurfaceMixedCorner,
    ring_on_the_cap: Vector3,
    last_on_the_cap: Vector3,
) -> Result<Vector3, BevelAdvisory> {
    let along_the_radius = last_on_the_cap - corner.outer_rail_centre;

    let apex = try_cross(
        corner,
        corner.outer_rail_centre,
        along_the_radius,
        ring_on_the_cap,
        the_trace(corner),
    )?;

    if (apex - corner.outer_rail_centre).length_squared()
        <= ON_THE_AXIS * ON_THE_AXIS * along_the_radius.length_squared()
    {
        // The trace runs through the axis - the ordinary corner - so the crossing IS the axis,
        // taken as itself rather than as arithmetic that lands a hair off it.
        return Ok(corner.outer_rail_centre);
    }

    Ok(apex)
}

/// The apex of a sector between two corners - the first corner's rail centre unless either hands
/// over at a mitre, then the crossing of each mitre's trace (or the radius, for the one that does
/// not).
pub fn try_apex_between(
    first: &SurfaceMixedCorner,
    last: &SurfaceMixedCorner,
    first_on_the_cap: Vector3,
    last_on_the_cap: Vector3,
) -> Result<Vector3, BevelAdvisory> {
    if !first.hands_over_at_a_mitre && !last.hands_over_at_a_mitre {
        return Ok(first.outer_rail_centre);
    }

    try_cross(
        first,
        if first.hands_over_at_a_mitre {
            first_on_the_cap
        } else {
            first.outer_rail_centre
        },
        if first.hands_over_at_a_mitre {
            the_trace(first)
        } else {
            first_on_the_cap - first.outer_rail_centre
        },
        if last.hands_over_at_a_mitre {
            last_on_the_cap
        } else {
            first.outer_rail_centre
        },
        if last.hands_over_at_a_mitre {
            the_trace(last)
        } else {
            last_on_the_cap - first.outer_rail_centre
        },
    )
}

/// The handover plane's trace on the cap face.
pub fn the_trace(corner: &SurfaceMixedCorner) -> Vector3 {
    corner.out_of_the_cap_face.cross(corner.cut_stops_at.inward)
}

/// Where the line from `from_here` along `along_here` crosses the one from `from_there` along
/// `along_there`, both in the cap's plane; `SurfaceCornerNotSupported` at the corner where they are
/// parallel.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_cross(
    corner: &SurfaceMixedCorner,
    from_here: Vector3,
    along_here: Vector3,
    from_there: Vector3,
    along_there: Vector3,
) -> Result<Vector3, BevelAdvisory> {
    let across = along_here
        .cross(along_there)
        .dot(corner.out_of_the_cap_face);

    if !(across.abs() > 1e-9) {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        ));
    }

    let how_far = (from_there - from_here)
        .cross(along_there)
        .dot(corner.out_of_the_cap_face)
        / across;

    Ok(from_here + (along_here * how_far))
}
