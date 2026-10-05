//! The extra rings a TOOL drives through past a run's end where that end overshoots onto a plane
//! of its own (`Bevel/BevelOvershootRings.cs`, a static class: free functions here).

use vector_math::{Vector2, Vector3};

use super::bevel_chain_sweep_mesh::SweptSegment;
use super::bevel_corner_seam::CAP_TAPER;
use super::{BevelChainEnds, BevelEndCap, BevelSweptSection};

/// `rings` with a ring added before the first (and/or after the last) where that end of a TOOL
/// sweep drives through to its overshoot plane; unchanged for any other section or where neither
/// end drives through.
pub fn add(
    rings: Vec<Vec<Vector3>>,
    first: &SweptSegment,
    last: &SweptSegment,
    ends: &BevelChainEnds,
    point_count: usize,
    section: BevelSweptSection,
) -> Vec<Vec<Vector3>> {
    if section != BevelSweptSection::Tool
        || (!ends.start.drives_through && !ends.end.drives_through)
    {
        return rings;
    }

    let mut built = Vec::with_capacity(rings.len() + 2);

    if ends.start.drives_through {
        built.push(ring(first, first.from, &ends.start, point_count));
    }

    built.extend(rings);

    if ends.end.drives_through {
        built.push(ring(last, last.to, &ends.end, point_count));
    }

    built
}

/// The ring `segment`'s profile (its clearance points tapered by [`CAP_TAPER`]) makes on the
/// overshoot's own plane, slid there along the segment from `anchor`.
pub fn ring(
    segment: &SweptSegment,
    anchor: Vector3,
    cap: &BevelEndCap,
    point_count: usize,
) -> Vec<Vector3> {
    // The same plane arithmetic the run's own ring uses, read against the overshoot's plane.
    let far = BevelEndCap::new(cap.plane_normal, cap.drive_through_distance);

    (0..point_count)
        .map(|point| {
            // Tapered offset for the slide as well as for the position: the ring has to LIE on
            // that plane, and the plane is reached from wherever the point actually is.
            let offset = tapered_offset(segment, point, CAP_TAPER);

            anchor
                + (segment.direction * far.slide_to_plane(anchor, offset, segment.direction))
                + offset
        })
        .collect()
}

/// Profile point `point`'s offset, with the three clearance points (1, 2, 3 of the original
/// five-point profile, whichever way its winding was normalised) pulled `fraction` of the way in.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn tapered_offset(segment: &SweptSegment, point: usize, fraction: f64) -> Vector3 {
    let profile = &segment.profile;
    let count = profile.len();
    let original_point = if segment.profile_reversed {
        count as i64 - 1 - point as i64
    } else {
        point as i64
    };

    if !(fraction > 0.0) || count < 5 || !(1..=3).contains(&original_point) {
        return segment.offset(point);
    }

    // Winding normalisation reverses the slots; only the original clearance points may move.
    let original = |index: usize| -> Vector2 {
        profile[if segment.profile_reversed {
            count - 1 - index
        } else {
            index
        }]
    };
    let tapered = match original_point {
        1 => original(1) - ((original(1) - original(0)) * fraction),
        3 => original(3) - ((original(3) - original(4)) * fraction),
        _ => original(2) * (1.0 - fraction),
    };

    (segment.x_axis * tapered.x) + (segment.y_axis * tapered.y)
}
