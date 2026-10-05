//! The RINGS half of the chain sweep (`Bevel/BevelChainSweep.Rings.cs`, part of the C#
//! `internal static partial class BevelChainSweep`): carrying a corner's seam onto an end ring
//! ([`carry`]), the mitred ring at a joint ([`try_build_joint`]) and how far two segments'
//! profiles differ ([`profile_deviation`]). Building the run's rings lands with a later package.

use vector_math::{cs_math, Vector3};

use super::bevel_chain_sweep::MINIMUM_MITER_COSINE;
use super::bevel_chain_sweep_mesh::SweptSegment;
use super::bevel_seam_ring;
use super::{BevelAdvisory, BevelAdvisoryCode, BevelEndCap};

/// Snaps an end `ring` onto the seam the corner patch at `cap` names, where it names one; a seam
/// the ring cannot be snapped onto within `size` adds a "corner seam not carried" warning at `at`
/// (where `warnings` is kept).
pub fn carry(
    ring: &mut [Vector3],
    cap: &BevelEndCap,
    at: Vector3,
    size: f64,
    warnings: Option<&mut Vec<BevelAdvisory>>,
) {
    let Some(seam) = &cap.seam_points else {
        return;
    };

    if !bevel_seam_ring::snap(ring, seam, size) {
        if let Some(warnings) = warnings {
            warnings.push(BevelAdvisory::new(
                BevelAdvisoryCode::CornerSeamNotCarried,
                vec![at.into()],
            ));
        }
    }
}

/// The farthest any profile point of `after` stands from the same point of `before`, or infinity
/// where the two have different point counts.
pub fn profile_deviation(before: &SweptSegment, after: &SweptSegment) -> f64 {
    if before.profile.len() != after.profile.len() {
        return f64::INFINITY;
    }

    before
        .profile
        .iter()
        .zip(&after.profile)
        .fold(0.0, |largest, (&b, &a)| {
            cs_math::max_f64(largest, (b - a).length())
        })
}

/// The ring at the joint between `before` and `after`: the profile (the two segments' averaged)
/// placed on the MITRE plane, in the two frames taken half way round the turn and stretched along
/// the turn by `1 / cos(half turn) - 1`. Records how far reaching the joint drags each strip along
/// its own segment (`before`'s end slide, `after`'s start slide; negative shortens the strip,
/// which is the fold when it outruns the segment). Refused where the chain doubles back, turns
/// too sharply to mitre, or leaves no plane at the joint.
pub fn try_build_joint(
    before: &mut SweptSegment,
    after: &mut SweptSegment,
    point_count: usize,
) -> Result<Vec<Vector3>, BevelAdvisory> {
    let refused = |code: BevelAdvisoryCode| BevelAdvisory::new(code, vec![after.from.into()]);

    let miter_normal = before.direction + after.direction;

    if miter_normal.length() < 1e-9 {
        return Err(refused(BevelAdvisoryCode::ChainDoublesBack));
    }

    let miter_normal = miter_normal.get_normal();

    let cosine_of_half_turn = before.direction.dot(miter_normal);

    if cosine_of_half_turn < MINIMUM_MITER_COSINE {
        return Err(refused(BevelAdvisoryCode::ChainTurnsTooSharply));
    }

    // The direction inside the miter plane that also lies in the plane of the turn - the only
    // direction the joint stretches along.
    let mut turn_axis = before.direction.cross(after.direction);
    let turns = turn_axis.length() > 1e-9;

    if turns {
        turn_axis = turn_axis.get_normal();
    }

    let in_turn_plane = if turns {
        turn_axis.cross(miter_normal).get_normal()
    } else {
        Vector3::ZERO
    };
    let stretch = (1.0 / cosine_of_half_turn) - 1.0;

    // The two frames taken half way round the turn, by averaging the axes - with the part
    // perpendicular to the turn axis divided back out by cos(half turn), so the axis stands
    // exactly half way round rather than leaning toward the turn axis.
    let mut across_axis = before.x_axis + after.x_axis;

    if turns {
        let along_turn_axis = turn_axis * across_axis.dot(turn_axis);

        across_axis =
            along_turn_axis + ((across_axis - along_turn_axis) * (1.0 / cosine_of_half_turn));
    }

    across_axis -= miter_normal * across_axis.dot(miter_normal);

    if across_axis.length() < 1e-9 {
        across_axis = before.x_axis - (miter_normal * before.x_axis.dot(miter_normal));

        if across_axis.length() < 1e-9 {
            return Err(refused(BevelAdvisoryCode::NoPlaneAtJoint));
        }
    }

    let across_axis = across_axis.get_normal();

    let along_axis = miter_normal.cross(across_axis).get_normal();

    let mut ring = Vec::with_capacity(point_count);

    for point in 0..point_count {
        let averaged = (before.profile[point] + after.profile[point]) * 0.5;
        let mut placed = (across_axis * averaged.x) + (along_axis * averaged.y);

        if turns {
            placed += in_turn_plane * (placed.dot(in_turn_plane) * stretch);
        }

        ring.push(after.from + placed);

        // Reaching the joint drags each strip along its own segment by this much. The caller
        // sizes both slides before the joints are built (C# reads them unguarded).
        let end_slide = (placed - before.offset(point)).dot(before.direction);
        let start_slide = (placed - after.offset(point)).dot(after.direction);
        before
            .end_slide
            .get_or_insert_with(|| vec![0.0; point_count])[point] = end_slide;
        after
            .start_slide
            .get_or_insert_with(|| vec![0.0; point_count])[point] = start_slide;
    }

    Ok(ring)
}
