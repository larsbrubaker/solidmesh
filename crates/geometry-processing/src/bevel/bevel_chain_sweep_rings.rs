//! The RINGS half of the chain sweep: carrying a corner's seam onto an end ring ([`carry`]), the
//! mitred ring at a joint ([`try_build_joint`]), every ring of a run ([`try_build_rings`]) and how
//! far two segments' profiles differ ([`profile_deviation`]).

use vector_math::{ieee_math, Vector3};

use super::bevel_chain_sweep::MINIMUM_MITER_COSINE;
use super::bevel_chain_sweep_mesh::SweptSegment;
use super::bevel_edge_tool_builder;
use super::bevel_seam_ring;
use super::{BevelAdvisory, BevelAdvisoryCode, BevelChainEnds, BevelEndCap, BevelSweptSection};

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
            ieee_math::max_f64(largest, (b - a).length())
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

        // Reaching the joint drags each strip along its own segment by this much. The caller sizes
        // both slides before the joints are built, so they are read unguarded here.
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

/// Every ring of a run: one per joint (mitred, [`try_build_joint`]) and, on an open run, one at
/// each free end - slid onto the end's cap plane where it is capped, and run past by the overshoot
/// (bounded by what the ends allow, floored by the caller's minimum) where it is not. Each slide
/// is recorded on its segment, which is what the fold test reads. A TOOL's end rings take the
/// corner patch's own seam where one is offered ([`carry`]).
#[allow(clippy::too_many_arguments)]
pub fn try_build_rings(
    swept: &mut [SweptSegment],
    closed: bool,
    ends: &BevelChainEnds,
    size: f64,
    point_count: usize,
    start_minimum_overshoot: f64,
    end_minimum_overshoot: f64,
    section: BevelSweptSection,
    mut warnings: Option<&mut Vec<BevelAdvisory>>,
) -> Result<Vec<Vec<Vector3>>, BevelAdvisory> {
    let segment_count = swept.len();

    let mut rings = vec![
        Vec::new();
        if closed {
            segment_count
        } else {
            segment_count + 1
        }
    ];

    for segment in swept.iter_mut() {
        segment.start_slide = Some(vec![0.0; point_count]);
        segment.end_slide = Some(vec![0.0; point_count]);
    }

    let first_joint = if closed { 0 } else { 1 };

    for (ring, slot) in rings
        .iter_mut()
        .enumerate()
        .take(segment_count)
        .skip(first_joint)
    {
        *slot = try_build_joint_at(swept, ring, point_count)?;
    }

    if !closed {
        let last_index = segment_count - 1;

        // Bounded by what the ends allow, floored by what the caller sweeping a ball asks for.
        let start_overshoot = ieee_math::max_f64(
            start_minimum_overshoot,
            ieee_math::min_f64(
                bevel_edge_tool_builder::overshoot(size, swept[0].tangent_distance),
                ends.start_maximum_overshoot,
            ),
        );
        let end_overshoot = ieee_math::max_f64(
            end_minimum_overshoot,
            ieee_math::min_f64(
                bevel_edge_tool_builder::overshoot(size, swept[last_index].tangent_distance),
                ends.end_maximum_overshoot,
            ),
        );

        let mut start_ring = Vec::with_capacity(point_count);
        let mut end_ring = Vec::with_capacity(point_count);

        for point in 0..point_count {
            let first = &swept[0];
            let last = &swept[last_index];
            let start_offset = first.offset(point);
            let end_offset = last.offset(point);

            let start_slide = if ends.start.is_capped() {
                ends.start
                    .slide_to_plane(first.from, start_offset, first.direction)
            } else {
                -start_overshoot
            };

            let end_slide = if ends.end.is_capped() {
                ends.end.slide_to_plane(last.to, end_offset, last.direction)
            } else {
                end_overshoot
            };

            start_ring.push(first.from + (first.direction * start_slide) + start_offset);
            end_ring.push(last.to + (last.direction * end_slide) + end_offset);

            if let Some(slides) = swept[0].start_slide.as_mut() {
                slides[point] = start_slide;
            }
            if let Some(slides) = swept[last_index].end_slide.as_mut() {
                slides[point] = end_slide;
            }
        }

        // Where a corner patch owns the end, the arc on that plane is the patch's own points: the
        // seam is carried, not computed twice. Only the TOOL has that arc.
        if section == BevelSweptSection::Tool {
            carry(
                &mut start_ring,
                &ends.start,
                swept[0].from,
                size,
                warnings.as_deref_mut(),
            );
            carry(
                &mut end_ring,
                &ends.end,
                swept[last_index].to,
                size,
                warnings,
            );
        }

        rings[0] = start_ring;
        rings[segment_count] = end_ring;
    }

    Ok(rings)
}

/// [`try_build_joint`] at joint `joint` of `swept` - between segment `joint - 1` (the last, for
/// joint 0 of a closed run) and segment `joint`.
pub fn try_build_joint_at(
    swept: &mut [SweptSegment],
    joint: usize,
    point_count: usize,
) -> Result<Vec<Vector3>, BevelAdvisory> {
    let count = swept.len();
    let before_index = (joint + count - 1) % count;

    if before_index < joint {
        let (head, tail) = swept.split_at_mut(joint);
        try_build_joint(&mut head[before_index], &mut tail[0], point_count)
    } else {
        let (head, tail) = swept.split_at_mut(before_index);
        try_build_joint(&mut tail[0], &mut head[joint], point_count)
    }
}
