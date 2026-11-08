//! Which joints of a chain a single mitred sweep CANNOT carry ([`joints_the_mitre_cannot_fit`]) -
//! where the run is split so each piece sweeps on its own.

use super::bevel_chain_sweep::MINIMUM_CLOSED_SEGMENTS;
use super::bevel_chain_sweep_rings::{profile_deviation, try_build_joint_at};
use super::bevel_chain_sweep_sections::try_build_segments;
use super::bevel_edge_tool_builder;
use crate::topology::ChainEdge;

/// How far two neighbouring sections may differ, as a share of the size, before the joint
/// between them is no mitre.
pub const SECTIONS_DIFFER_FRACTION: f64 = 0.25;

/// Per joint, whether the mitre cannot fit there: the joint cannot be built, the sections either
/// side differ by more than [`SECTIONS_DIFFER_FRACTION`] of the size, or a middle segment between
/// two good joints is dragged back by its joints further than it is long (charged to the joint
/// that drags it most). `None` where every joint fits, or the chain is too short to ask or cannot
/// be sectioned at all.
#[allow(clippy::needless_range_loop)] // index loops, read against two arrays.
pub fn joints_the_mitre_cannot_fit(
    chain: &[ChainEdge],
    is_closed: bool,
    size: f64,
    segments: i32,
    round_profile: bool,
    back_depth: f64,
) -> Option<Vec<bool>> {
    let least = if is_closed {
        MINIMUM_CLOSED_SEGMENTS as usize
    } else {
        3
    };
    if chain.len() < least || bevel_edge_tool_builder::try_validate_size(size).is_err() {
        return None;
    }

    let mut swept = try_build_segments(chain, size, segments, round_profile, back_depth).ok()?;

    let point_count = swept[0].profile.len();
    let segment_count = swept.len();
    let mut cannot_fit = vec![false; segment_count];

    for segment in swept.iter_mut() {
        segment.start_slide = Some(vec![0.0; point_count]);
        segment.end_slide = Some(vec![0.0; point_count]);
    }

    for joint in (if is_closed { 0 } else { 1 })..segment_count {
        if try_build_joint_at(&mut swept, joint, point_count).is_err() {
            cannot_fit[joint] = true;
        }

        let before = &swept[(joint + segment_count - 1) % segment_count];
        if profile_deviation(before, &swept[joint]) > size * SECTIONS_DIFFER_FRACTION {
            cannot_fit[joint] = true;
        }
    }

    for i in 0..segment_count {
        let start_joint = i;
        let end_joint = (i + 1) % segment_count;

        if !is_closed && (i == 0 || i == segment_count - 1) {
            continue;
        }

        if cannot_fit[start_joint] || cannot_fit[end_joint] {
            continue;
        }

        let mut worst_drag = 0.0;
        let mut from_start = 0.0;
        let mut from_end = 0.0;

        let start_slides = swept[i].start_slide.as_deref().unwrap_or_default();
        let end_slides = swept[i].end_slide.as_deref().unwrap_or_default();
        for point in 0..point_count {
            let drag = start_slides[point] - end_slides[point];

            if drag > worst_drag {
                worst_drag = drag;
                from_start = start_slides[point];
                from_end = -end_slides[point];
            }
        }

        if worst_drag >= swept[i].length {
            cannot_fit[if from_start >= from_end {
                start_joint
            } else {
                end_joint
            }] = true;
        }
    }

    cannot_fit.iter().any(|&c| c).then_some(cannot_fit)
}
