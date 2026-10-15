//! How far a sweep runs STRAIGHT on - segments carrying one section along one line - and the rings
//! an end cap has already passed on such a run.

use std::collections::HashSet;

use vector_math::{ieee_math, math_helper, Vector3};

use super::bevel_chain_sweep_mesh::SweptSegment;
use super::bevel_chain_sweep_rings::profile_deviation;
use super::BevelChainEnds;
use crate::topology::FeatureEdgeChain;

/// How far two segments' profiles may differ, relative to the profile's own reach, and still be
/// one section: the profiles are built from face normals off FLOAT mesh coordinates, which agree
/// to about a ten-millionth.
const SAME_PROFILE_FRACTION: f64 = 1e-6;

/// How near a ring has to stand to the cap's reach to count as passed.
pub const TOUCHING_RING: f64 = 1e-9;

/// The length of the straight run through segment `index`, walking `forward` (or back) for as
/// long as each next segment carries straight on.
pub fn straight_run_from(swept: &[SweptSegment], index: usize, forward: bool) -> f64 {
    let mut run = swept[index].length;
    let step: i64 = if forward { 1 } else { -1 };

    let mut i = index as i64 + step;
    while i >= 0 && (i as usize) < swept.len() {
        if !carries_straight_on(&swept[(i - step) as usize], &swept[i as usize]) {
            break;
        }

        run += swept[i as usize].length;
        i += step;
    }

    run
}

/// Whether `after` carries `before`'s section straight on: direction and both frame axes within
/// the collinear angle, and the profiles one to [`SAME_PROFILE_FRACTION`] of their reach.
pub fn carries_straight_on(before: &SweptSegment, after: &SweptSegment) -> bool {
    let straight =
        math_helper::degrees_to_radians(FeatureEdgeChain::default_collinear_angle_degrees()).cos();

    if before.direction.dot(after.direction) < straight
        || before.x_axis.dot(after.x_axis) < straight
        || before.y_axis.dot(after.y_axis) < straight
    {
        return false;
    }

    let largest = before.profile.iter().fold(0.0, |largest, point| {
        ieee_math::max_f64(largest, point.length())
    });

    profile_deviation(before, after) <= largest * SAME_PROFILE_FRACTION
}

/// `rings` without those a capped end has already passed: from a capped start (or end), each ring
/// of the straight run that stands within the cap's furthest slide of that end.
pub fn drop_rings_the_cap_has_passed(
    rings: Vec<Vec<Vector3>>,
    swept: &[SweptSegment],
    ends: &BevelChainEnds,
) -> Vec<Vec<Vector3>> {
    let mut passed: HashSet<usize> = HashSet::new();

    if ends.start.is_capped() {
        let reach = enumerable_max(swept[0].start_slide.as_deref().unwrap_or_default());

        for ring in 1..swept.len() {
            if !carries_straight_on(&swept[ring - 1], &swept[ring]) {
                break;
            }

            if run_to_ring(swept, ring) - reach > TOUCHING_RING {
                break;
            }

            passed.insert(ring);
        }
    }

    if ends.end.is_capped() {
        let reach = -enumerable_min(
            swept[swept.len() - 1]
                .end_slide
                .as_deref()
                .unwrap_or_default(),
        );
        let whole_run = run_to_ring(swept, swept.len());

        for ring in (1..swept.len()).rev() {
            if !carries_straight_on(&swept[ring - 1], &swept[ring]) {
                break;
            }

            if whole_run - run_to_ring(swept, ring) - reach > TOUCHING_RING {
                break;
            }

            passed.insert(ring);
        }
    }

    if passed.is_empty() {
        return rings;
    }

    rings
        .into_iter()
        .enumerate()
        .filter(|(ring, _)| !passed.contains(ring))
        .map(|(_, kept)| kept)
        .collect()
}

/// How far along the run ring `ring` stands: the segments before it, added up.
pub fn run_to_ring(swept: &[SweptSegment], ring: usize) -> f64 {
    swept[..ring].iter().map(|segment| segment.length).sum()
}

/// The largest value: NaNs are passed over unless every value is one. A capped end always has its
/// slides; an empty list reads as no reach.
fn enumerable_max(values: &[f64]) -> f64 {
    let mut iter = values.iter().copied().skip_while(|v| v.is_nan());
    let Some(mut value) = iter.next() else {
        return values.first().copied().unwrap_or(0.0);
    };
    for x in iter {
        if x > value {
            value = x;
        }
    }
    value
}

/// The smallest value: any NaN is the answer.
fn enumerable_min(values: &[f64]) -> f64 {
    let mut value = match values.first() {
        Some(&first) => first,
        None => return 0.0,
    };
    for &x in values {
        if x.is_nan() {
            return x;
        }
        if x < value {
            value = x;
        }
    }
    value
}
