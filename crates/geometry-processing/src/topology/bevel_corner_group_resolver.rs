//! Making [`BevelCornerGroup`] anchors, and finding them again on a path that has been rebuilt,
//! renumbered or parametrically changed ([`BevelCornerGroupResolution`]).
//!
//! The 2D reading of [`super::bevel_edge_group_resolver`] and the same three stages: hard gates
//! (drift within a fraction of the path's extent, both edge directions within a tolerance under
//! either pairing), a score among the survivors with each term divided by its own gate, then a
//! check that the winner actually won, so repeated geometry (a row of teeth) whose anchored corner
//! was deleted is refused rather than resolved onto a sibling.

use vector_math::{ieee_math, math_helper};

use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

use super::bevel_corner_group::BevelCornerGroup;
use super::error::TopologyError;
use super::path_corner_set::{PathCorner, PathCornerSet};

/// How far either of the corner's two edges may have turned from the stored direction, degrees.
pub const DEFAULT_DIRECTION_TOLERANCE_DEGREES: f64 = 20.0;
/// How far the corner may have drifted, as a fraction of the path's bounding diagonal.
pub const DEFAULT_DRIFT_FRACTION_OF_EXTENT: f64 = 0.25;
/// A best score at or above this fraction of the runner-up's is two matches, not one.
pub const DEFAULT_AMBIGUITY_RATIO: f64 = 0.8;

/// What a stored [`BevelCornerGroup`] came to on one path.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelCornerGroupResolution {
    /// The group that was resolved.
    pub group: BevelCornerGroup,
    /// Why the group found nothing; `None` when it resolved.
    pub failure_reason: Option<BevelAdvisory>,
    /// The matched corner's index in the [`PathCornerSet`], or -1.
    pub corner_index: i32,
    /// The corner itself; meaningless unless resolved.
    pub corner: PathCorner,
}

impl BevelCornerGroupResolution {
    /// Whether the group found its corner on this path.
    pub fn is_resolved(&self) -> bool {
        self.failure_reason.is_none()
    }

    fn failed(group: &BevelCornerGroup, reason: BevelAdvisory) -> Self {
        BevelCornerGroupResolution {
            group: *group,
            failure_reason: Some(reason),
            corner_index: -1,
            corner: PathCorner::default(),
        }
    }

    fn succeeded(group: &BevelCornerGroup, corner_index: usize, corner: PathCorner) -> Self {
        BevelCornerGroupResolution {
            group: *group,
            failure_reason: None,
            corner_index: corner_index as i32,
            corner,
        }
    }
}

/// Snapshots a corner as the anchor of a new group. Errors where `corner_index` is not a corner
/// of the set.
pub fn create_from_corner(
    corners: &PathCornerSet,
    corner_index: i32,
) -> Result<BevelCornerGroup, TopologyError> {
    if corner_index < 0 || corner_index as usize >= corners.count() {
        return Err(TopologyError::CornerIndexOutOfRange {
            index: corner_index,
            corner_count: corners.count(),
        });
    }

    let corner = corners.corners[corner_index as usize];
    Ok(BevelCornerGroup {
        anchor_position: corner.position,
        anchor_to_previous: corner.to_previous,
        anchor_to_next: corner.to_next,
    })
}

/// Finds a stored group's corner on a path at the default tolerances.
pub fn resolve(group: &BevelCornerGroup, corners: &PathCornerSet) -> BevelCornerGroupResolution {
    resolve_with(
        group,
        corners,
        DEFAULT_DIRECTION_TOLERANCE_DEGREES,
        DEFAULT_DRIFT_FRACTION_OF_EXTENT,
        DEFAULT_AMBIGUITY_RATIO,
    )
}

/// Finds a stored group's corner on a path, with explicit tolerances.
pub fn resolve_with(
    group: &BevelCornerGroup,
    corners: &PathCornerSet,
    direction_tolerance_degrees: f64,
    drift_fraction_of_extent: f64,
    ambiguity_ratio: f64,
) -> BevelCornerGroupResolution {
    if corners.count() == 0 {
        return BevelCornerGroupResolution::failed(
            group,
            BevelAdvisory::new(
                BevelAdvisoryCode::PathHasNoCorners,
                vec![corners.bend_threshold_degrees.into()],
            ),
        );
    }

    if group.anchor_to_previous.length_squared() <= 0.0
        || group.anchor_to_next.length_squared() <= 0.0
    {
        return BevelCornerGroupResolution::failed(
            group,
            BevelAdvisory::new(BevelAdvisoryCode::GroupHasNoAnchorDirections, Vec::new()),
        );
    }

    let to_previous = group.anchor_to_previous.get_normal();
    let to_next = group.anchor_to_next.get_normal();
    let maximum_drift = corners.extent * drift_fraction_of_extent;
    let minimum_agreement = math_helper::degrees_to_radians(direction_tolerance_degrees).cos();

    // each term divided by the most it can be at its own gate, so the equal weights are equal
    let worst_direction_term = 1.0 - minimum_agreement;

    let mut best_score = f64::MAX;
    let mut runner_up_score = f64::MAX;
    let mut best_index: Option<usize> = None;

    for (i, candidate) in corners.corners.iter().enumerate() {
        let drift = (candidate.position - group.anchor_position).length();

        if drift > maximum_drift {
            continue;
        }

        // both pairings, because a contour wound the other way swaps the two directions; the
        // worse edge within a pairing is what scores
        let straight_pairing = ieee_math::min_f64(
            candidate.to_previous.dot(to_previous),
            candidate.to_next.dot(to_next),
        );
        let swapped_pairing = ieee_math::min_f64(
            candidate.to_next.dot(to_previous),
            candidate.to_previous.dot(to_next),
        );
        let agreement = ieee_math::max_f64(straight_pairing, swapped_pairing);

        if agreement < minimum_agreement {
            continue;
        }

        let score = term(drift, maximum_drift) + term(1.0 - agreement, worst_direction_term);

        if score < best_score {
            runner_up_score = best_score;
            best_score = score;
            best_index = Some(i);
        } else if score < runner_up_score {
            runner_up_score = score;
        }
    }

    let Some(best_index) = best_index else {
        return BevelCornerGroupResolution::failed(
            group,
            BevelAdvisory::new(
                BevelAdvisoryCode::NoCornerMatchesAnchor,
                vec![
                    group.anchor_position.into(),
                    (corners.count() as i32).into(),
                    maximum_drift.into(),
                    direction_tolerance_degrees.into(),
                ],
            ),
        );
    };

    // per-candidate gates cannot see repeated geometry: two siblings pass equally well
    if runner_up_score < f64::MAX && best_score >= runner_up_score * ambiguity_ratio {
        return BevelCornerGroupResolution::failed(
            group,
            BevelAdvisory::new(
                BevelAdvisoryCode::AnchorMatchesMoreThanOneCorner,
                vec![
                    group.anchor_position.into(),
                    best_score.into(),
                    runner_up_score.into(),
                ],
            ),
        );
    }

    BevelCornerGroupResolution::succeeded(group, best_index, corners.corners[best_index])
}

/// Finds every group's corner on a path; groups resolve independently.
pub fn resolve_all(
    groups: &[BevelCornerGroup],
    corners: &PathCornerSet,
) -> Vec<BevelCornerGroupResolution> {
    groups.iter().map(|group| resolve(group, corners)).collect()
}

/// One score term as a fraction of the most it could have been and still passed its gate.
fn term(value: f64, worst_allowed: f64) -> f64 {
    if worst_allowed > 0.0 {
        value / worst_allowed
    } else {
        0.0
    }
}
