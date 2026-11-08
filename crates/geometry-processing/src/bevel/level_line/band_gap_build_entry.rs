//! Entering the band gap's build: the pick's arguments checked, step 1's setback readings made
//! (`band_gap_setback_fields.rs`), and the pass handed to the build proper
//! ([`build_from_the_setbacks`], `band_gap_build_within.rs`). The funnel
//! ([`band_gap::build_unvalidated`](super::band_gap::build_unvalidated)) calls [`build_within`].

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::band_gap::{BandGapBuildError, BuildWithinArgs};
use super::band_gap_build_fault::BuildFault;
use super::band_gap_build_within::build_from_the_setbacks;
use super::band_gap_setback_fields::PassSetbacks;
use super::pick_validation;
use super::seam_edges::FeatureEdgeTest;
use super::{BandGapResult, LevelLineMesh};

/// The build proper. `working` receives the working mesh as soon as it is made, so a refusal out
/// of the middle of the build can still hand it back; `unflowed_room` receives the plain
/// tagging's room where the flowed tagging is cut instead (see
/// [`tag_flowed_where_it_builds`](super::narrow_face_flow::tag_flowed_where_it_builds)).
#[allow(clippy::too_many_arguments)]
pub fn build_within(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    args: &BuildWithinArgs,
    working: &mut Option<Rc<RefCell<LevelLineMesh>>>,
    unflowed_room: &mut Option<f64>,
) -> Result<BandGapResult, BuildFault> {
    let BuildWithinArgs {
        size,
        profile,
        sharp_turn_degrees,
        ..
    } = *args;

    if chains.iter().any(|one| one.len() < 2) {
        return Err(BuildFault::Error(BandGapBuildError::Arguments(
            pick_validation::PickArgumentError::ChainTooShort,
        )));
    }

    if size.is_nan() || size <= 0.0 {
        return Err(BuildFault::Error(BandGapBuildError::SizeNotPositive(size)));
    }

    // 1. SETBACK, per chain.
    let no_feature_edges = |_: i32, _: i32| false;
    let feature: &dyn Fn(i32, i32) -> bool = match &is_feature_edge {
        Some(test) => test.as_ref(),
        None => &no_feature_edges,
    };
    let setbacks = PassSetbacks::of(
        positions,
        indices,
        chains,
        closed,
        sharp_turn_degrees,
        profile,
        feature,
    )?;
    build_from_the_setbacks(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        args,
        working,
        unflowed_room,
        &setbacks,
    )
}
