//! The band gap's public builds: the optional arguments are [`BandGapBuildOptions`] (whose
//! `Default` holds the usual values) and there are three functions - one chain
//! ([`build`]), a whole pick ([`build_pick`]) and one chain unvalidated with the topology-only
//! switch ([`build_one_unvalidated`]). The funnel they reach is
//! [`band_gap::build_unvalidated`](super::band_gap::build_unvalidated).
//!
//! A pick with an end face leaning off square (`square_crease_end_clip::a_clip_leans`, read as the
//! build reads its clips) is built twice: first with square clips only, whose answer stands where
//! it flows over a narrow end face, then with the leaning clips, each build with the whole work
//! allowance (the counter is put back between them, so the progress restarts with the second).
//! Where the leaning build refuses what the square one built, the square answer stands.

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::band_gap::{build_unvalidated, the_chains_as_built, BandGapBuildError};
use super::seam_edges::FeatureEdgeTest;
use super::{
    pick_validation, square_crease_end_clip, BandGapResult, BandProfile, LevelLineWork, WorkBudget,
};

/// The optional arguments of the public builds.
#[derive(Clone)]
pub struct BandGapBuildOptions {
    /// The chord tolerance; zero (the default) reads size / 20.
    pub chord_tolerance: f64,
    /// The snap (default 1e-9).
    pub snap: f64,
    /// A room read before, or `None`.
    pub known_room: Option<f64>,
    /// The work allowance, or `None` for the default one.
    pub budget: Option<Rc<WorkBudget>>,
    /// The seam epsilon (default 0).
    pub seam_epsilon: f64,
    /// The sharp turn, in degrees (default +infinity).
    pub sharp_turn_degrees: f64,
    /// The fillet's segment count (default 0).
    pub fillet_segments: i32,
}

impl Default for BandGapBuildOptions {
    fn default() -> Self {
        BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            budget: None,
            seam_epsilon: 0.0,
            sharp_turn_degrees: f64::INFINITY,
            fillet_segments: 0,
        }
    }
}

/// One chain built as a pick of one - one chain is a pick like any other, and takes the same
/// validation.
#[allow(clippy::too_many_arguments)]
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    chain_is_closed: bool,
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
) -> Result<BandGapResult, BandGapBuildError> {
    build_pick(
        positions,
        indices,
        &[chain.to_vec()],
        &[chain_is_closed],
        is_feature_edge,
        size,
        profile,
        options,
    )
}

/// Every
/// chain is read on the INPUT mesh, before anything is cut, so a pick that will not do is told so
/// while the mesh is still the one the user picked on.
#[allow(clippy::too_many_arguments)]
pub fn build_pick(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
) -> Result<BandGapResult, BandGapBuildError> {
    pick_validation::check_pick_arguments(positions, indices, chains, closed)?;
    if size.is_nan() || size <= 0.0 {
        return Err(BandGapBuildError::SizeNotPositive(size));
    }

    let budget = match &options.budget {
        Some(budget) => Rc::clone(budget),
        None => Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("the default limit is a positive count of work"),
        ),
    };
    if budget.cancellation_token.is_cancelled() {
        return Err(BandGapBuildError::Cancelled);
    }

    let (fault, _) = pick_validation::validate_on_the_input(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge.as_deref(),
    );
    if let Some(fault) = fault {
        return Ok(fault.as_band_gap(&budget));
    }

    let build = |budget: Rc<WorkBudget>| {
        build_unvalidated(
            positions,
            indices,
            chains,
            closed,
            is_feature_edge.clone(),
            size,
            profile,
            options.chord_tolerance,
            options.snap,
            options.known_room,
            Some(budget),
            false,
            options.seam_epsilon,
            options.sharp_turn_degrees,
            options.fillet_segments,
        )
    };

    // A leaning end face stops the round flush only where it holds the round's section. Where it
    // is narrower than this size's setback the narrow-face flow carries the round over it onto the
    // next face (`narrow_face_flow::takes_the_flow`), so the pick is first built as it would be
    // with square clips only: where that build takes the flow, it is the answer; otherwise the
    // leaning clips are read, and where they refuse what the square clips built, the square
    // build stands - a pick that built before the leaning clip never starts refusing. Each build
    // has the whole allowance: the work counter is put back between them.
    // The guard reads the pick as the build will: split at its setback steps, and at the
    // cove-to-round flips the build's budget names (`WorkBudget::sense_flips_to_cut`), so it is
    // read under that budget.
    let reading = LevelLineWork::begin(Some(Rc::clone(&budget)));
    let as_built = the_chains_as_built(
        positions,
        indices,
        chains,
        closed,
        profile,
        options.sharp_turn_degrees,
        is_feature_edge.as_deref(),
    );
    drop(reading);
    if as_built.as_ref().is_ok_and(|(split, split_closed)| {
        square_crease_end_clip::a_clip_leans(
            positions,
            indices,
            split,
            split_closed,
            options.sharp_turn_degrees,
        )
    }) {
        let spent_before = budget.spent.get();
        let was_off = budget.leaning_crease_clips_off.replace(true);
        let square_only = build(Rc::clone(&budget));
        budget.leaning_crease_clips_off.set(was_off);
        let square_only = match square_only {
            Err(BandGapBuildError::Cancelled) => return Err(BandGapBuildError::Cancelled),
            other => other,
        };
        if let Ok(square) = &square_only {
            if square.unflowed_room.is_some() {
                return square_only;
            }
        }

        budget.spent.set(spent_before);
        let leaning = build(Rc::clone(&budget));
        return match (leaning, square_only) {
            (Ok(leaning), Ok(square)) if !leaning.built() && square.built() => Ok(square),
            (Err(BandGapBuildError::Cancelled), _) => Err(BandGapBuildError::Cancelled),
            (Err(_), Ok(square)) if square.built() => Ok(square),
            (leaning, _) => leaning,
        };
    }

    build(budget)
}

/// One chain, unvalidated, with a work budget and the topology-only switch the pinch ladder's
/// probes set.
#[allow(clippy::too_many_arguments)]
pub fn build_one_unvalidated(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    chain_is_closed: bool,
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
    topology_only: bool,
) -> Result<BandGapResult, BandGapBuildError> {
    build_unvalidated(
        positions,
        indices,
        &[chain.to_vec()],
        &[chain_is_closed],
        is_feature_edge,
        size,
        profile,
        options.chord_tolerance,
        options.snap,
        options.known_room,
        options.budget.clone(),
        topology_only,
        options.seam_epsilon,
        options.sharp_turn_degrees,
        options.fillet_segments,
    )
}
