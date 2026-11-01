//! The band gap's public builds: the optional arguments are [`BandGapBuildOptions`] (whose
//! `Default` holds the usual values) and there are four functions - one chain
//! ([`build`]), a whole pick ([`build_pick`]), a whole pick with the caller's face regions for the
//! rolling ball's sides ([`build_pick_with_side_regions`]), and one chain unvalidated with the
//! topology-only switch ([`build_one_unvalidated`]). The funnel they reach is
//! [`band_gap::build_unvalidated`](super::band_gap::build_unvalidated).

use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap::{build_unvalidated, BandGapBuildError};
use super::seam_edges::FeatureEdgeTest;
use super::{pick_validation, BandGapResult, BandProfile, WorkBudget};

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
    build_pick_with_side_regions(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        size,
        profile,
        options,
        None,
    )
}

/// [`build_pick`] with the caller's face region of every triangle of `indices`
/// (`side_regions[t]` is triangle t's region; regions are told apart by their numbers, nothing
/// else). Where the rolling ball's switch is on, each chain's two sides are the regions holding
/// the triangles either side of its edges, in place of the kernel's own flood over the feature
/// edges (`rolling_ball_sides`); `None` is [`build_pick`]. A list whose length is not the
/// triangle count keeps the level line, the route saying so.
#[allow(clippy::too_many_arguments)]
pub fn build_pick_with_side_regions(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
    side_regions: Option<&[u32]>,
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

    build_unvalidated(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
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
        side_regions,
    )
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
        None,
    )
}
