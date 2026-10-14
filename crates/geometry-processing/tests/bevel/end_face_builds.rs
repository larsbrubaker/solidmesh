//! Shared plumbing of the end-face test classes (`end_face_carry_tests`, `flat_ends_clip_tests`,
//! `oblique_end_face_owner_tests` and their neighbours): the band gap build on a fixture (no chord
//! tolerance, a 1e-9 snap, ...) those classes repeat, and the flat-ends override they set round
//! it, as a guard that clears the per-thread switches when it drops.

#![allow(dead_code)]

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::corner_cap::WATCH_END_HOLE_AT;
use geometry_processing::bevel::level_line::corner_cap_flat_ends::set_use_flat_ends_override;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapResult, WorkBudget};

use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;

/// The fixture's feature edges as the kernel takes them.
pub fn feature_of(fixture: &Rc<LevelLineFixture>) -> FeatureEdgeTest {
    let fixture = Rc::clone(fixture);
    Rc::new(move |a, b| fixture.is_feature_edge(a, b))
}

/// The band gap on the fixture at `size` with the given options: the chains picked are `chains`
/// (indices into the fixture's).
pub fn build_chains(
    fixture: &Rc<LevelLineFixture>,
    chains: &[usize],
    size: f64,
    options: &BandGapBuildOptions,
) -> BandGapResult {
    let picked: Vec<Vec<i32>> = chains.iter().map(|&c| fixture.chains[c].clone()).collect();
    let closed: Vec<bool> = chains.iter().map(|&c| fixture.closed[c]).collect();
    band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &picked,
        &closed,
        Some(feature_of(fixture)),
        size,
        profile_of(fixture),
        options,
    )
    .expect("a valid pick")
}

/// No chord tolerance, a 1e-9 snap and sharp turns from `degrees` (every other argument its
/// default).
pub fn at_bend(degrees: f64) -> BandGapBuildOptions {
    BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        sharp_turn_degrees: degrees,
        ..BandGapBuildOptions::default()
    }
}

/// The fixture's own build: no chord tolerance, a 1e-9 snap, `known_room`, and the fixture's seam
/// epsilon, sharp turn and segments.
pub fn as_saved(fixture: &LevelLineFixture, known_room: Option<f64>) -> BandGapBuildOptions {
    BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room,
        budget: None,
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    }
}

/// The default work allowance for the fixture's size, marked float-stored when
/// `input_is_float_stored`.
pub fn default_budget(fixture: &LevelLineFixture, input_is_float_stored: bool) -> Rc<WorkBudget> {
    let mut budget = WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
        .expect("a positive limit");
    budget.input_is_float_stored = input_is_float_stored;
    Rc::new(budget)
}

/// Sets this thread's flat-ends override and, on drop, clears it and the corner cap's
/// `WATCH_END_HOLE_AT`, however the test leaves.
pub struct FlatEnds;

impl FlatEnds {
    pub fn set(value: bool) -> FlatEnds {
        set_use_flat_ends_override(Some(value));
        FlatEnds
    }
}

impl Drop for FlatEnds {
    fn drop(&mut self) {
        set_use_flat_ends_override(None);
        WATCH_END_HOLE_AT.with(|w| *w.borrow_mut() = None);
    }
}
