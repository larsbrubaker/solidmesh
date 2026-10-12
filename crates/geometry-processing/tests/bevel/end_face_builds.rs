//! Shared plumbing of the end-face test classes (`LevelLine/EndFaceCarryTests.cs`,
//! `FlatEndsClipTests.cs`, `ObliqueEndFaceOwnerTests.cs` and their neighbours): the C#
//! `BandGap.Build(fixture..., chordTolerance: 0, snap: 1e-9, ...)` call those classes repeat, and
//! the `try { CornerCap.UseFlatEndsOverride = ...; } finally { ... = null; }` they wrap it in, as a
//! guard that clears the per-thread switches when it drops.

#![allow(dead_code)]

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::corner_cap::WATCH_END_HOLE_AT;
use geometry_processing::bevel::level_line::corner_cap_flat_ends::set_use_flat_ends_override;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapResult, WorkBudget};

use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;

/// The fixture's feature edges as the kernel takes them (C# `fixture.IsFeatureEdge`).
pub fn feature_of(fixture: &Rc<LevelLineFixture>) -> FeatureEdgeTest {
    let fixture = Rc::clone(fixture);
    Rc::new(move |a, b| fixture.is_feature_edge(a, b))
}

/// C# `BandGap.Build(positions, indices, chains, closed, IsFeatureEdge, size, Profile, ...)` on the
/// fixture with the given options: the chains picked are `chains` (indices into the fixture's).
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

/// C#'s `chordTolerance: 0, snap: 1e-9, sharpTurnDegrees: degrees` (every other argument its
/// default).
pub fn at_bend(degrees: f64) -> BandGapBuildOptions {
    BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        sharp_turn_degrees: degrees,
        ..BandGapBuildOptions::default()
    }
}

/// C#'s `0, 1e-9, knownRoom, null, fixture.SeamEpsilon, fixture.SharpTurnDegrees,
/// fixture.Segments`: the fixture's own build.
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

/// C#'s `new WorkBudget(WorkBudget.DefaultLimit(fixture.Indices.Count)) { InputIsFloatStored =
/// ... }`: the default allowance for the fixture's size.
pub fn default_budget(fixture: &LevelLineFixture, input_is_float_stored: bool) -> Rc<WorkBudget> {
    let mut budget = WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
        .expect("a positive limit");
    budget.input_is_float_stored = input_is_float_stored;
    Rc::new(budget)
}

/// Sets this thread's `CornerCap.UseFlatEndsOverride` and, on drop, clears it and
/// `CornerCap.WatchEndHoleAt` (C#'s `finally`).
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
