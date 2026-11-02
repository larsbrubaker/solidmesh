//! The rail joined past a rib (`virtual_rail::join`): the points the join mints on a side stand at
//! the chain's own setback.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::virtual_rail;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, GapLoop};

use crate::level_line_feature_edges::feature_edges_over;
use crate::strip_clip_rib_meshes;

/// The rib synthetic's band at size 5 (its top side's rail cut in two by the rib).
fn the_ribs_band() -> BandGapResult {
    let positions = strip_clip_rib_meshes::positions(None);
    let indices = strip_clip_rib_meshes::ribbed();
    band_gap_entry_points::build(
        &positions,
        &indices,
        &strip_clip_rib_meshes::CHAIN,
        false,
        Some(feature_edges_over(&positions, &indices, 20.0)),
        5.0,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the rib builds or refuses by name")
}

/// The minted points' distances from the chain's line y = 0, z = 10 (the top's plane is z = 10).
fn minted_at(gap: &BandGapResult, minted: &[i32]) -> Vec<f64> {
    let mesh = gap.mesh.as_ref().expect("a built gap's mesh").borrow();
    minted
        .iter()
        .map(|&v| {
            let p = mesh.position(v);
            assert!((p.z - 10.0).abs() <= 1e-9, "on the top's plane: {p:?}");
            p.y
        })
        .collect()
}

/// The top side's two runs, in ascending chain parameter.
fn top_runs(gap: &BandGapResult) -> Vec<GapLoop> {
    let least = |l: &GapLoop| l.parameters.iter().copied().fold(f64::INFINITY, f64::min);
    let mut runs: Vec<GapLoop> = gap.loops.iter().filter(|l| l.side == 1).cloned().collect();
    // A stable sort, so loops with equal keys keep their order.
    runs.sort_by(|a, b| least(a).total_cmp(&least(b)));
    assert_eq!(runs.len(), 2, "the rib cuts the top's rail in two");
    runs
}

/// `join` lays each minted point at the chain's own setback, 5.
#[test]
fn a_rail_joined_past_the_rib_stands_at_the_chains_setback() {
    let gap = the_ribs_band();
    let runs = top_runs(&gap);
    let chain = Rc::clone(&gap.chains[0]);
    let mut minted = Vec::new();
    {
        let mesh = gap.mesh.as_ref().expect("a built gap's mesh");
        virtual_rail::join(
            &mut mesh.borrow_mut(),
            &chain,
            &runs[0],
            &runs[1],
            &mut minted,
        )
        .expect("no budget")
        .expect("the runs join");
    }
    assert!(!minted.is_empty());
    for y in minted_at(&gap, &minted) {
        assert!((y - 5.0).abs() <= 1e-9, "at the chain's setback: {y}");
    }
}
