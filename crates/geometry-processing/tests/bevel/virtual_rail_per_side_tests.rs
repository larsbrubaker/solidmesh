//! The rail joined past a rib laid at its own side's setback: where the rolling ball cuts the two
//! sides at different distances the result carries each side's field (`side_chains`), and the
//! points the join mints on a side stand at that side's setback, not the chain's own (the larger).

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::virtual_rail;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, ChainField, GapLoop};

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

/// The chain's own field with every setback `scale` times its own.
fn scaled(chain: &ChainField, scale: f64) -> Rc<ChainField> {
    let setbacks: Vec<f64> = chain.setbacks.iter().map(|s| s * scale).collect();
    Rc::new(
        ChainField::from_polyline_bounded(
            &chain.points,
            chain.is_closed,
            &setbacks,
            chain.sharp_turn_degrees,
            chain.end_rays,
            chain.end_ray_bounds,
        )
        .expect("the chain's polyline builds a field"),
    )
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

/// `join_at` lays each minted point at the rail field's setback (4, where the chain's own is 5),
/// and `join` - the rail the chain itself - lays them where `join_at` with the chain does.
#[test]
fn a_rail_joined_past_the_rib_stands_at_the_rail_fields_setback() {
    let gap = the_ribs_band();
    let chain = Rc::clone(&gap.chains[0]);
    let rail = scaled(&chain, 0.8);
    let runs = top_runs(&gap);
    let mut minted = Vec::new();
    {
        let mesh = gap.mesh.as_ref().expect("a built gap's mesh");
        virtual_rail::join_at(
            &mut mesh.borrow_mut(),
            &chain,
            &rail,
            &runs[0],
            &runs[1],
            &mut minted,
        )
        .expect("the join spends no budget it lacks")
        .expect("the top's two runs join");
    }
    assert!(!minted.is_empty());
    for y in minted_at(&gap, &minted) {
        assert!(
            (y - 4.0).abs() <= 1e-9,
            "at the rail's setback 4, not 5: {y}"
        );
    }

    let plain = the_ribs_band();
    let same = the_ribs_band();
    let (mut by_join, mut by_join_at) = (Vec::new(), Vec::new());
    for (gap, minted, own) in [
        (&plain, &mut by_join, true),
        (&same, &mut by_join_at, false),
    ] {
        let runs = top_runs(gap);
        let chain = Rc::clone(&gap.chains[0]);
        let mesh = gap.mesh.as_ref().expect("a built gap's mesh");
        let mut mesh = mesh.borrow_mut();
        let joined = if own {
            virtual_rail::join(&mut mesh, &chain, &runs[0], &runs[1], minted)
        } else {
            virtual_rail::join_at(&mut mesh, &chain, &chain, &runs[0], &runs[1], minted)
        };
        joined.expect("no budget").expect("the runs join");
    }
    assert_eq!(minted_at(&plain, &by_join), minted_at(&same, &by_join_at));
    for y in minted_at(&plain, &by_join) {
        assert!((y - 5.0).abs() <= 1e-9, "at the chain's setback: {y}");
    }
}

/// With the ball's side fields on the result, `joined_sides` joins the top side (side 1) at side
/// 1's own setback (4), not the chain's own field's (5).
#[test]
fn joined_sides_lays_each_sides_rail_at_its_own_sides_setback() {
    let mut gap = the_ribs_band();
    let chain = Rc::clone(&gap.chains[0]);
    gap.side_chains = Some(vec![[scaled(&chain, 0.8), Rc::clone(&chain)]]);
    let mut minted = Vec::new();
    let loops = virtual_rail::joined_sides(&gap, &mut minted).expect("the join runs");
    assert_eq!(loops.iter().filter(|l| l.side == 1).count(), 1, "joined");
    assert!(!minted.is_empty());
    for y in minted_at(&gap, &minted) {
        assert!(
            (y - 4.0).abs() <= 1e-9,
            "side 1's setback 4, not the chain's 5: {y}"
        );
    }
}
