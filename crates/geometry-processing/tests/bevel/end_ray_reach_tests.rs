//! A chain built alone adds its exact prism, however its end ray runs. Inner Right Sides and
//! Center's concave chains 5-9 and 9-11-8 end at 9, the tip of a tent standing on a flat floor; an
//! unbounded ray ran the floor's rail 8.4 mm straight past the tip. Built alone, each must now add
//! its exact prism (2.40) within 0.3. Parameter rows are a loop.

use std::rc::Rc;

use solidmesh::bevel::level_line::{BandProfile, GapFillRefusal};

use crate::end_face_builds::{as_saved, feature_of, FlatEnds};
use crate::end_face_measures::prism;
use crate::hole_fill_corner_tests::fill_with_the_filler;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use solidmesh::bevel::level_line::band_gap_entry_points;

#[test]
fn a_concave_chain_ending_at_a_tent_tip_adds_its_prism_alone() {
    for chain_index in [0usize, 2] {
        // flat ends (plan stage 2): the ray bounded at the end face's plane AND the hole it leaves
        // closed flat - the bound alone left the last row starting at the end vertex, which
        // ZipEnd refused as not star-shaped
        let _flat_ends = FlatEnds::set(true);
        chain_alone_adds_its_prism(chain_index);
    }
}

fn chain_alone_adds_its_prism(chain_index: usize) {
    let fixture = Rc::new(LevelLineFixture::load("inner-right-sides-and-center-op0"));
    let chain = &fixture.chains[chain_index];
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        std::slice::from_ref(chain),
        &[fixture.closed[chain_index]],
        Some(feature_of(&fixture)),
        fixture.size,
        BandProfile::Fillet,
        &as_saved(&fixture, None),
    )
    .expect("a valid pick");

    assert!(gap.built(), "chain {chain_index}: {:?}", gap.refusal);
    let filled = fill_with_the_filler(&gap, BandProfile::Fillet, fixture.segments);
    println!(
        "chain {chain_index}: fill {:?}, vertex {}, edge {:?}",
        filled.refusal, filled.offending_vertex, filled.open_edge
    );
    assert_eq!(filled.refusal, GapFillRefusal::None, "chain {chain_index}");

    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    let removed = signed_volume_of(&fixture.positions, &fixture.indices) - signed_volume(&mesh);
    let prism = prism(
        &fixture.positions,
        &fixture.indices,
        chain,
        false,
        fixture.size,
    );
    println!(
        "chain {chain_index} alone removes {removed:.4}, exact prism {prism:.4}, difference {:.4}",
        removed - prism
    );
    assert!((removed - prism).abs() < 0.3, "chain {chain_index}");
}
