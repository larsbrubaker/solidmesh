//! Port of `LevelLine/LandSliverJoinsTwoBandsTests.cs`: Many Holes' touching-split pass A (37 port
//! mouths and the cut end's rim, fillet 0.5) was refused by a PINCH at 0.234 the part does not
//! have. The smallest pick that shows it is three chains: the cut end's rim (chain 0) and the two
//! ports of one column (chains 4 and 5). The land between the ports is a fan of float-stored
//! slivers; `LevelLineMesh::split_edge` reading a vertex within the float step as ON the edge, and
//! the needle apex re-tile before rooting, keep the two bands apart.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{BandGapRefusal, WorkBudget};

use crate::end_face_builds::build_chains;
use crate::level_line_fixture::LevelLineFixture;

#[test]
fn two_ports_of_a_column_and_the_cut_end_build_at_half_a_millimetre() {
    let fixture = Rc::new(LevelLineFixture::load("many-holes-passA-37chains-r05"));

    let mut budget = WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
        .expect("a positive limit");
    budget.input_is_float_stored = true;
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::new(budget)),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    let built = build_chains(&fixture, &[0, 4, 5], fixture.size, &options);

    assert_eq!(
        built.refusal,
        BandGapRefusal::None,
        "the two ports are 4.28 mm apart, so their 0.5 mm bands cannot meet: room {}, set by {:?}",
        built.room,
        built.room_set_by
    );
}
