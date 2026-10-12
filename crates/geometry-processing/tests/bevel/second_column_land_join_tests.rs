//! Port of `LevelLine/SecondColumnLandJoinTests.cs`: two trios of Many Holes' ports - pass A's
//! chains 2, 3 and 9 and the face rim's 2, 3 and 8, each a few millimetres apart on the same 15,214
//! triangles - that must build at 0.5. Both falsely refused Size at 0.469 on a ChainToChain join
//! across the land between the ports. The join was not in the input but MINTED by the refinement:
//! the dip, bump and ridge rules split the land's 16 mm strip edges at the same few fixed places of
//! the field until the children fell under the no-height floor and a kept and a removed needle
//! shared a long edge from one port's line to the other's. The sliver guard in
//! `BandRefinement.RefineByAsking` ends it.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::BandGapRefusal;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::level_line_fixture::LevelLineFixture;

#[test]
fn three_ports_of_the_second_column_build_at_half_a_millimetre() {
    let fixture = Rc::new(LevelLineFixture::load("many-holes-passA-37chains-r05"));
    let options = BandGapBuildOptions {
        budget: Some(default_budget(&fixture, true)),
        ..as_saved(&fixture, None)
    };
    let built = build_chains(&fixture, &[2, 3, 9], fixture.size, &options);

    assert_eq!(
        built.refusal,
        BandGapRefusal::None,
        "the ports are about 4.3 mm apart, so their 0.5 mm bands cannot meet: room {}, set by {:?}",
        built.room,
        built.room_set_by
    );
}

/// The face rim's chains 2, 3 and 8. A false join here is what makes the app's 21- and 20-survivor
/// builds of this face refuse at 0.469, each paying a four-probe pinch ladder and a dropped loop -
/// the cost behind `BevelManyHolesRegressionTests.EveryPortMouthTakesTheHalfMillimetreFilletLarsAsked`.
#[test]
fn three_ports_of_the_face_rim_build_at_half_a_millimetre() {
    let fixture = Rc::new(LevelLineFixture::load("many-holes-facerim-25chains-r05"));
    let options = BandGapBuildOptions {
        budget: Some(default_budget(&fixture, true)),
        ..as_saved(&fixture, None)
    };
    let built = build_chains(&fixture, &[2, 3, 8], fixture.size, &options);

    assert_eq!(
        built.refusal,
        BandGapRefusal::None,
        "these ports built at 0.5 before the reuse, so their bands do not meet: {:?} (phase {:?}, edge {:?}), room {}, set by {:?}",
        built.refusal,
        built.phase,
        built.offending_edge,
        built.room,
        built.room_set_by
    );
}
