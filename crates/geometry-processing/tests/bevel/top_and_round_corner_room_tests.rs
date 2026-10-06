//! Port of `LevelLine/TopAndRoundCornerRoomTests.cs`: "Top & Round Should be Union.mcx", its
//! second fillet as the app route hands it to the kernel - three open chains that all run between
//! the same two corners. Size 3.756, which the old construction built cleanly.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_room_pair_room::room_of_pick;
use geometry_processing::bevel::level_line::BandProfile;

use crate::level_line_fixture::LevelLineFixture;

#[test]
fn the_saved_size_fits_between_chains_that_share_both_corners() {
    let fixture = Rc::new(LevelLineFixture::load("top-and-round-op1"));
    let features = Rc::clone(&fixture);
    let pick = room_of_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        BandProfile::Fillet,
        1e-9,
        fixture.size,
        0.0,
        f64::INFINITY,
        0,
    )
    .expect("the room reads");
    println!(
        "top and round: room {:?} set by {:?}",
        pick.room, pick.set_by
    );
    for candidate in &pick.candidates {
        println!("    {candidate:?}");
    }

    assert!(
        pick.room >= fixture.size,
        "room {} size {}",
        pick.room,
        fixture.size
    );
}
