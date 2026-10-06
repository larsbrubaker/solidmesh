//! Port of `LevelLine/ManyHolesMouthRoomTests.cs`: Many Holes' port mouths refused
//! SizeExceedsFaceWidth at 0.5 on the face-rim pick, read one at a time off the face rim's own dump -
//! the room is honest (the part really has less land there), not read low.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_room;
use geometry_processing::bevel::level_line::{
    BandProfile, RoomReading, RoomReadingKind, WorkBudget,
};

use crate::level_line_fixture::LevelLineFixture;

const FACE_RIM: &str = "many-holes-facerim-25chains-r05";

/// The z of the ring crease the top row of mouths stands under.
const RING_Z: f64 = -51.687;

/// C# `ATopRowMouthsRoomIsTheLandUnderTheRing`: a top-row mouth (chain 18, centre z -53.33): its
/// room is the land between the mouth's top and the ring above it - about 0.17 mm on the part.
#[test]
fn a_top_row_mouths_room_is_the_land_under_the_ring() {
    let fixture = Rc::new(LevelLineFixture::load(FACE_RIM));
    const K: usize = 18;
    let chain = &fixture.chains[K];
    let land = RING_Z
        - chain
            .iter()
            .map(|&v| fixture.positions[v as usize].z)
            .fold(f64::NEG_INFINITY, f64::max);

    let reading = read_one(&fixture, K);

    assert!(
        land < 0.2,
        "the mouth's top stands {land:.4} under the ring"
    );
    assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign);
    assert!(
        reading.room <= land,
        "the room {:.4} cannot exceed the land {land:.4}",
        reading.room
    );
    assert!(
        reading.room > 0.8 * land,
        "and is not read low against it: room {:.4}, land {land:.4}",
        reading.room
    );
}

/// C# `AMouthACreaseRunsIntoHasNoRoom`: chain 16 reads a room of zero - a 7.2 mm crease of the
/// picked face's own rim (edge 6081-7095) runs up the barrel and ends ON this mouth.
#[test]
fn a_mouth_a_crease_runs_into_has_no_room() {
    let fixture = Rc::new(LevelLineFixture::load(FACE_RIM));
    const K: usize = 16;
    let chain = &fixture.chains[K];

    let reading = read_one(&fixture, K);

    assert!(
        chain.contains(&7095),
        "the crease's end is a vertex of the mouth"
    );
    assert!(
        !chain.contains(&6081),
        "and its other end is 7.2 mm away, off the mouth"
    );
    assert!(fixture.is_feature_edge(6081, 7095), "the edge is a crease");
    assert_eq!(
        reading.room, 0.0,
        "a crease touching the mouth leaves it no land"
    );
}

fn read_one(fixture: &Rc<LevelLineFixture>, k: usize) -> RoomReading {
    let mut budget = WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
        .expect("a positive limit");
    budget.input_is_float_stored = true;
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let features = Rc::clone(fixture);
    band_room::read_room_of_one(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains[k],
        fixture.closed[k],
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        profile,
        1e-9,
        &Rc::new(budget),
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
    )
    .expect("the reading")
}
