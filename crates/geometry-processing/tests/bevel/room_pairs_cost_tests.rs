//! A pick of several chains reads an ordered pair of chains only where that pair could be the
//! narrowest thing on the part. The fixture is the two-boss plate (several-chains fixture B)
//! stretched threefold in x, so neither pair can be the least reading; the count of `ChainToChain`
//! candidate rows is the call count the tests hold.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_reading::{read_room_of_pick, room_margin};
use solidmesh::bevel::level_line::{
    BandProfile, RoomReading, RoomReadingKind, RoomSource, WorkBudget,
};
use solidmesh_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChains;
use crate::several_chains_fixtures::plate_with_two_bosses;

const FOLD_DEGREES: f64 = 20.0;

/// The same proper rotation and translation `RoomRigidMotionTests` moves by.
fn motion() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125))
}

/// Fixture B stretched threefold in x.
fn bosses_far_apart(moved: bool) -> SolidWithChains {
    let (raw, indices, first, second) = plate_with_two_bosses();
    let stretched = raw.iter().map(|p| Vector3::new(p.x * 3.0, p.y, p.z));
    let positions: Vec<Vector3> = if moved {
        let by = motion();
        stretched.map(|p| p.transform(by)).collect()
    } else {
        stretched.collect()
    };
    (positions, indices, vec![first, second])
}

fn read(moved: bool, budget: Rc<WorkBudget>) -> RoomReading {
    let (positions, indices, chains) = bosses_far_apart(moved);
    let features = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &[true, true],
        Some(features),
        BandProfile::Chamfer,
        1e-9,
        Some(budget),
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick")
}

fn budget() -> Rc<WorkBudget> {
    Rc::new(WorkBudget::new(5_000_000).expect("a positive limit"))
}

/// Pairs that cannot be the least are not paid for; one test per case.
fn pairs_that_cannot_be_the_least_are_not_paid_for(moved: bool) {
    let reading = read(moved, budget());

    assert!(reading.read);

    // Each boss's own wall, 2 up to its cap rim, less the margin a kept strip needs.
    let expected = 2.0 - room_margin(1.0, 1e-9);
    assert!(
        (reading.room - expected).abs() <= 1e-9,
        "room {} expected {}",
        reading.room,
        expected
    );

    // Neither pair may NAME the room either.
    assert_ne!(reading.room_set_by.kind, RoomReadingKind::ChainToChain);

    // A skipped pair has no row: the table says what was read, and the least of it is the room.
    assert!(!reading
        .candidates
        .iter()
        .any(|c| c.source.kind == RoomReadingKind::ChainToChain));
    let least = reading
        .candidates
        .iter()
        .map(|c| c.room)
        .fold(f64::INFINITY, f64::min);
    assert_eq!(least, reading.room);
}

#[test]
fn pairs_that_cannot_be_the_least_are_not_paid_for_false() {
    pairs_that_cannot_be_the_least_are_not_paid_for(false);
}

#[test]
fn pairs_that_cannot_be_the_least_are_not_paid_for_true() {
    pairs_that_cannot_be_the_least_are_not_paid_for(true);
}

/// The moved copy answers the SAME room to 1e-9, having taken the same readings.
#[test]
fn the_moved_copy_reads_the_same_room() {
    let still = read(false, budget());
    let moved = read(true, budget());

    assert!(
        (moved.room - still.room).abs() / still.room < 1e-9,
        "still {} moved {}",
        still.room,
        moved.room
    );
    assert_eq!(still.room_set_by, RoomSource::foreign(0));

    assert_eq!(moved.candidates.len(), still.candidates.len());
    assert!(!moved
        .candidates
        .iter()
        .any(|c| c.source.kind == RoomReadingKind::ChainToChain));
}
