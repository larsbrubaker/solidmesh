//! Ports `Tests/GeometryProcessingTests/LevelLine/ParallelEdgesRoomTests.cs`: "Parallel Edges Too
//! Close.mcx" at the kernel - the two top edges of a 20 x 6 x 20 bar, chamfered. The room is half
//! the 6 mm face less only the room margin, and a build AT the room builds.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;

/// C# `TheRoomIsHalfTheFaceAndBuildsAtExactlyThatSize`.
#[test]
fn the_room_is_half_the_face_and_builds_at_exactly_that_size() {
    let positions = vec![
        Vector3::new(-10.0, -3.0, 10.0),
        Vector3::new(10.0, -3.0, 10.0),
        Vector3::new(10.0, 3.0, 10.0),
        Vector3::new(-10.0, 3.0, 10.0),
        Vector3::new(-10.0, -3.0, -10.0),
        Vector3::new(10.0, -3.0, -10.0),
        Vector3::new(10.0, 3.0, -10.0),
        Vector3::new(-10.0, 3.0, -10.0),
    ];
    let indices = vec![
        (0, 1, 2),
        (0, 2, 3),
        (4, 0, 3),
        (4, 3, 7),
        (1, 5, 6),
        (1, 6, 2),
        (4, 7, 6),
        (4, 6, 5),
        (3, 2, 6),
        (3, 6, 7),
        (4, 5, 1),
        (4, 1, 0),
    ];
    let feature = feature_edges_over(&positions, &indices, 10.0);
    let chains = vec![vec![3, 2], vec![0, 1]];
    let closed = [false, false];

    let reading = band_gap_reading::read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(Rc::clone(&feature)),
        BandProfile::Chamfer,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the room reads");

    let expected = 3.0 - band_gap_reading::room_margin(1.0, 1e-9);
    assert!(
        (reading.room - expected).abs() <= 1e-12,
        "each chamfer may take half the 6 mm face - read {} set by {:?}",
        reading.room,
        reading.room_set_by
    );

    let at_the_room = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(Rc::clone(&feature)),
        reading.room,
        BandProfile::Chamfer,
        &BandGapBuildOptions {
            known_room: Some(reading.room),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick builds or refuses by name");

    assert_eq!(
        at_the_room.refusal,
        BandGapRefusal::None,
        "the room quoted to the user is a size that builds"
    );

    let over_the_room = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature),
        4.0,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");

    assert_eq!(over_the_room.refusal, BandGapRefusal::Size);
    assert_eq!(
        over_the_room.room, reading.room,
        "the refusal names the same room the reading does"
    );
}
