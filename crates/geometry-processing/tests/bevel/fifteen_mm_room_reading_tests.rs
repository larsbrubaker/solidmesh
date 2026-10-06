//! Port of `LevelLine/FifteenMmRoomReadingTests.cs`: what refuses "15mm Bevel Top Edge" at its saved
//! 15 - not the top face's width, as the refusal's words suggest, but the buried nose's concave
//! crease on the slope, 20/3 below the top at the pick's crease end.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::BandProfile;
use geometry_processing::bevel::level_line::{RoomReading, RoomReadingKind};

use crate::level_line_fixture::LevelLineFixture;

/// The nose's upper crease on the left slope, in the fixture's frame.
const RIDGE_FOOT: (f64, f64, f64) = (-15.0, -113.5, 10.0);

const UPPER_CREASE_TOP: (f64, f64, f64) = (-10.0, -108.5, 13.3333);

/// C# `TheSavedFifteenIsRefusedByTheNosesCreaseOnTheSlope`.
#[test]
fn the_saved_fifteen_is_refused_by_the_noses_crease_on_the_slope() {
    let fixture = Rc::new(LevelLineFixture::load("fifteen-mm-top-edge-r659"));

    let all = Rc::clone(&fixture);
    let reading = read(&fixture, Rc::new(move |a, b| all.is_feature_edge(a, b)));
    assert!(
        (reading.room - 20.0 / 3.0).abs() <= 1e-5,
        "the nose's upper face meets the slope 20/3 below the top at the pick's crease end: room {}",
        reading.room
    );
    assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign);
    assert!(
        !reading.room_set_by.is_a_growing_crease(),
        "the nose's upper crease is ON the slope the band crosses, not a growing neighbour's far edge"
    );

    let ridge_foot = find(&fixture, RIDGE_FOOT);
    let crease_top = find(&fixture, UPPER_CREASE_TOP);
    let the_crease = move |a: i32, b: i32| {
        (a == ridge_foot && b == crease_top) || (a == crease_top && b == ridge_foot)
    };
    let rest = Rc::clone(&fixture);
    let without_the_crease = read(
        &fixture,
        Rc::new(move |a, b| rest.is_feature_edge(a, b) && !the_crease(a, b)),
    );
    assert!(
        (without_the_crease.room - 10.0).abs() <= 1e-5,
        "with that crease gone the nose's ridge, 10 down at its foot, is the next reading - still \
         under 15 and still the nose, not the top face's 14.14: room {}",
        without_the_crease.room
    );
}

fn read(fixture: &LevelLineFixture, is_feature_edge: FeatureEdgeTest) -> RoomReading {
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    band_gap_reading::read_room_of_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(is_feature_edge),
        profile,
        1e-9,
        None,
        15.0,
        0.0,
        20.0,
    )
    .expect("the reading")
}

fn find(fixture: &LevelLineFixture, point: (f64, f64, f64)) -> i32 {
    let mut index = -1;
    for (i, p) in fixture.positions.iter().enumerate() {
        if (p.x - point.0).abs() < 1e-3
            && (p.y - point.1).abs() < 1e-3
            && (p.z - point.2).abs() < 1e-3
        {
            index = i as i32;
        }
    }

    index
}
