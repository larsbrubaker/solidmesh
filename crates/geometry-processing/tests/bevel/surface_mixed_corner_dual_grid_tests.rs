//! Port of `SurfaceMixedCornerDualGridTests.cs`: the grid the leaning one-crease mixed corner's
//! patch is sewn over, read off the fixture's own strips - the bead's ring at station zero, the
//! cut's ring along the far rail while that strip lasts, the run-out's own rows past it, and one
//! point at the end.

use std::rc::Rc;

use geometry_processing::bevel::surface::{
    SurfaceEdgePlan, SurfaceMixedCornerDualCanal, SurfaceMixedCornerDualGrid, SurfaceStrip,
};
use vector_math::math_helper::radians_to_degrees;

use crate::nose_in_notch_mixed_corners;
use crate::nose_in_notch_test_meshes;

const RADIUS: f64 = 1.0;

/// The sampling the fixture's own group asks for, which the cut's ring is chorded at.
const SEGMENTS: i32 = 16;

const EXACT: f64 = 1e-9;

/// The grid of the fixture's own leaning corner, at the radius and sampling its group asks for.
fn the_grid() -> SurfaceMixedCornerDualGrid {
    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(
        nose_in_notch_test_meshes::left_ridge_end(),
        RADIUS,
    );
    let crease = (nose_in_notch_test_meshes::lower_crease_end()
        - nose_in_notch_test_meshes::left_ridge_end())
    .length();

    let canal = SurfaceMixedCornerDualCanal::try_read(&corner, crease)
        .unwrap_or_else(|why| panic!("the canal refused: {why:?}"));

    SurfaceMixedCornerDualGrid::try_read_strips(
        Rc::new(canal),
        &the_strip_of(&corner.cut),
        &the_strip_of(&corner.bead),
    )
    .unwrap_or_else(|why| panic!("the grid refused: {why:?}"))
}

fn the_strip_of(edge: &SurfaceEdgePlan) -> SurfaceStrip {
    SurfaceStrip::try_create(edge, SEGMENTS)
        .unwrap_or_else(|why| panic!("the strip refused: {why:?}"))
}

/// Rail zero rides the WALL at every station of the whole turn, the run-out's included.
#[test]
fn rail_zero_rides_the_wall_the_whole_way() {
    let grid = the_grid();
    let corner = Rc::clone(&grid.canal.corner);

    for station in 0..grid.station_count() {
        let at = grid.at(0, station);

        let off = (at - corner.corner).dot(corner.out_of_the_wall);
        assert!(
            off.abs() <= EXACT,
            "station {station} of {} stands off the wall at {at:?}",
            grid.station_count()
        );
    }
}

/// The far rail stands on the CUT's tube while the cut's strip carries it and on the CAP's plane
/// afterwards.
#[test]
fn the_far_rail_is_the_cuts_tube_and_then_the_caps_plane() {
    let grid = the_grid();
    let corner = Rc::clone(&grid.canal.corner);
    let last = grid.rails_per_row() - 1;

    for station in 0..grid.stations_the_cut_carries() {
        let at = grid.at(last, station);

        let off = corner.cut_center_line().distance_to(at);
        assert!(
            (off - RADIUS).abs() <= EXACT,
            "station {station} is one the cut carries, and its far rail stands at {at:?}"
        );

        assert!(
            (at - grid.cut_ring[station]).length() < EXACT,
            "and it IS the cut ring's own rail there - instead {at:?} against {:?}",
            grid.cut_ring[station]
        );
    }

    for station in grid.stations_the_cut_carries()..grid.station_count() {
        let at = grid.at(last, station);

        let off = (at - corner.corner).dot(corner.out_of_the_cap_face);
        assert!(
            off.abs() <= EXACT,
            "station {station} is past the cut's own strip, so its far rail stands on the cap - instead {at:?}"
        );
    }
}

/// Station zero IS the bead's own end ring, rail for rail, and every point of every row stands
/// one radius off the canal's spine.
#[test]
fn station_zero_is_the_beads_ring_and_every_row_is_on_the_canal() {
    let grid = the_grid();
    let spine = &grid.canal.dual.spine;

    for rail in 0..grid.rails_per_row() {
        assert!(
            (grid.at(rail, 0) - grid.bead_ring[rail]).length() < EXACT,
            "rail {rail} of station zero is the bead ring's own - instead {:?} against {:?}",
            grid.at(rail, 0),
            grid.bead_ring[rail]
        );
    }

    for station in 0..grid.station_count() {
        for rail in 0..grid.rails_per_row() {
            let off = (grid.at(rail, station) - spine.centre_at(grid.stations[station])).length();

            assert!(
                (off - RADIUS).abs() <= EXACT,
                "rail {rail} of station {station} stands {off} off the ball it is on, at {:?}",
                grid.at(rail, station)
            );
        }
    }
}

/// The closing station is ONE point - every rail of it the place the blend dies on the crease.
#[test]
fn the_closing_station_is_one_point() {
    let grid = the_grid();
    let closing = grid.station_count() - 1;

    assert!(
        grid.is_the_point(closing),
        "that is what the last station is"
    );

    assert!(
        (grid.stations[closing] - grid.canal.dies_at).abs() <= EXACT,
        "and it stands at the turn the blend dies at - instead {} against {}",
        grid.stations[closing],
        grid.canal.dies_at
    );

    for rail in 0..grid.rails_per_row() {
        assert!(
            (grid.at(rail, closing) - grid.canal.dies_on()).length() < EXACT,
            "rail {rail} of it stands at {:?} where the blend dies on {:?}",
            grid.at(rail, closing),
            grid.canal.dies_on()
        );
    }
}

/// The stations march ONE way, from zero to the turn the blend dies at, and the run-out is chorded
/// no coarser than the cut's own ring already was.
#[test]
fn the_stations_march_one_way_no_coarser_than_the_cuts_own_ring() {
    let grid = the_grid();

    assert!(
        grid.stations[0].abs() <= 1e-5,
        "instead {}",
        grid.stations[0]
    );

    let closing = grid.station_count() - 1;
    assert!(
        (grid.stations[closing] - grid.canal.dies_at).abs() <= EXACT,
        "instead {}",
        grid.stations[closing]
    );

    let mut widest: f64 = 0.0;

    for station in 1..grid.station_count() {
        assert!(
            grid.stations[station] > grid.stations[station - 1],
            "station {station} doubles back on {}",
            station - 1
        );

        widest = widest.max(grid.stations[station] - grid.stations[station - 1]);
    }

    let cuts_own_step =
        grid.canal.the_cut_strip_ends_at() / (grid.stations_the_cut_carries() - 1) as f64;

    assert!(
        widest <= cuts_own_step + EXACT,
        "the widest step of the whole grid is {:.3} degrees where the cut's own ring steps {:.3}",
        radians_to_degrees(widest),
        radians_to_degrees(cuts_own_step)
    );

    // Recorded, not derived: 21 rails over the cut's 112.62 degrees is 5.63 a step, and the
    // run-out's 45.88 takes nine of them.
    assert_eq!(
        grid.stations_the_cut_carries(),
        21,
        "instead {}",
        grid.stations_the_cut_carries()
    );

    assert_eq!(
        grid.station_count() - grid.stations_the_cut_carries(),
        9,
        "instead the run-out takes {} stations",
        grid.station_count() - grid.stations_the_cut_carries()
    );
}
