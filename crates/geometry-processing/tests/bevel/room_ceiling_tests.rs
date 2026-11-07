//! What a room reading may COST. A side whose own room stands above the pick's ceiling (its own
//! span) is reported as unbounded from its vertices alone, and reading it never costs what the
//! surface behind it happens to measure.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap::chain_ceiling;
use geometry_processing::bevel::level_line::band_gap_reading::read_room;
use geometry_processing::bevel::level_line::{BandProfile, RoomReading, WorkBudget};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::box_with_square_hole;

/// A rim of half-width 0.5 in a plate of half-width `plate_half`, and
/// the budget's limit.
fn room_of_mouth_in_plate(plate_half: f64) -> (RoomReading, i64) {
    let (positions, indices, rim) = box_with_square_hole(plate_half, 0.5, 1.0);
    let features = feature_edges_over(&positions, &indices, 30.0);
    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit"),
    );
    let limit = budget.limit;
    let reading = read_room(
        &positions,
        &indices,
        &rim,
        true,
        Some(features),
        BandProfile::Fillet,
        1e-9,
        Some(budget),
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick");
    (reading, limit)
}

#[test]
fn small_rim_on_a_very_large_plate_costs_what_the_rim_costs_and_not_what_the_plate_measures() {
    let (near, near_limit) = room_of_mouth_in_plate(4.0);
    let (far, far_limit) = room_of_mouth_in_plate(400.0);

    assert!(near.read);
    assert!(far.read);

    // A hundredfold more flat plate behind the same small rim is not one reading more of the field.
    assert!(
        far.work_spent <= near.work_spent * 2,
        "far {} near {}",
        far.work_spent,
        near.work_spent
    );
    assert!(
        far.work_spent * 20 < far_limit,
        "far {} limit {}",
        far.work_spent,
        far_limit
    );
    assert!(near_limit > 0);
}

#[test]
fn waving_the_far_surface_away_does_not_make_the_room_smaller() {
    let (near, _) = room_of_mouth_in_plate(4.0);
    let (far, _) = room_of_mouth_in_plate(400.0);

    // What bounds this mouth is its own wall, the same number whatever plate stands behind the rim.
    assert!(far.read);
    assert_eq!(far.room, near.room);
    assert!(far.room > 0.0);
}

#[test]
fn a_narrow_side_still_reads_its_true_number() {
    // A plate barely wider than the mouth: the face IS what bounds the size, below the ceiling.
    let (narrow, _) = room_of_mouth_in_plate(1.0);

    assert!(narrow.read);
    assert!(!narrow.unbounded());
    assert!(narrow.room > 0.0, "room {}", narrow.room);
    assert!(narrow.room < 1.0, "room {}", narrow.room);
}

#[test]
fn a_chain_that_does_not_divide_its_sides_says_so_rather_than_naming_a_room_of_nothing() {
    // Nothing is a feature edge at a 100 degree threshold, so the mouth's two sides are one region:
    // the pick is unbuildable at ANY size, which the reading says by name.
    let (positions, indices, rim) = box_with_square_hole(4.0, 0.5, 1.0);
    let nothing_is_a_feature = feature_edges_over(&positions, &indices, 100.0);
    let reading = read_room(
        &positions,
        &indices,
        &rim,
        true,
        Some(nothing_is_a_feature),
        BandProfile::Fillet,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick");

    assert!(reading.read);
    assert!(!reading.chain_divides_sides);
    assert_eq!(reading.room, 0.0);
}

/// The ceiling is a reading of the PICK, so turning the part may not change it.
#[test]
fn the_ceiling_is_the_same_when_the_pick_is_turned() {
    let square = [
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 0.0),
    ];
    let turn = Matrix4X4::create_rotation_z(std::f64::consts::PI / 4.0)
        * Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1));
    let turned: Vec<Vector3> = square.iter().map(|p| p.transform(turn)).collect();

    let still = chain_ceiling(&square);
    let moved = chain_ceiling(&turned);

    assert!(
        (moved - still).abs() <= still * 1e-9,
        "the ceiling of one square rim: {still} still, {moved} turned"
    );

    // And it is still the rim's own span - the diagonal of the square.
    assert!(
        (still - (2.0 * 2f64.sqrt())).abs() <= 1e-9,
        "ceiling {still}"
    );
}

/// A pick's ceiling is its own span however large the part behind it.
#[test]
fn a_small_rim_keeps_a_small_ceiling_however_large_the_part() {
    let rim = [
        Vector3::new(-0.5, -0.5, 0.0),
        Vector3::new(0.5, -0.5, 0.0),
        Vector3::new(0.5, 0.5, 0.0),
        Vector3::new(-0.5, 0.5, 0.0),
    ];

    assert!(chain_ceiling(&rim) < 2.0);
}
