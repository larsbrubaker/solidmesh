//! An open fill's skew row on one straight run of the chain
//! (`fillet_arc_rows::on_one_straight_run`): drawn on the round's cylinder however far its ends
//! cross the chain apart, and left to the plain row across a joint, past an end, with an end
//! leaning off square, and on tied readings.

use solidmesh::bevel::level_line::fillet_arc_rows::on_one_straight_run;
use solidmesh::bevel::level_line::open_gap_fill::PARAMETER_TIE_RELATIVE;
use solidmesh::bevel::level_line::ChainField;
use solidmesh_math::Vector3;

/// The round's radius: the fold along the chain is the floor z = 0 (y below 0) meeting the wall
/// y = 0 (z above 0), so the round's axis runs along x at y = -R, z = R.
const R: f64 = 12.0;

/// The reading tie the open fill uses on a chain of `length`.
fn tie(length: f64) -> f64 {
    PARAMETER_TIE_RELATIVE * length.max(1.0)
}

/// A straight chain along x from 0 to 100.
fn straight() -> ChainField {
    let points = [Vector3::ZERO, Vector3::new(100.0, 0.0, 0.0)];
    ChainField::from_polyline_sharp(&points, false, &[R, R], 20.0).expect("a chain")
}

/// The floor rail's point and the wall rail's point square to the chain at `x`.
fn floor_end(x: f64) -> Vector3 {
    Vector3::new(x, -R, 0.0)
}

fn wall_end(x: f64) -> Vector3 {
    Vector3::new(x, 0.0, R)
}

#[test]
fn a_row_skewed_55_mm_along_a_straight_chain_lies_on_the_cylinder() {
    let chain = straight();
    let (from, to) = (10.0, 65.2);
    let mean = chain.point_at((from + to) / 2.0);
    let row = on_one_straight_run(
        &chain,
        floor_end(from),
        wall_end(to),
        from,
        to,
        mean,
        tie(chain.length),
    )
    .expect("both readings on one straight run, both ends square");

    for k in 0..=8 {
        let point = row(f64::from(k) / 8.0);
        let off = (point.y + R).hypot(point.z - R) - R;
        assert!(
            off.abs() <= 1e-12 * R,
            "point {k} of the row, {point:?}, stands {off:+e} off the cylinder"
        );
    }
    assert!((row(0.0) - floor_end(from)).length() <= 1e-12 * R);
    assert!((row(1.0) - wall_end(to)).length() <= 1e-12 * R);
}

#[test]
fn a_row_across_a_two_degree_joint_is_not_drawn_on_one_run() {
    let turn = 2.0_f64.to_radians();
    let points = [
        Vector3::ZERO,
        Vector3::new(50.0, 0.0, 0.0),
        Vector3::new(50.0 + (50.0 * turn.cos()), 50.0 * turn.sin(), 0.0),
    ];
    let chain = ChainField::from_polyline_sharp(&points, false, &[R, R, R], 20.0).expect("a chain");
    let (from, to) = (40.0, 60.0);
    let beyond = chain.point_at(to);
    let row = on_one_straight_run(
        &chain,
        floor_end(from),
        beyond + Vector3::new(0.0, 0.0, R),
        from,
        to,
        chain.point_at((from + to) / 2.0),
        tie(chain.length),
    );
    assert!(row.is_none());
}

#[test]
fn a_row_reading_past_the_chains_end_is_not_drawn_on_one_run() {
    let chain = straight();
    let row = on_one_straight_run(
        &chain,
        floor_end(-5.0),
        wall_end(30.0),
        -5.0,
        30.0,
        chain.point_at(12.5),
        tie(chain.length),
    );
    assert!(row.is_none());
}

#[test]
fn a_row_with_an_end_leaning_off_square_is_not_drawn_on_one_run() {
    let chain = straight();
    let leaning = floor_end(10.0) + Vector3::new(1e-3 * R, 0.0, 0.0);
    let row = on_one_straight_run(
        &chain,
        leaning,
        wall_end(65.2),
        10.0,
        65.2,
        chain.point_at(37.6),
        tie(chain.length),
    );
    assert!(row.is_none());
}

#[test]
fn a_row_on_tied_readings_is_not_drawn_on_one_run() {
    let chain = straight();
    let row = on_one_straight_run(
        &chain,
        floor_end(30.0),
        wall_end(30.0),
        30.0,
        30.0,
        chain.point_at(30.0),
        tie(chain.length),
    );
    assert!(row.is_none());
}
