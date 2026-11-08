// The size rule every curve tolerance scales by.

use solidmesh_clipper::curve_tolerance::{for_size, scale_for};

/// Desk-size shapes keep EXACTLY the tolerance they always had.
#[test]
fn at_or_under_desk_size_the_tolerance_is_exactly_the_base() {
    for size in [0.0, 0.001, 50.0, 299.999, 300.0, -5000.0] {
        assert_eq!(scale_for(size), 1.0);
        assert_eq!(for_size(0.125, size), 0.125);
        assert_eq!(for_size(0.25, size), 0.25);
    }
}

/// Above the bed the tolerance is a pure proportion of the size.
#[test]
fn above_desk_size_the_tolerance_grows_in_proportion() {
    for (size, expected_scale) in [(600.0, 2.0), (3000.0, 10.0), (6096.0, 20.32)] {
        assert!((scale_for(size) - expected_scale).abs() <= 1e-12);
        assert!((for_size(0.25, size) - 0.25 * expected_scale).abs() <= 1e-12);
    }
}

/// A size that is not a number reads as desk size.
#[test]
fn a_size_that_is_not_a_number_reads_as_desk_size() {
    for size in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(scale_for(size), 1.0);
    }
}
