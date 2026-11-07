//! Unit tests for the STL number formatting in `stl_writer.rs`. Expected values are the
//! reference "0.000000" invariant-culture formatting of each value.

use super::*;

#[test]
fn rust_only_format_fixed6_writes_six_places() {
    assert_eq!(format_fixed6(0.0, 7), "0.000000");
    assert_eq!(format_fixed6(1.0, 7), "1.000000");
    assert_eq!(format_fixed6(10.0, 7), "10.000000");
    assert_eq!(format_fixed6(-2.5, 7), "-2.500000");
    // 123.456789f is 123.45678710937500 exactly; 7 significant digits first gives 123.4568.
    assert_eq!(format_fixed6(f64::from(123.456_79_f32), 7), "123.456800");
    // 0.1f is 0.100000001490116...; 7 significant digits first gives 0.1000000.
    assert_eq!(format_fixed6(f64::from(0.1_f32), 7), "0.100000");
    assert_eq!(format_fixed6(f64::from(0.012_345_678_f32), 7), "0.012346");
    // 99999.9921875 to 7 significant digits is 99999.99.
    assert_eq!(format_fixed6(f64::from(99_999.99_f32), 7), "99999.990000");
    // A carry out of the first digit at the significant-digit rounding.
    assert_eq!(format_fixed6(0.999_999_999_999_999_9, 15), "1.000000");
    assert_eq!(format_fixed6(0.0000004, 7), "0.000000");
    assert_eq!(format_fixed6(0.0000005, 15), "0.000001");
    assert_eq!(format_fixed6(1234567.0, 15), "1234567.000000");
}
