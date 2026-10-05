//! Rust-only unit tests for `number_format`: the expected strings are what .NET 10's
//! `double.ToString()`, `float.ToString()` and `ToString("0.####")` produce (InvariantCulture).

use super::*;

#[test]
fn rust_only_format_f64_matches_dotnet_round_trip() {
    assert_eq!(format_f64(1.0), "1");
    assert_eq!(format_f64(-2.5), "-2.5");
    assert_eq!(format_f64(0.1), "0.1");
    assert_eq!(format_f64(1.0 / 3.0), "0.3333333333333333");
    assert_eq!(format_f64(0.0001), "0.0001");
    assert_eq!(format_f64(0.00001), "1E-05");
    assert_eq!(format_f64(1e14), "100000000000000");
    assert_eq!(format_f64(1e15), "1E+15");
    assert_eq!(format_f64(1.5e300), "1.5E+300");
    assert_eq!(format_f64(-0.0), "-0");
    assert_eq!(format_f64(0.0), "0");
    assert_eq!(format_f64(f64::NAN), "NaN");
    assert_eq!(format_f64(f64::INFINITY), "Infinity");
    assert_eq!(format_f64(f64::NEG_INFINITY), "-Infinity");
}

#[test]
fn rust_only_format_f32_matches_dotnet_round_trip() {
    assert_eq!(format_f32(1.1), "1.1");
    assert_eq!(format_f32(1234567.0), "1234567");
    assert_eq!(format_f32(1e7), "1E+07");
    assert_eq!(format_f32(0.3), "0.3");
}

#[test]
fn rust_only_format_max_decimals_matches_dotnet_custom_format() {
    assert_eq!(format_f64_max_decimals(1.0, 4), "1");
    assert_eq!(format_f64_max_decimals(1.23456, 4), "1.2346");
    assert_eq!(format_f64_max_decimals(0.00004, 4), "0");
    assert_eq!(format_f64_max_decimals(0.00005, 4), "0.0001");
    assert_eq!(format_f64_max_decimals(9.99996, 4), "10");
    assert_eq!(format_f64_max_decimals(-1.5, 4), "-1.5");
    assert_eq!(format_f64_max_decimals(123456.0, 4), "123456");
    // Half away from zero on the 15-digit value, not half to even.
    assert_eq!(format_f64_max_decimals(0.03125, 4), "0.0313");
}

#[test]
fn rust_only_parse_f64_matches_dotnet_try_parse() {
    assert_eq!(parse_f64(" 1.5 "), 1.5);
    assert_eq!(parse_f64("-2"), -2.0);
    assert_eq!(parse_f64("+3e2"), 300.0);
    assert_eq!(parse_f64(".5"), 0.5);
    assert_eq!(parse_f64(""), 0.0);
    assert_eq!(parse_f64("abc"), 0.0);
    assert_eq!(parse_f64("inf"), 0.0);
    assert_eq!(parse_f64("-Infinity"), f64::NEG_INFINITY);
    assert!(parse_f64("NaN").is_nan());
}
