//! Rust-only unit tests for `number_format`: the expected strings are what .NET 10's
//! `double.ToString()`, `float.ToString()`, `ToString("0.####")` and the standard "F"/"N"/"P" formats
//! produce (InvariantCulture).

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

#[test]
fn rust_only_try_parse_f64_reports_failure() {
    assert_eq!(try_parse_f64(" 12.5 "), Some(12.5));
    assert_eq!(try_parse_f64("0"), Some(0.0));
    assert_eq!(try_parse_f64("12,5"), None);
    assert_eq!(try_parse_f64("wide"), None);
    assert_eq!(try_parse_f64(""), None);
}

/// .NET 10's invariant `double.TryParse` rejects the "∞" sign (its symbol is "Infinity").
#[test]
fn rust_only_try_parse_f64_rejects_the_infinity_sign() {
    for text in ["∞", "-∞", "+∞", " ∞ "] {
        assert_eq!(try_parse_f64(text), None, "{text:?}");
    }
    assert_eq!(try_parse_f64(" -INFINITY "), Some(f64::NEG_INFINITY));
}

#[test]
fn rust_only_format_round_trip_ties_round_half_to_even() {
    // A value whose shortest digits sit exactly halfway between two
    // candidates: .NET 10 picks the even last digit, Rust's shortest formatter rounds up. These
    // values occur in MatterCAD's golden .mcx files (a float widened to double).
    assert_eq!(format_f64(-159.60000610351562), "-159.60000610351562");
    assert_eq!(format_f64(-14.097000122070312), "-14.097000122070312");
    assert_eq!(format_f64(1.0000000000000002), "1.0000000000000002");
    assert_eq!(format_f64(0.30000001192092896), "0.30000001192092896");
    // The same tie at 16 digits (MatterCAD's 3MF writer, test-data/formats).
    assert_eq!(format_f64(-8.847000122070312), "-8.847000122070312");
    assert_eq!(format_f64(9.583999633789062), "9.583999633789062");
}

/// Expected text read from .NET 10's `double.ToString("G17", InvariantCulture)`.
#[test]
fn rust_only_format_f64_g17_matches_dotnet() {
    let cases = [
        (0.1, "0.10000000000000001"),
        (1.0, "1"),
        (20.0, "20"),
        (35.4, "35.399999999999999"),
        (1e17, "1E+17"),
        (1.5e17, "1.5E+17"),
        (1e16, "10000000000000000"),
        (123456789012345678.0, "1.2345678901234568E+17"),
        (0.0001, "0.0001"),
        (0.00001, "1.0000000000000001E-05"),
        (1.0 / 3.0, "0.33333333333333331"),
        (-0.0, "-0"),
        (-2.5, "-2.5"),
        (1e-300, "1E-300"),
        (f64::MAX, "1.7976931348623157E+308"),
        (f64::INFINITY, "Infinity"),
        (f64::NAN, "NaN"),
        (100.0, "100"),
    ];
    for (value, expected) in cases {
        assert_eq!(format_f64_g17(value), expected, "{value}");
    }
}

/// Each row: value, then .NET 10's F20, F0, F2, N2, N0, P0, P1, P2 (InvariantCulture).
const STANDARD_ROWS: &[(f64, [&str; 8])] = &[
    (
        0.1,
        [
            "0.10000000000000000555",
            "0",
            "0.10",
            "0.10",
            "0",
            "10 %",
            "10.0 %",
            "10.00 %",
        ],
    ),
    (
        1e23,
        [
            "99999999999999991611392.00000000000000000000",
            "99999999999999991611392",
            "99999999999999991611392.00",
            "99,999,999,999,999,991,611,392.00",
            "99,999,999,999,999,991,611,392",
            "9,999,999,999,999,999,161,139,200 %",
            "9,999,999,999,999,999,161,139,200.0 %",
            "9,999,999,999,999,999,161,139,200.00 %",
        ],
    ),
    (
        -1234.5,
        [
            "-1234.50000000000000000000",
            "-1234",
            "-1234.50",
            "-1,234.50",
            "-1,234",
            "-123,450 %",
            "-123,450.0 %",
            "-123,450.00 %",
        ],
    ),
    (
        0.0,
        [
            "0.00000000000000000000",
            "0",
            "0.00",
            "0.00",
            "0",
            "0 %",
            "0.0 %",
            "0.00 %",
        ],
    ),
    (
        -0.0,
        [
            "-0.00000000000000000000",
            "-0",
            "-0.00",
            "-0.00",
            "-0",
            "-0 %",
            "-0.0 %",
            "-0.00 %",
        ],
    ),
    (
        0.0135,
        [
            "0.01349999999999999985",
            "0",
            "0.01",
            "0.01",
            "0",
            "1 %",
            "1.3 %",
            "1.35 %",
        ],
    ),
    (
        2.675,
        [
            "2.67499999999999982236",
            "3",
            "2.67",
            "2.67",
            "3",
            "267 %",
            "267.5 %",
            "267.50 %",
        ],
    ),
    (
        0.125,
        [
            "0.12500000000000000000",
            "0",
            "0.12",
            "0.12",
            "0",
            "12 %",
            "12.5 %",
            "12.50 %",
        ],
    ),
    (
        1e-7,
        [
            "0.00000010000000000000",
            "0",
            "0.00",
            "0.00",
            "0",
            "0 %",
            "0.0 %",
            "0.00 %",
        ],
    ),
    (
        12345.678,
        [
            "12345.67799999999988358468",
            "12346",
            "12345.68",
            "12,345.68",
            "12,346",
            "1,234,568 %",
            "1,234,567.8 %",
            "1,234,567.80 %",
        ],
    ),
    (
        -0.5,
        [
            "-0.50000000000000000000",
            "-0",
            "-0.50",
            "-0.50",
            "-0",
            "-50 %",
            "-50.0 %",
            "-50.00 %",
        ],
    ),
    (
        0.995,
        [
            "0.99499999999999999556",
            "1",
            "0.99",
            "0.99",
            "1",
            "99 %",
            "99.5 %",
            "99.50 %",
        ],
    ),
    (
        1.5,
        [
            "1.50000000000000000000",
            "2",
            "1.50",
            "1.50",
            "2",
            "150 %",
            "150.0 %",
            "150.00 %",
        ],
    ),
    (
        999.9999,
        [
            "999.99990000000002510205",
            "1000",
            "1000.00",
            "1,000.00",
            "1,000",
            "100,000 %",
            "100,000.0 %",
            "99,999.99 %",
        ],
    ),
];

#[test]
fn rust_only_standard_formats_match_dotnet() {
    for (value, expected) in STANDARD_ROWS {
        let actual = [
            format_f64_fixed(*value, 20),
            format_f64_fixed(*value, 0),
            format_f64_fixed(*value, 2),
            format_f64_number(*value, 2),
            format_f64_number(*value, 0),
            format_f64_percent(*value, 0),
            format_f64_percent(*value, 1),
            format_f64_percent(*value, 2),
        ];
        assert_eq!(&actual.map(|s| s), expected, "{value}");
    }
    // P1 reads the exact value, not value * 100 (0.0105 * 100 is 1.0500000000000000444).
    assert_eq!(format_f64_percent(0.0105, 1), "1.1 %");
    assert_eq!(format_f64_percent(0.0015, 1), "0.2 %");
    assert_eq!(format_f64_percent(-0.004, 1), "-0.4 %");
    assert_eq!(format_f64_fixed(0.375, 2), "0.38");
    assert_eq!(format_f64_number(2.5, 0), "2");
    assert_eq!(format_f64_number(3.5, 0), "4");
    assert_eq!(format_f64_fixed(f64::NAN, 2), "NaN");
    assert_eq!(format_f64_fixed(f64::INFINITY, 2), "Infinity");
    assert_eq!(format_f64_percent(f64::NEG_INFINITY, 1), "-Infinity");
    assert_eq!(format_f64_number(f64::NAN, 0), "NaN");
    for (value, expected) in [
        (0, "0"),
        (999, "999"),
        (1000, "1,000"),
        (-1234567, "-1,234,567"),
        (i64::MIN, "-9,223,372,036,854,775,808"),
        (i64::MAX, "9,223,372,036,854,775,807"),
    ] {
        assert_eq!(format_i64_number(value), expected);
    }
}

#[test]
fn custom_scientific_e0_matches_dotnet() {
    // Expected strings from .NET 10's `x.ToString("0.###e0", CultureInfo.InvariantCulture)`.
    for (value, expected) in [
        (0.0, "0e0"),
        (-0.0, "0e0"),
        (1.0, "1e0"),
        (-1.0, "-1e0"),
        (1.2345, "1.235e0"),
        (1.2355, "1.236e0"),
        (1.0005, "1.001e0"),
        (1.0015, "1.002e0"),
        (9.9995, "1e1"),
        (9.99951, "1e1"),
        (0.000123456, "1.235e-4"),
        (-2.5e-7, "-2.5e-7"),
        (12345.678, "1.235e4"),
        (1e300, "1e300"),
        (1.5e-320, "1.5e-320"),
        (f64::NAN, "NaN"),
        (f64::INFINITY, "Infinity"),
        (f64::NEG_INFINITY, "-Infinity"),
        (0.1, "1e-1"),
        (100.0, "1e2"),
        (1.23e-5, "1.23e-5"),
        (0.30000000000000004, "3e-1"),
        (2.0004999999999997, "2.001e0"),
        (1.2345678901234568e17, "1.235e17"),
        (4.4999999999999996e-5, "4.5e-5"),
    ] {
        assert_eq!(format_f64_scientific_e0(value, 3), expected, "{value:?}");
    }
}

#[test]
fn custom_scientific_e0_reads_fifteen_digits_not_the_shortest() {
    // .NET 10, `ToString("0.################e0")` and `("0.##############e0")`: values whose
    // shortest round-trip text has more than 15 digits print from their 15-digit rounding.
    for (value, expected) in [
        (0.1 + 0.2, "3e-1"),
        (1.0000000000000002, "1e0"),
        (1.2345678901234567, "1.23456789012346e0"),
        (9.999999999999998, "1e1"),
        (123.45678901234567, "1.23456789012346e2"),
    ] {
        assert_eq!(format_f64_scientific_e0(value, 16), expected, "{value:?}");
        assert_eq!(format_f64_scientific_e0(value, 14), expected, "{value:?}");
    }
}
