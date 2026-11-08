//! Rust-only tests for `number_format::format_f64_max_decimals` against .NET's custom format
//! `"0.#########"` (InvariantCulture), the values and texts read off `dotnet` (.NET 10): ties at the
//! last kept decimal round AWAY from zero on the 15-significant-digit value, a negative value that
//! rounds to zero keeps its sign, and the infinities and NaN are spelled as .NET spells them.

use solidmesh_math::number_format::format_f64_max_decimals;

#[test]
fn rust_only_nine_places_write_the_pinned_text() {
    let table: [(f64, &str); 16] = [
        (0.0000000005, "0.000000001"),
        (0.0000000015, "0.000000002"),
        (0.0000000025, "0.000000003"),
        (1.0000000005, "1.000000001"),
        (2.5e-10, "0"),
        (0.1234567895, "0.12345679"),
        (0.1234567885, "0.123456789"),
        (1e-10, "0"),
        (-1e-10, "-0"),
        (-0.0000000005, "-0.000000001"),
        (12.3456789012345, "12.345678901"),
        (f64::INFINITY, "Infinity"),
        (f64::NEG_INFINITY, "-Infinity"),
        (f64::NAN, "NaN"),
        (0.000000000499999999, "0"),
        (3.0000000015, "3.000000002"),
    ];

    for (value, expected) in table {
        assert_eq!(format_f64_max_decimals(value, 9), expected, "{value:e}");
    }
}
