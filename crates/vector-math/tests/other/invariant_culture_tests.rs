// Ports agg-sharp `Tests/Agg.Tests/Other/InvariantCultureTests.cs`, its vector parse case:
// `Vector2/3/4.Parse` and the Vector2 type converter read '.' decimals. C# runs it under de-DE,
// where a culture-sensitive read made "1.5" 15; Rust parses numbers with no culture at all, so the
// test is the same check without the culture scope. The packed polygon string is in agg-path's
// tests/other, the ASCII STL case in polygon-mesh's tests/polygon_mesh.

use vector_math::{Vector2, Vector3, Vector4};

#[test]
fn vector_parsing_reads_point_decimals() {
    assert_eq!(Vector2::parse("1.5,2.5"), Vector2::new(1.5, 2.5));
    assert_eq!(Vector3::parse("1.5,2.5,3.5"), Vector3::new(1.5, 2.5, 3.5));
    assert_eq!(
        Vector4::parse("1.5,2.5,3.5,4.5"),
        Vector4::new(1.5, 2.5, 3.5, 4.5)
    );

    let converted = Vector2::convert_from_string("(1.5,2.5)");
    assert_eq!(converted, Some(Vector2::new(1.5, 2.5)));
}
