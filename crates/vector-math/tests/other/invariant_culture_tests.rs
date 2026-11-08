// Vector parsing reads '.' decimals: `Vector2/3/4::parse` and the Vector2 string conversion.
// Files store numbers with a '.' whatever the user's locale, and a locale-sensitive read would
// turn "1.5" into 15 under a comma-decimal locale. Rust parses numbers with no locale at all, so
// this pins that the parse stays that way. The packed polygon string is in clipper-library's
// tests/clipper, the ASCII STL case in polygon-mesh's tests/polygon_mesh.

use solidmesh_math::{Vector2, Vector3, Vector4};

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
