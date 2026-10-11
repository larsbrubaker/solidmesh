// Ports agg-sharp `Tests/Agg.Tests/Other/InvariantCultureTests.cs`, its packed polygon string
// case: `PolygonsExtensions.CreateFromString` reads '.' decimals. C# runs it under de-DE, where a
// culture-sensitive read made "1.5" 15; Rust parses numbers with no culture at all, so the test is
// the same check without the culture scope. The vector parse is in vector-math's tests/other, the
// ASCII STL case in polygon-mesh's tests/polygon_mesh.

use agg_path::clipper_library::polygons_extensions::create_from_string;

#[test]
fn packed_polygon_strings_read_point_decimals() {
    let polygons = create_from_string("x:1.5,y:2.5,x:10.5,y:0.5,", 10.0).expect("parses");
    assert_eq!(polygons.len(), 1);
    assert_eq!(polygons[0][0].x, 15i64);
    assert_eq!(polygons[0][0].y, 25i64);
}
