// The packed polygon string case: `create_from_string` reads '.' decimals whatever the user's
// locale (a locale-sensitive read under a ',' decimal locale such as de-DE would make "1.5" 15).
// Rust parses numbers with no locale at all, so the test checks the result directly. The vector
// parse is in vector-math's tests/other, the ASCII STL case in polygon-mesh's tests/polygon_mesh.

use clipper_library::polygons_extensions::create_from_string;

#[test]
fn packed_polygon_strings_read_point_decimals() {
    let polygons = create_from_string("x:1.5,y:2.5,x:10.5,y:0.5,", 10.0).expect("parses");
    assert_eq!(polygons.len(), 1);
    assert_eq!(polygons[0][0].x, 15i64);
    assert_eq!(polygons[0][0].y, 25i64);
}
