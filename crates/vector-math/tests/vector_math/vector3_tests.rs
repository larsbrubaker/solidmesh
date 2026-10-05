// Port of `Vector3Tests` (agg-sharp VectorMathTests.cs): the tests that cover the vector
// types. The rest of that class (Vector2/Vector2Float/Vector3Float/Vector3 SerializeTest,
// WorldView*, WorldUnitsPerPixelIsUnclampedAtRoomScale, Frustum*, PlaneClipLineTests,
// TestGetHashCode) needs Matrix4X4, Plane, Frustum, WorldView or a JSON shape decision and
// joins in Phase 1b.

use vector_math::Vector3;

#[test]
fn static_function_tests() {
    assert!(Vector3::collinear(
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(0.0, 0.0, 3.0)
    ));
    assert!(!Vector3::collinear(
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(0.0, 1.0, 3.0)
    ));
}

#[test]
fn vector3_parse_test() {
    // Two segments
    assert_eq!(Vector3::parse("1, 2"), Vector3::new(1.0, 2.0, 0.0));

    // Two segments, trailing comma
    assert_eq!(Vector3::parse("1, 2,"), Vector3::new(1.0, 2.0, 0.0));

    // Three segments with whitespace
    assert_eq!(Vector3::parse("1, 2, 3"), Vector3::new(1.0, 2.0, 3.0));

    // Three segments without whitespace
    assert_eq!(Vector3::parse("1,2,3"), Vector3::new(1.0, 2.0, 3.0));

    // Doubles
    assert_eq!(Vector3::parse("1.1, 2.2, 3.3"), Vector3::new(1.1, 2.2, 3.3));

    // Negative values
    assert_eq!(Vector3::parse("-1, -2, -3"), Vector3::new(-1.0, -2.0, -3.0));

    // Empty
    assert_eq!(Vector3::parse(""), Vector3::ZERO);
}
