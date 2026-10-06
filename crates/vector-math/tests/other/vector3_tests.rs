// Port of `Vector3Tests` from agg-sharp `Tests/Agg.Tests/Other/Vector3Tests.cs`: arithmetic,
// scalar multiply/divide, dot, cross, normalize and the true 3D `DistanceToSegment`.
// C#'s static `Vector3.Add` is the `+` operator and `Vector3Ex.Dot`/`Cross` are the
// `dot`/`cross` methods (see the vector-math mapping notes).

use vector_math::Vector3;

#[test]
fn vector_addition_and_subtraction() {
    let point1 = Vector3::new(1.0, 1.0, 1.0);

    let point2 = Vector3::new(2.0, 2.0, 2.0);

    let mut point3 = point1 + point2;
    assert!(point3 == Vector3::new(3.0, 3.0, 3.0));

    point3 = point1 - point2;
    assert!(point3 == Vector3::new(-1.0, -1.0, -1.0));

    point3 += point1;
    assert!(point3 == Vector3::new(0.0, 0.0, 0.0));

    point3 += point2;
    assert!(point3 == Vector3::new(2.0, 2.0, 2.0));

    point3 = Vector3::new(3.0, -4.0, 5.0);
    assert!(point3.length() > 7.07 && point3.length() < 7.08);

    let inline_op_left_side = Vector3::new(5.0, -3.0, 0.0);
    let inline_op_right_side = Vector3::new(-5.0, 4.0, 1.0);
    assert!(inline_op_left_side + inline_op_right_side == Vector3::new(0.0, 1.0, 1.0));

    assert!(inline_op_left_side - inline_op_right_side == Vector3::new(10.0, -7.0, -1.0));
}

#[test]
fn scalar_multiplication() {
    let scalar_multiplication_argument = Vector3::new(5.0, 4.0, 3.0);
    assert!(scalar_multiplication_argument * -0.5 == -Vector3::new(2.5, 2.0, 1.5));
    assert!(-0.5 * scalar_multiplication_argument == -Vector3::new(2.5, 2.0, 1.5));
    assert!(5.0 * scalar_multiplication_argument == Vector3::new(25.0, 20.0, 15.0));

    let mut point3 = Vector3::new(2.0, 3.0, 4.0);
    point3 *= 6.0;
    assert!(point3.equals_within(Vector3::new(12.0, 18.0, 24.0), 0.01f32 as f64));
}

#[test]
fn scalar_division() {
    let scalar_multiplication_argument = Vector3::new(5.0, 4.0, 3.0);
    assert!(scalar_multiplication_argument / 2.0 == Vector3::new(2.5, 2.0, 1.5));

    let mut point3 = Vector3::new(12.0, 18.0, 24.0);
    point3 /= 6.0;
    assert!(point3.equals_within(Vector3::new(2.0, 3.0, 4.0), 0.01f32 as f64));
}

#[test]
fn dot_product() {
    let test1 = Vector3::new(10.0, 1.0, 2.0);
    let test2 = Vector3::new(1.0, 0.0, 0.0);
    let dot_result = test2.dot(test1);
    assert!(dot_result == 10.0);
}

#[test]
fn cross_product() {
    let test1 = Vector3::new(10.0, 0.0, 0.0);
    let test2 = Vector3::new(1.0, 1.0, 0.0);
    let cross_result = test2.cross(test1);
    assert!(cross_result.x == 0.0);
    assert!(cross_result.y == 0.0);
    assert!(cross_result.z < 0.0);
}

#[test]
fn normalize() {
    let mut point3 = Vector3::new(3.0, -4.0, 5.0);
    point3.normalize();
    assert!(point3.length() > 0.99 && point3.length() < 1.01);
}

fn assert_within(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-9,
        "expected {expected} within 1e-9, got {actual}"
    );
}

#[test]
fn distance_to_segment_is_true_3d_distance() {
    let start = Vector3::new(0.0, 0.0, 0.0);
    let end = Vector3::new(10.0, 0.0, 0.0);

    // Off the segment along Z - the offset the old +Z perpendicular ignored.
    assert_within(
        Vector3::new(5.0, 0.0, 3.0).distance_to_segment(start, end),
        3.0,
    );
    // Off in Y, and off in both Y and Z.
    assert_within(
        Vector3::new(5.0, 4.0, 0.0).distance_to_segment(start, end),
        4.0,
    );
    assert_within(
        Vector3::new(5.0, 3.0, 4.0).distance_to_segment(start, end),
        5.0,
    );
    // Beyond each end measures to that end point.
    assert_within(
        Vector3::new(-3.0, 0.0, 4.0).distance_to_segment(start, end),
        5.0,
    );
    assert_within(
        Vector3::new(13.0, 4.0, 0.0).distance_to_segment(start, end),
        5.0,
    );
    // Exactly on the segment, including at its end.
    assert_within(
        Vector3::new(7.0, 0.0, 0.0).distance_to_segment(start, end),
        0.0,
    );
    assert_within(
        Vector3::new(10.0, 0.0, 0.0).distance_to_segment(start, end),
        0.0,
    );
    // A segment in general position.
    let slanted = Vector3::new(0.0, 10.0, 10.0);
    assert_within(
        Vector3::new(3.0, 5.0, 5.0).distance_to_segment(start, slanted),
        3.0,
    );
    // A zero-length segment is the distance to its point.
    assert_within(
        Vector3::new(1.0, 2.0, 2.0).distance_to_segment(start, start),
        3.0,
    );
}
