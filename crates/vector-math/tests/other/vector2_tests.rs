// Vector2 basics: arithmetic, length, polyline lengths and positions (`vector2_ex`), and
// cross/dot agreeing with Vector3.
//
// The inputs are random but drawn from a seeded `DotNetRandom`, so a failure reproduces. The
// assertions hold for any inputs.

use vector_math::dotnet_random::DotNetRandom;
use vector_math::vector2_ex::{get_position_at, polygon_length};
use vector_math::{Vector2, Vector3};

#[test]
fn arithmatic_operations() {
    let point1 = Vector2::new(1.0, 1.0);
    let point2 = Vector2::new(2.0, 2.0);

    let mut point3 = point1 + point2;
    assert!(point3 == Vector2::new(3.0, 3.0));

    point3 = point1 - point2;
    assert!(point3 == Vector2::new(-1.0, -1.0));

    point3 += point1;
    assert!(point3 == Vector2::new(0.0, 0.0));

    point3 += point2;
    assert!(point3 == Vector2::new(2.0, 2.0));

    point3 *= 6.0;
    assert!(point3 == Vector2::new(12.0, 12.0));

    let inline_op_left_side = Vector2::new(5.0, -3.0);
    let inline_op_right_side = Vector2::new(-5.0, 4.0);
    assert!(inline_op_left_side + inline_op_right_side == Vector2::new(0.0, 1.0));

    assert!(inline_op_left_side - inline_op_right_side == Vector2::new(10.0, -7.0));
}

#[test]
fn get_length_and_normalize() {
    let mut point3 = Vector2::new(3.0, -4.0);
    assert!(point3.length() > 4.999f32 as f64 && point3.length() < 5.001f32 as f64);

    point3.normalize();
    assert!(point3.length() > 0.99f32 as f64 && point3.length() < 1.01f32 as f64);
}

#[test]
fn get_position_at_tests() {
    let line1 = vec![
        Vector2::new(10.0, 3.0),
        Vector2::new(20.0, 3.0),
        Vector2::new(20.0, 13.0),
        Vector2::new(10.0, 13.0),
    ];

    assert_eq!(polygon_length(&line1, false), 30.0);

    // open segments should also give correct values
    assert_eq!(get_position_at(&line1, 3.0, false), Vector2::new(13.0, 3.0));
    assert_eq!(
        get_position_at(&line1, 33.0, false),
        Vector2::new(10.0, 13.0)
    );
    assert_eq!(
        get_position_at(&line1, (33 + 22 * 10) as f64, false),
        Vector2::new(10.0, 13.0)
    );
    assert_eq!(
        get_position_at(&line1, -2.0, false),
        Vector2::new(10.0, 3.0)
    );
    assert_eq!(
        get_position_at(&line1, (-2 + -23 * 10) as f64, false),
        Vector2::new(10.0, 3.0)
    );

    assert_eq!(polygon_length(&line1, true), 40.0);

    // closed loops should wrap correctly
    let error = 0.000001;
    assert_eq!(get_position_at(&line1, 3.0, true), Vector2::new(13.0, 3.0));
    assert!(Vector2::new(13.0, 3.0).equals_within(get_position_at(&line1, 43.0, true), error));
    assert!(Vector2::new(13.0, 3.0)
        .equals_within(get_position_at(&line1, (43 + 22 * 40) as f64, true), error));
    assert!(Vector2::new(10.0, 5.0).equals_within(get_position_at(&line1, -2.0, true), error));
    assert!(Vector2::new(10.0, 5.0)
        .equals_within(get_position_at(&line1, (-2 + 23 * 40) as f64, true), error));
}

#[test]
fn scaler_operations() {
    let scalar_multiplication_argument = Vector2::new(5.0f32 as f64, 4.0f32 as f64);
    assert!(scalar_multiplication_argument * -0.5 == Vector2::new(-2.5f32 as f64, -2.0));
    assert!(scalar_multiplication_argument / 2.0 == Vector2::new(2.5, 2.0));
    assert!(2.0 / scalar_multiplication_argument == Vector2::new(0.4, 0.5));
    assert!(5.0 * scalar_multiplication_argument == Vector2::new(25.0, 20.0));
}

fn random_vector(rand: &mut DotNetRandom) -> Vector2 {
    let x = rand.next_double() * 1000.0;
    let y = rand.next_double() * 1000.0;
    Vector2::new(x, y)
}

#[test]
fn cross_product() {
    let mut rand = DotNetRandom::new(1);
    let test_vector_2d1 = random_vector(&mut rand);
    let test_vector_2d2 = random_vector(&mut rand);
    let cross_2d = test_vector_2d1.cross(test_vector_2d2);

    let test_vector31 = Vector3::new(test_vector_2d1.x, test_vector_2d1.y, 0.0);
    let test_vector32 = Vector3::new(test_vector_2d2.x, test_vector_2d2.y, 0.0);
    let cross_3d = test_vector31.cross(test_vector32);

    assert!(cross_3d.z == cross_2d);
}

#[test]
fn dot_product() {
    let mut rand = DotNetRandom::new(2);
    let test_vector_2d1 = random_vector(&mut rand);
    let test_vector_2d2 = random_vector(&mut rand);
    let cross_2d = test_vector_2d1.dot(test_vector_2d2);

    let test_vector31 = Vector3::new(test_vector_2d1.x, test_vector_2d1.y, 0.0);
    let test_vector32 = Vector3::new(test_vector_2d2.x, test_vector_2d2.y, 0.0);
    let cross_3d = test_vector31.dot(test_vector32);

    assert!(cross_3d == cross_2d);
}

#[test]
fn length_and_distance() {
    let mut rand = DotNetRandom::new(3);
    let test1 = random_vector(&mut rand);
    let test2 = random_vector(&mut rand);
    let test3 = test1 + test2;
    let distance1 = test2.length();
    let distance2 = (test1 - test3).length();

    assert!(distance1 < distance2 + 0.001f32 as f64 && distance1 > distance2 - 0.001f32 as f64);
}
