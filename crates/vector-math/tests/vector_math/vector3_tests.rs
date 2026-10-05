// Port of `Vector3Tests` (agg-sharp VectorMathTests.cs): the vector, JSON, frustum, plane and
// hash tests. The rest of that class (WorldView*, WorldUnitsPerPixelIsUnclampedAtRoomScale)
// needs WorldView and joins in Phase 1c. `TestGetHashCode` hashes through `std::hash::Hash`
// where the C# calls `GetHashCode()`; its assertions are unchanged.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use vector_math::{
    math_helper, AxisAlignedBoundingBox, Frustum, FrustumIntersection, Matrix4X4, Plane, Vector2,
    Vector2Float, Vector3, Vector3Float, Vector4,
};

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

#[test]
fn vector2_serialize_test() {
    assert_eq!(
        serde_json::to_string(&Vector2::new(1.0, 2.0)).unwrap(),
        "{\"X\":1.0,\"Y\":2.0}"
    );
}

#[test]
fn vector2_float_serialize_test() {
    assert_eq!(
        serde_json::to_string(&Vector2Float::new(1.1, 2.2)).unwrap(),
        "{\"X\":1.1,\"Y\":2.2}"
    );
}

#[test]
fn vector3_float_serialize_test() {
    assert_eq!(
        serde_json::to_string(&Vector3Float::new(1.1, 2.2, 3.3)).unwrap(),
        "{\"X\":1.1,\"Y\":2.2,\"Z\":3.3}"
    );
}

#[test]
fn vector3_serialize_test() {
    assert_eq!(
        serde_json::to_string(&Vector3::new(1.0, 2.0, 3.0)).unwrap(),
        "{\"X\":1.0,\"Y\":2.0,\"Z\":3.0}"
    );
}

#[test]
fn frustum_extraction_tests() {
    {
        let perspective_matrix =
            Matrix4X4::create_perspective_field_of_view(math_helper::TAU / 4.0, 1.0, 3.0, 507.0)
                .unwrap();
        let frustum = Frustum::frustum_from_projection_matrix(perspective_matrix);

        // left
        assert!(frustum.planes[0]
            .normal
            .equals_within(Vector3::new(1.0, 0.0, -1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[0].distance_from_origin, 0.0);
        // right
        assert!(frustum.planes[1]
            .normal
            .equals_within(Vector3::new(-1.0, 0.0, -1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[1].distance_from_origin, 0.0);
        // bottom
        assert!(frustum.planes[2]
            .normal
            .equals_within(Vector3::new(0.0, 1.0, -1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[2].distance_from_origin, 0.0);
        // top
        assert!(frustum.planes[3]
            .normal
            .equals_within(Vector3::new(0.0, -1.0, -1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[3].distance_from_origin, 0.0);
        // near
        assert!(frustum.planes[4]
            .normal
            .equals_within(Vector3::new(0.0, 0.0, -1.0), 0.0001));
        assert_within(frustum.planes[4].distance_from_origin, 3.0, 0.0001);
        // far
        assert!(frustum.planes[5]
            .normal
            .equals_within(Vector3::new(0.0, 0.0, 1.0), 0.0001));
        assert_within(frustum.planes[5].distance_from_origin, -507.0, 0.0001);
    }
}

/// TUnit's `IsEqualTo(expected).Within(tolerance)`.
pub fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not within {tolerance} of {expected}"
    );
}

/// The C# public helper `TestFrustumClipLine`.
pub fn test_frustum_clip_line(
    frustum: &Frustum,
    line_start: Vector3,
    line_end: Vector3,
    expect_in_frustum: bool,
    start_clipped: Option<Vector3>,
    end_clipped: Option<Vector3>,
) {
    let mut line_start = line_start;
    let mut line_end = line_end;
    let in_frustum = frustum.clip_line(&mut line_start, &mut line_end);
    assert_eq!(in_frustum, expect_in_frustum);
    if expect_in_frustum {
        assert!(line_start.equals_within(start_clipped.unwrap(), 0.0001));
        assert!(line_end.equals_within(end_clipped.unwrap(), 0.001));
    }
}

fn aabb(min: [f64; 3], max: [f64; 3]) -> AxisAlignedBoundingBox {
    AxisAlignedBoundingBox::new(
        Vector3::new(min[0], min[1], min[2]),
        Vector3::new(max[0], max[1], max[2]),
    )
}

fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

#[test]
fn frustum_interset_aabb_tests() {
    {
        // A box this is 40 units on each side centered at the origin
        let frustum = Frustum::from_planes(
            Plane::new(v3(1.0, 0.0, 0.0), 20.0),
            Plane::new(v3(-1.0, 0.0, 0.0), 20.0),
            Plane::new(v3(0.0, 1.0, 0.0), 20.0),
            Plane::new(v3(0.0, -1.0, 0.0), 20.0),
            Plane::new(v3(0.0, 0.0, 1.0), 20.0),
            Plane::new(v3(0.0, 0.0, -1.0), 20.0),
        );

        // outside to left
        let intersection = frustum.get_intersect(&aabb([-30.0, -10.0, -10.0], [-25.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // intersect
        let intersection = frustum.get_intersect(&aabb([-25.0, 0.0, -10.0], [-15.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // intersect 2 // A thin bar at the center crossing both x planes
        let intersection = frustum.get_intersect(&aabb([-25.0, -5.0, -5.0], [25.0, 5.0, 5.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // not intersect
        let intersection = frustum.get_intersect(&aabb([-25.0, 0.0, 30.0], [-15.0, 10.0, 35.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // inside
        let intersection = frustum.get_intersect(&aabb([-5.0, -5.0, -5.0], [5.0, 5.0, 5.0]));
        assert!(intersection == FrustumIntersection::Inside);
    }

    {
        let frustum = Frustum::from_planes(
            Plane::new(v3(-1.0, -1.0, 0.0), 0.0),
            Plane::new(v3(1.0, -1.0, 0.0), 0.0),
            Plane::new(v3(0.0, -1.0, -1.0), 0.0),
            Plane::new(v3(0.0, -1.0, 1.0), 0.0),
            Plane::new(v3(0.0, -1.0, 0.0), 0.0),
            Plane::new(v3(0.0, 1.0, 0.0), 10000.0),
        );

        // outside to left
        let intersection = frustum.get_intersect(&aabb([-110.0, 0.0, -10.0], [-100.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // intersect with origin (front)
        let intersection = frustum.get_intersect(&aabb([-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // inside
        let intersection = frustum.get_intersect(&aabb([-5.0, 100.0, -5.0], [5.0, 110.0, 5.0]));
        assert!(intersection == FrustumIntersection::Inside);
    }

    {
        // looking down -z
        let frustum5_plane_neg_z = Frustum::from_normals(
            v3(-1.0, 0.0, 1.0),
            v3(-1.0, 0.0, 1.0),
            v3(0.0, 1.0, 1.0),
            v3(0.0, -1.0, 1.0),
            v3(0.0, 0.0, -1.0),
            10000.0,
        );

        // outside to left
        let intersection =
            frustum5_plane_neg_z.get_intersect(&aabb([-110.0, 0.0, -10.0], [-100.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Outside);

        // intersect with origin (front)
        let intersection =
            frustum5_plane_neg_z.get_intersect(&aabb([-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]));
        assert!(intersection == FrustumIntersection::Intersect);

        // inside
        let intersection =
            frustum5_plane_neg_z.get_intersect(&aabb([-5.0, -5.0, -110.0], [5.0, 5.0, -100.0]));
        assert!(intersection == FrustumIntersection::Inside);
    }
}

#[test]
fn frustum_transform_tests() {
    {
        let frustum = Frustum::from_planes(
            Plane::new(v3(1.0, 0.0, 0.0), -20.0),
            Plane::new(v3(-1.0, 0.0, 0.0), -20.0),
            Plane::new(v3(0.0, 1.0, 0.0), -20.0),
            Plane::new(v3(0.0, -1.0, 0.0), -20.0),
            Plane::new(v3(0.0, 0.0, 1.0), -20.0),
            Plane::new(v3(0.0, 0.0, -1.0), -20.0),
        );

        test_frustum_clip_line(
            &frustum,
            v3(-5.0, 0.0, 0.0),
            v3(5.0, 0.0, 0.0),
            true,
            Some(v3(-5.0, 0.0, 0.0)),
            Some(v3(5.0, 0.0, 0.0)),
        );
        test_frustum_clip_line(
            &frustum,
            v3(-50.0, 0.0, 0.0),
            v3(-21.0, 0.0, 0.0),
            false,
            None,
            None,
        );
        test_frustum_clip_line(
            &frustum,
            v3(-50.0, 0.0, 0.0),
            v3(-19.0, 0.0, 0.0),
            true,
            Some(v3(-20.0, 0.0, 0.0)),
            Some(v3(-19.0, 0.0, 0.0)),
        );

        // moved right
        {
            let moved_right_frustum =
                Frustum::transform(&frustum, Matrix4X4::create_translation(10.0, 0.0, 0.0));
            assert!(moved_right_frustum.planes[0].equals(&Plane::new(v3(1.0, 0.0, 0.0), -10.0)));
            assert!(moved_right_frustum.planes[1].equals(&Plane::new(v3(-1.0, 0.0, 0.0), -30.0)));
            assert!(moved_right_frustum.planes[2].equals(&frustum.planes[2]));
            assert!(moved_right_frustum.planes[3].equals(&frustum.planes[3]));
            assert!(moved_right_frustum.planes[4].equals(&frustum.planes[4]));
            assert!(moved_right_frustum.planes[5].equals(&frustum.planes[5]));

            test_frustum_clip_line(
                &moved_right_frustum,
                v3(-5.0, 0.0, 0.0),
                v3(5.0, 0.0, 0.0),
                true,
                Some(v3(-5.0, 0.0, 0.0)),
                Some(v3(5.0, 0.0, 0.0)),
            );
            test_frustum_clip_line(
                &moved_right_frustum,
                v3(-50.0, 0.0, 0.0),
                v3(-11.0, 0.0, 0.0),
                false,
                None,
                None,
            );
            test_frustum_clip_line(
                &moved_right_frustum,
                v3(-50.0, 0.0, 0.0),
                v3(-9.0, 0.0, 0.0),
                true,
                Some(v3(-10.0, 0.0, 0.0)),
                Some(v3(-9.0, 0.0, 0.0)),
            );
        }

        // rotated right
        {
            let moved_right_frustum = Frustum::transform(
                &frustum,
                Matrix4X4::create_rotation_y(math_helper::degrees_to_radians(45.0)),
            );
            let test_matrix = Matrix4X4::create_rotation_y(math_helper::degrees_to_radians(45.0));
            let control = Plane::new(
                frustum.planes[0].normal.transform_normal(test_matrix),
                frustum.planes[0].distance_from_origin,
            );
            assert!(moved_right_frustum.planes[0].equals_within(&control, 0.001, 0.01));
            assert!(moved_right_frustum.planes[1].equals(&Plane::new(
                frustum.planes[1].normal.transform_normal(test_matrix),
                frustum.planes[1].distance_from_origin
            )));
            assert!(moved_right_frustum.planes[2].equals(&frustum.planes[2]));
            assert!(moved_right_frustum.planes[3].equals(&frustum.planes[3]));
            assert!(moved_right_frustum.planes[4].equals(&Plane::new(
                frustum.planes[4].normal.transform_normal(test_matrix),
                frustum.planes[4].distance_from_origin
            )));
            assert!(moved_right_frustum.planes[5].equals(&Plane::new(
                frustum.planes[5].normal.transform_normal(test_matrix),
                frustum.planes[5].distance_from_origin
            )));
        }
    }

    // rotate about y 180 degrees
    {
        let perspective_matrix =
            Matrix4X4::create_perspective_field_of_view(math_helper::TAU / 4.0, 1.0, 3.0, 507.0)
                .unwrap();
        let perspective_frustum = Frustum::frustum_from_projection_matrix(perspective_matrix);

        test_frustum_clip_line(
            &perspective_frustum,
            v3(-10.0, 0.0, -5.0),
            v3(0.0, 0.0, -5.0),
            true,
            Some(v3(-5.0, 0.0, -5.0)),
            Some(v3(0.0, 0.0, -5.0)),
        );
        test_frustum_clip_line(
            &perspective_frustum,
            v3(-50.0, 0.0, -5.0),
            v3(-21.0, 0.0, -5.0),
            false,
            None,
            None,
        );
        test_frustum_clip_line(
            &perspective_frustum,
            v3(-50.0, 0.0, -20.0),
            v3(-19.0, 0.0, -20.0),
            true,
            Some(v3(-20.0, 0.0, -20.0)),
            Some(v3(-19.0, 0.0, -20.0)),
        );

        let frustum = Frustum::transform(
            &perspective_frustum,
            Matrix4X4::create_rotation_y(math_helper::TAU / 2.0),
        );

        // left
        assert!(frustum.planes[0]
            .normal
            .equals_within(v3(-1.0, 0.0, 1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[0].distance_from_origin, 0.0);
        // right
        assert!(frustum.planes[1]
            .normal
            .equals_within(v3(1.0, 0.0, 1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[1].distance_from_origin, 0.0);
        // bottom
        assert!(frustum.planes[2]
            .normal
            .equals_within(v3(0.0, 1.0, 1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[2].distance_from_origin, 0.0);
        // top
        assert!(frustum.planes[3]
            .normal
            .equals_within(v3(0.0, -1.0, 1.0).get_normal(), 0.0001));
        assert_eq!(frustum.planes[3].distance_from_origin, 0.0);
        // near
        assert!(frustum.planes[4]
            .normal
            .equals_within(v3(0.0, 0.0, 1.0), 0.0001));
        assert_within(frustum.planes[4].distance_from_origin, 3.0, 0.0001);
        // far
        assert!(frustum.planes[5]
            .normal
            .equals_within(v3(0.0, 0.0, -1.0), 0.0001));
        assert_within(frustum.planes[5].distance_from_origin, -507.0, 0.0001);
    }

    // translate 10 down z
    {
        let z_move: f64 = 10.0;
        let perspective_matrix =
            Matrix4X4::create_perspective_field_of_view(math_helper::TAU / 4.0, 1.0, 3.0, 507.0)
                .unwrap();
        let frustum = Frustum::frustum_from_projection_matrix(perspective_matrix);
        let frustum = Frustum::transform(&frustum, Matrix4X4::create_translation(0.0, 0.0, -10.0));

        let expected_plane_offset = (2.0 * (z_move / 2.0) * (z_move / 2.0)).sqrt();
        // left
        assert!(frustum.planes[0]
            .normal
            .equals_within(v3(1.0, 0.0, -1.0).get_normal(), 0.0001));
        assert_within(
            frustum.planes[0].distance_from_origin,
            expected_plane_offset,
            0.0001,
        );
        // right
        assert!(frustum.planes[1]
            .normal
            .equals_within(v3(-1.0, 0.0, -1.0).get_normal(), 0.0001));
        assert_within(
            frustum.planes[1].distance_from_origin,
            expected_plane_offset,
            0.0001,
        );
        // bottom
        assert!(frustum.planes[2]
            .normal
            .equals_within(v3(0.0, 1.0, -1.0).get_normal(), 0.0001));
        assert_within(
            frustum.planes[2].distance_from_origin,
            expected_plane_offset,
            0.0001,
        );
        // top
        assert!(frustum.planes[3]
            .normal
            .equals_within(v3(0.0, -1.0, -1.0).get_normal(), 0.0001));
        assert_within(
            frustum.planes[3].distance_from_origin,
            expected_plane_offset,
            0.0001,
        );
        // near
        assert!(frustum.planes[4]
            .normal
            .equals_within(v3(0.0, 0.0, -1.0), 0.0001));
        assert_within(frustum.planes[4].distance_from_origin, 3.0 + z_move, 0.0001);
        // far
        assert!(frustum.planes[5]
            .normal
            .equals_within(v3(0.0, 0.0, 1.0), 0.0001));
        assert_within(
            frustum.planes[5].distance_from_origin,
            -507.0 - z_move,
            0.0001,
        );
    }
}

#[test]
fn plane_clip_line_tests() {
    {
        let plane = Plane::new(v3(1.0, 0.0, 0.0), 0.0);
        let mut line_start = v3(-5.0, 0.0, 0.0);
        let mut line_end = v3(5.0, 0.0, 0.0);
        let clipped = plane.clip_line(&mut line_start, &mut line_end);
        assert!(clipped);
        assert!(line_start.equals_within(v3(0.0, 0.0, 0.0), 0.0001));
        assert!(line_end.equals_within(v3(5.0, 0.0, 0.0), 0.0001));
    }

    {
        let plane = Plane::new(v3(1.0, 0.0, 0.0), 0.0);
        let mut line_start = v3(-5.0, 0.0, 0.0);
        let mut line_end = v3(-1.0, 0.0, 0.0);
        let clipped = plane.clip_line(&mut line_start, &mut line_end);
        assert!(!clipped);
    }

    {
        let plane = Plane::new(v3(1.0, 0.0, 0.0), 0.0);
        let mut line_start = v3(1.0, 0.0, 0.0);
        let mut line_end = v3(5.0, 0.0, 0.0);
        let clipped = plane.clip_line(&mut line_start, &mut line_end);
        assert!(clipped);
        assert!(line_start.equals_within(v3(1.0, 0.0, 0.0), 0.0001));
        assert!(line_end.equals_within(v3(5.0, 0.0, 0.0), 0.0001));
    }
}

/// C# `GetHashCode()`: here, `std::hash::Hash` through the default hasher.
fn get_hash_code<T: Hash>(value: &T) -> u64 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);
    hasher.finish()
}

#[test]
fn test_get_hash_code() {
    {
        let a = Vector2::new(10.0, 11.0);
        let b = Vector2::new(10.0, 11.0);
        assert!(get_hash_code(&a) == get_hash_code(&b));
    }
    {
        let a = Vector3::new(10.0, 11.0, 12.0);
        let b = Vector3::new(10.0, 11.0, 12.0);
        assert!(get_hash_code(&a) == get_hash_code(&b));
    }
    {
        let a = Vector4::new(10.0, 11.0, 12.0, 13.0);
        let b = Vector4::new(10.0, 11.0, 12.0, 13.0);
        assert!(get_hash_code(&a) == get_hash_code(&b));
    }
    {
        let a = Matrix4X4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        let b = Matrix4X4::new(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        );
        assert!(get_hash_code(&a) == get_hash_code(&b));
    }
}
