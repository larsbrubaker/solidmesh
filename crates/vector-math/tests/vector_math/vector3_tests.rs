// Vector3 behaviour: the vector, JSON, WorldView, frustum, plane and hash tests. The hash test
// hashes through `std::hash::Hash`.

use solidmesh_math::{
    math_helper, AxisAlignedBoundingBox, Frustum, FrustumIntersection, Matrix4X4, Plane,
    Quaternion, Vector2, Vector2Float, Vector3, Vector3Float, Vector4, WorldView,
};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

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

/// Clips a line against a frustum and checks whether any of it is inside and, if so, where the
/// clipped ends land.
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

/// A value's hash through `std::hash::Hash` and the default hasher.
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

#[test]
fn world_view_perspective_projection_tests() {
    let mut world = WorldView::new(1.0, 1.0);
    assert!(world
        .eye_position()
        .equals_within(Vector3::UNIT_Z * 7.0, 1e-3));
    world
        .calculate_perspective_matrix_off_center(567.0, 123.0, -200.0, 5.0, 55.0, 45.0)
        .unwrap();
    assert!(world
        .get_screen_position(Vector3::new(0.0, 0.0, 0.0))
        .equals_within(Vector2::new((567.0 - 200.0) / 2.0, 123.0 / 2.0), 1e-3));
    assert_eq!(world.near_z(), 5.0);
    assert_eq!(world.far_z(), 55.0);
    // 10 * (sqrt(2) - 1), reached through Math.Tan. The last bit of that is the C runtime's, not
    // ours: x64 answers the double nearest 4.14213562373095 and arm64 the one nearest
    // 4.142135623730951, one ulp apart. The value is what is being tested, not the rounding.
    assert_within(
        world.near_plane_height_in_viewspace(),
        4.14213562373095,
        1e-9,
    );
    let ray = world.get_ray_for_local_bounds(Vector2::new((567.0 - 200.0) / 2.0, 123.0)); // top center
    assert_eq!(
        math_helper::radians_to_degrees(ray.direction_normal.y.atan2(-ray.direction_normal.z)),
        WorldView::DEFAULT_PERSPECTIVE_VFOV_DEGREES / 2.0
    );
    assert!((Vector3::UNIT_Z * 7.0).equals_within(ray.origin, 1e-3));
    assert_eq!(
        world.get_viewspace_height_at_position(Vector3::new(1.0, 1.0, -10.0)),
        world.near_plane_height_in_viewspace() * 2.0
    );
    world.set_scale(3.0);
    assert_within(
        world.get_world_units_per_screen_pixel_at_position(
            Vector3::new(1.0, 1.0, (7.0 - 10.0) / 3.0),
            f64::INFINITY,
        ) * 123.0,
        world.near_plane_height_in_viewspace() * 2.0 / 3.0,
        1e-6,
    );
}

#[test]
fn world_view_orthographic_projection_tests() {
    let mut world = WorldView::new(1.0, 1.0);
    assert!(world
        .eye_position()
        .equals_within(Vector3::UNIT_Z * 7.0, 1e-3));
    world.calculate_orthographic_matrix_off_center_with_viewspace_height(
        680.0, 240.0, -200.0, 128.0, 5.0, 55.0,
    );
    assert!(world
        .get_screen_position(Vector3::new(0.0, 0.0, 0.0))
        .equals_within(Vector2::new((680.0 - 200.0) / 2.0, 240.0 / 2.0), 1e-3));
    assert!(world
        .get_screen_position(Vector3::new(128.0, 64.0, 0.0))
        .equals_within(Vector2::new(680.0 - 200.0, 240.0), 1e-3));
    assert!(world
        .get_screen_position(Vector3::new(-128.0, -64.0, 0.0))
        .equals_within(Vector2::new(0.0, 0.0), 1e-3));
    assert_eq!(world.near_z(), 5.0);
    assert_eq!(world.far_z(), 55.0);
    assert_eq!(world.near_plane_height_in_viewspace(), 128.0);
    let ray = world.get_ray_for_local_bounds(Vector2::new((680.0 - 200.0) / 2.0, 240.0)); // top center
    assert!(Vector3::UNIT_Z.equals_within(-ray.direction_normal.get_normal(), 1e-3));
    assert!(Vector3::new(0.0, 64.0, 2.0).equals_within(ray.origin, 1e-3));
    assert_eq!(
        world.get_viewspace_height_at_position(Vector3::new(1.0, 1.0, -10.0)),
        world.near_plane_height_in_viewspace()
    );
    world.set_scale(3.0);
    assert_eq!(
        world.get_world_units_per_screen_pixel_at_position(
            Vector3::new(1.0, 1.0, (7.0 - 10.0) / 3.0),
            f64::INFINITY,
        ) * 240.0,
        world.near_plane_height_in_viewspace() / 3.0
    );
}

/// A 3D handle sizes itself as "so many pixels times the world units per pixel here", so it only
/// holds its pixel size while that ratio is the honest one. It used to be capped at 5 world units
/// per pixel, which nothing smaller than a few metres could reach - but someone modelling a room
/// is looking at tens of metres across a thousand pixels, where the true ratio is 30 or more, and
/// every handle in the view silently shrank to a sixth of its size or less. The cap is now off by
/// default; what remains is the guard it was standing in for, which is that a degenerate camera
/// must still return a usable number.
#[test]
fn world_units_per_pixel_is_unclamped_at_room_scale() {
    const VIEW_SIZE: f64 = 1000.0;
    const EYE_DISTANCE: f64 = 40000.0;

    let mut world = WorldView::new(VIEW_SIZE, VIEW_SIZE);
    world
        .calculate_perspective_matrix_off_center(
            VIEW_SIZE,
            VIEW_SIZE,
            0.0,
            WorldView::DEFAULT_NEAR_Z,
            WorldView::DEFAULT_FAR_Z,
            WorldView::DEFAULT_PERSPECTIVE_VFOV_DEGREES,
        )
        .unwrap();
    world.set_eye_position(Vector3::new(0.0, 0.0, EYE_DISTANCE));

    // The view is EyeDistance away from the origin down -Z, so the visible height there is the
    // perspective height at that distance - about 33.1 metres of scene over 1000 pixels, or ~33
    // units per pixel.
    let true_ratio = WorldView::calc_perspective_height(
        EYE_DISTANCE,
        WorldView::DEFAULT_PERSPECTIVE_VFOV_DEGREES,
    ) / VIEW_SIZE;
    assert!(true_ratio > 30.0);
    assert_within(
        world.get_world_units_per_screen_pixel_at_position(Vector3::ZERO, f64::INFINITY),
        true_ratio,
        1e-6,
    );

    // A caller that asks for a ceiling still gets one - and a nonsense ceiling is caught by the
    // same guard as a nonsense camera, rather than passed straight back out.
    assert_eq!(
        world.get_world_units_per_screen_pixel_at_position(Vector3::ZERO, 5.0),
        5.0
    );
    assert_eq!(
        world.get_world_units_per_screen_pixel_at_position(Vector3::ZERO, f64::NAN),
        WorldView::DEGENERATE_WORLD_UNITS_PER_PIXEL
    );
    assert_eq!(
        world.get_world_units_per_screen_pixel_at_position(Vector3::ZERO, 0.0),
        WorldView::DEGENERATE_WORLD_UNITS_PER_PIXEL
    );

    // The degenerate cases the old cap doubled as a catch-all for: callers divide by this and
    // size geometry with it, so it must stay finite and positive however nonsensical the input.
    for broken in [
        Vector3::new(f64::NAN, 0.0, 0.0),
        Vector3::new(f64::INFINITY, 0.0, 0.0),
    ] {
        let ratio = world.get_world_units_per_screen_pixel_at_position(broken, f64::INFINITY);
        assert!(ratio.is_finite());
        assert!(ratio > 0.0);
    }
}

#[test]
fn world_view_eye_position_tests() {
    let mut world = WorldView::new(1.0, 1.0);
    world.set_eye_position(Vector3::new(1.0, 2.0, 3.0));
    assert!(Vector3::new(1.0, 2.0, 3.0).equals_within(world.eye_position(), 1e-3));
}

/// Scale is the camera's uniform scale alone. It once read the transformed unit X as a POSITION,
/// so the camera's translation leaked in: a moved camera reported roughly scale times its
/// distance to the origin, and one that had world X = 1 at the view pivot reported 0, which made
/// every zoom a no-op.
#[test]
fn world_view_scale_ignores_translation_tests() {
    let mut world = WorldView::new(100.0, 100.0);
    world.reset();
    world.set_scale(0.05);
    world.translate(Vector3::new(-12.0, 7.0, 3.0));
    world.rotate(Quaternion::from_euler_angles(Vector3::new(0.4, -0.2, 1.1)));
    assert_within(world.scale(), 0.05, 1e-12);

    // Setting it scales about the view pivot (the world point at the view-space origin): the
    // pivot stays put and the eye moves along its line to it, closer by the factor.
    let pivot = Vector3::ZERO.transform(world.get_transform_4x4().inverted());
    let eye_from_pivot = world.eye_position() - pivot;
    world.set_scale(0.1);
    assert_within(world.scale(), 0.1, 1e-12);
    assert!(pivot.equals_within(
        Vector3::ZERO.transform(world.get_transform_4x4().inverted()),
        1e-9
    ));
    assert!((eye_from_pivot / 2.0).equals_within(world.eye_position() - pivot, 1e-9));

    // World X = 1 at the view pivot.
    world.reset();
    world.translate(-Vector3::UNIT_X);
    world.set_scale(world.scale() * 1.2);
    assert_within(world.scale(), 1.2, 1e-12);
}
