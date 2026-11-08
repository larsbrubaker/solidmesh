// Plane crossings, ray state, and the small numeric helpers: offset planes, Vector4 clamping,
// perspective transforms, exact powers of two, saturating factorials, and the ray's slab terms,
// shadow flag and back-face normals.

use solidmesh_math::{
    math_helper, AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Sign, Vector3,
    Vector4,
};

#[test]
fn line_hit_plane_finds_the_crossing_of_a_plane_off_the_origin() {
    // The plane z = 5.
    let plane = Plane::new(Vector3::new(0.0, 0.0, 1.0), 5.0);

    let (hit, at) = plane.line_hit_plane(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 10.0));
    assert!(hit);
    assert_eq!(at, Vector3::new(0.0, 0.0, 5.0));

    let (hit, at) = plane.line_hit_plane(Vector3::new(1.0, 2.0, 4.0), Vector3::new(3.0, 2.0, 8.0));
    assert!(hit);
    assert_eq!(at, Vector3::new(1.5, 2.0, 5.0));

    // A segment through the origin's parallel plane but nowhere near z = 5 does not cross.
    let (hit, at) = plane.line_hit_plane(Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, 1.0));
    assert!(!hit);
    assert_eq!(at, Vector3::POSITIVE_INFINITY);

    // An end on the plane is the crossing.
    let (hit, at) = plane.line_hit_plane(Vector3::new(2.0, 0.0, 5.0), Vector3::new(0.0, 0.0, 9.0));
    assert!(hit);
    assert_eq!(at, Vector3::new(2.0, 0.0, 5.0));
}

#[test]
fn vector4_clamp_limits_every_component_by_its_own_value() {
    let min = Vector4::new(0.0, 0.0, 0.0, 0.0);
    let max = Vector4::new(1.0, 1.0, 1.0, 1.0);
    assert_eq!(
        Vector4::clamp(Vector4::new(0.5, 0.5, -3.0, -4.0), min, max),
        Vector4::new(0.5, 0.5, 0.0, 0.0)
    );
    assert_eq!(
        Vector4::clamp(Vector4::new(-1.0, -1.0, 0.25, 0.75), min, max),
        Vector4::new(0.0, 0.0, 0.25, 0.75)
    );
}

#[test]
fn transform_perspective_applies_the_translation_row() {
    let moved = Matrix4X4::create_translation(1.0, 2.0, 3.0);
    assert_eq!(
        Vector3::new(1.0, 1.0, 1.0).transform_perspective(moved),
        Vector3::new(2.0, 3.0, 4.0)
    );
}

#[test]
fn next_power_of_two_is_exact_for_every_power_of_two() {
    for k in 0..31 {
        let pow2: i32 = 1 << k;
        assert_eq!(math_helper::next_power_of_two_i32(pow2), Ok(pow2), "2^{k}");
        assert_eq!(
            math_helper::next_power_of_two_i64(i64::from(pow2)),
            Ok(i64::from(pow2))
        );
        assert_eq!(
            math_helper::next_power_of_two_f64(f64::from(pow2)),
            Ok(f64::from(pow2))
        );
        if k < 30 {
            assert_eq!(
                math_helper::next_power_of_two_i32(pow2 + 1),
                Ok(pow2 * 2),
                "2^{k}+1"
            );
        }
    }
    for k in 31..62 {
        let pow2: i64 = 1 << k;
        assert_eq!(math_helper::next_power_of_two_i64(pow2), Ok(pow2), "2^{k}");
    }
    assert_eq!(math_helper::next_power_of_two_f64(0.3), Ok(0.5));
    assert_eq!(math_helper::next_power_of_two_f64(0.25), Ok(0.25));
}

#[test]
fn first_power_of_two_at_or_above_saturates_instead_of_looping() {
    assert_eq!(math_helper::first_power_two_greater_than_or_equal_to(0), 1);
    assert_eq!(math_helper::first_power_two_greater_than_or_equal_to(5), 8);
    assert_eq!(
        math_helper::first_power_two_greater_than_or_equal_to(1 << 30),
        1 << 30
    );
    assert_eq!(
        math_helper::first_power_two_greater_than_or_equal_to((1 << 30) + 1),
        i32::MAX
    );
    assert_eq!(
        math_helper::first_power_two_greater_than_or_equal_to(i32::MAX),
        i32::MAX
    );
}

#[test]
fn factorial_saturates_past_twenty() {
    assert_eq!(math_helper::factorial(20), 2_432_902_008_176_640_000);
    assert_eq!(math_helper::factorial(21), i64::MAX);
    assert_eq!(math_helper::factorial(40), i64::MAX);
}

#[test]
fn perturbed_ray_tests_boxes_along_its_new_direction() {
    let mut ray = Ray::new(Vector3::new(0.0, 0.0, 10.0), Vector3::new(-1e-6, 0.0, -1.0));
    // A thin box just to the +x side, only reachable once the direction turns towards +x.
    let thin =
        AxisAlignedBoundingBox::new(Vector3::new(2e-5, -1.0, -1.0), Vector3::new(1e-3, 1.0, 1.0));
    assert!(!ray.intersection(&thin));

    ray.perturb_direction(|| 0.5);
    assert_eq!(ray.direction_normal.x, -1e-6 + 5e-6);
    assert_eq!(ray.one_over_direction, 1.0 / ray.direction_normal);
    assert_eq!(ray.sign, [Sign::Positive, Sign::Positive, Sign::Negative]);
    assert!(ray.intersection(&thin));
}

#[test]
fn transformed_ray_keeps_its_shadow_flag() {
    let mut ray = Ray::new(Vector3::ZERO, Vector3::new(0.0, 0.0, -1.0));
    ray.is_shadow_ray = true;
    let moved = Ray::transform(&ray, Matrix4X4::create_translation(1.0, 0.0, 0.0));
    assert!(moved.is_shadow_ray);
    assert_eq!(moved.origin, Vector3::new(1.0, 0.0, 0.0));
}

#[test]
fn back_face_hit_reports_the_exit_face_normal() {
    let bounds =
        AxisAlignedBoundingBox::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let back = |origin: Vector3, direction: Vector3| {
        Ray::with_limits(
            origin,
            direction,
            0.0,
            f64::INFINITY,
            IntersectionType::BackFace,
        )
    };

    // Straight down: leaves through the bottom.
    let hit = back(Vector3::new(0.25, 0.25, 5.0), Vector3::new(0.0, 0.0, -1.0))
        .get_closest_intersection(&bounds);
    assert_eq!(hit.hit_type, IntersectionType::BackFace);
    assert_eq!(hit.normal_at_hit, Vector3::new(0.0, 0.0, -1.0));

    // Enters through the top, leaves through the +x side.
    let hit = back(Vector3::new(-0.5, 0.0, 1.5), Vector3::new(0.6, 0.0, -0.8))
        .get_closest_intersection(&bounds);
    assert_eq!(hit.hit_type, IntersectionType::BackFace);
    assert_eq!(hit.normal_at_hit, Vector3::new(1.0, 0.0, 0.0));
    assert!((hit.hit_position.x - 1.0).abs() < 1e-12);
}
