// Plane/ray intersection distance: the trace API case that uses only vector-math's Plane and
// Ray. The BVH enumeration case builds ray-tracer shapes and lives in polygon-mesh's
// tests/ray_tracer/trace_api_tests.rs.

use solidmesh_math::{Plane, Ray, Vector3};

#[test]
fn plane_get_distance_to_intersection() {
    let test_plane = Plane::new(Vector3::UNIT_Z, 10.0);

    let looking_at_front_of_plane =
        Ray::new(Vector3::new(0.0, 0.0, 11.0), Vector3::new(0.0, 0.0, -1.0));
    let (hit, distance_to_hit, hit_front_of_plane) =
        test_plane.ray_hit_plane(&looking_at_front_of_plane);
    assert!(hit);
    assert!(distance_to_hit == 1.0);
    assert!(hit_front_of_plane);

    let not_looking_at_front_of_plane =
        Ray::new(Vector3::new(0.0, 0.0, 11.0), Vector3::new(0.0, 0.0, 1.0));
    let (hit, distance_to_hit, hit_front_of_plane) =
        test_plane.ray_hit_plane(&not_looking_at_front_of_plane);
    assert!(!hit);
    assert!(distance_to_hit == f64::INFINITY);
    assert!(!hit_front_of_plane);

    let looking_at_back_of_plane =
        Ray::new(Vector3::new(0.0, 0.0, 9.0), Vector3::new(0.0, 0.0, 1.0));
    let (hit, distance_to_hit, hit_front_of_plane) =
        test_plane.ray_hit_plane(&looking_at_back_of_plane);
    assert!(hit);
    assert!(distance_to_hit == 1.0);
    assert!(!hit_front_of_plane);

    let not_looking_at_back_of_plane =
        Ray::new(Vector3::new(0.0, 0.0, 9.0), Vector3::new(0.0, 0.0, -1.0));
    let (hit, distance_to_hit, hit_front_of_plane) =
        test_plane.ray_hit_plane(&not_looking_at_back_of_plane);
    assert!(!hit);
    assert!(distance_to_hit == f64::INFINITY);
    assert!(hit_front_of_plane);
}
