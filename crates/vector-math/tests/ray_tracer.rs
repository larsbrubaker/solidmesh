// Ray-tracer tests that need only vector-math types:
// the frustum, the ray-box slab test and the plane/ray intersection. The ray tracer's own tests
// are in polygon-mesh's tests/ray_tracer.

#[path = "ray_tracer/frustum_tests.rs"]
mod frustum_tests;
#[path = "ray_tracer/ray_box_face_plane_tests.rs"]
mod ray_box_face_plane_tests;
#[path = "ray_tracer/ray_box_nan_guard_tests.rs"]
mod ray_box_nan_guard_tests;
#[path = "ray_tracer/trace_api_tests.rs"]
mod trace_api_tests;
