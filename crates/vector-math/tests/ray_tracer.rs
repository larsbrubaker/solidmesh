// Ray-tracer tests that need only vector-math types:
// the frustum and the plane/ray intersection. The ray tracer's own tests are in polygon-mesh's
// tests/ray_tracer.

#[path = "ray_tracer/frustum_tests.rs"]
mod frustum_tests;
#[path = "ray_tracer/trace_api_tests.rs"]
mod trace_api_tests;
