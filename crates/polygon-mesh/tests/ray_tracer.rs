//! The ray-tracer test binary: triangle and shape hits, view-space CSG, the trace API, and the
//! BVH built in parallel against the sequential build, one file per test class.

#[path = "ray_tracer/boolean_tests.rs"]
mod boolean_tests;
#[path = "ray_tracer/minimal_triangle_tests.rs"]
mod minimal_triangle_tests;
#[path = "ray_tracer/parallel_sequential_equivalence_tests.rs"]
mod parallel_sequential_equivalence_tests;
#[path = "ray_tracer/polygon_trace_tests.rs"]
mod polygon_trace_tests;
#[path = "ray_tracer/rust_only_primary_ray_tests.rs"]
mod rust_only_primary_ray_tests;
#[path = "ray_tracer/rust_only_ray_tracer_tests.rs"]
mod rust_only_ray_tracer_tests;
#[path = "ray_tracer/rust_only_shape_tests.rs"]
mod rust_only_shape_tests;
#[path = "ray_tracer/rust_only_triangle_edge_tests.rs"]
mod rust_only_triangle_edge_tests;
#[path = "ray_tracer/trace_api_tests.rs"]
mod trace_api_tests;
