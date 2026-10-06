//! Ported agg-sharp `Tests/Agg.Tests/Agg.RayTracer` test classes (and the ray-tracer cases of
//! `Agg.VectorMath/ParallelSequentialEquivalenceTests`), one file each (CLAUDE.md net 1).
//! Cases that need code not ported yet are listed as skipped in each file's header.

#[path = "ray_tracer/boolean_tests.rs"]
mod boolean_tests;
#[path = "ray_tracer/minimal_triangle_tests.rs"]
mod minimal_triangle_tests;
#[path = "ray_tracer/parallel_sequential_equivalence_tests.rs"]
mod parallel_sequential_equivalence_tests;
#[path = "ray_tracer/rust_only_ray_tracer_tests.rs"]
mod rust_only_ray_tracer_tests;
#[path = "ray_tracer/rust_only_shape_tests.rs"]
mod rust_only_shape_tests;
#[path = "ray_tracer/rust_only_triangle_edge_tests.rs"]
mod rust_only_triangle_edge_tests;
#[path = "ray_tracer/trace_api_tests.rs"]
mod trace_api_tests;
