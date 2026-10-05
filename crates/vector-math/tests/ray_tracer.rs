// Ported tests from agg-sharp `Tests/Agg.Tests/Agg.RayTracer` that exercise only VectorMath
// types (CLAUDE.md "Testing framework", net 1). The rest of that folder tests the PolygonMesh
// RayTracer and joins with Phase 2 (polygon-mesh).

#[path = "ray_tracer/frustum_tests.rs"]
mod frustum_tests;
#[path = "ray_tracer/trace_api_tests.rs"]
mod trace_api_tests;
