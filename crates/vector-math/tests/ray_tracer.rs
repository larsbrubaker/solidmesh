// Ported tests from agg-sharp `Tests/Agg.Tests/Agg.RayTracer` that exercise only VectorMath
// types (CLAUDE.md "Testing framework", net 1). The rest of that folder tests the RayTracer/BVH
// and joins with Phase 2 (polygon-mesh).

#[path = "ray_tracer/frustum_tests.rs"]
mod frustum_tests;
