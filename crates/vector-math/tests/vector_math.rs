// Ported tests from agg-sharp `Tests/Agg.Tests/Agg.VectorMath/VectorMathTests.cs`, one file
// per C# test class (CLAUDE.md "Testing framework", net 1). Phase 1a ports the classes and
// tests that cover the vector types; the Easing, WorldView, Frustum, Plane, hash and JSON
// serialization tests join with Phase 1b.

#[path = "vector_math/math_helper_constant_tests.rs"]
mod math_helper_constant_tests;
#[path = "vector_math/vector2_tests.rs"]
mod vector2_tests;
#[path = "vector_math/vector3_tests.rs"]
mod vector3_tests;
