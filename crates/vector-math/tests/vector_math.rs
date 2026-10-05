// Ported tests from agg-sharp `Tests/Agg.Tests/Agg.VectorMath/VectorMathTests.cs`, one file
// per C# test class (CLAUDE.md "Testing framework", net 1), plus Rust-only checks. The Easing
// and WorldView tests join with Phase 1c.

#[path = "vector_math/math_helper_constant_tests.rs"]
mod math_helper_constant_tests;
#[path = "vector_math/rust_only_matrix_tests.rs"]
mod rust_only_matrix_tests;
#[path = "vector_math/vector2_tests.rs"]
mod vector2_tests;
#[path = "vector_math/vector3_tests.rs"]
mod vector3_tests;
