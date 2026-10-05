// Ported tests from agg-sharp `Tests/Agg.Tests/Agg.VectorMath/VectorMathTests.cs`, one file
// per C# test class (CLAUDE.md "Testing framework", net 1), plus the two Parallel test classes
// from the same folder and Rust-only checks.

#[path = "vector_math/easing_tests.rs"]
mod easing_tests;
#[path = "vector_math/math_helper_constant_tests.rs"]
mod math_helper_constant_tests;
#[path = "vector_math/parallel_inline_on_marked_thread_tests.rs"]
mod parallel_inline_on_marked_thread_tests;
#[path = "vector_math/parallel_sequential_equivalence_tests.rs"]
mod parallel_sequential_equivalence_tests;
#[path = "vector_math/rust_only_bvh_tests.rs"]
mod rust_only_bvh_tests;
#[path = "vector_math/rust_only_cs_sort_tests.rs"]
mod rust_only_cs_sort_tests;
#[path = "vector_math/rust_only_dotnet_random_tests.rs"]
mod rust_only_dotnet_random_tests;
#[path = "vector_math/rust_only_matrix_tests.rs"]
mod rust_only_matrix_tests;
#[path = "vector_math/rust_only_number_format_tests.rs"]
mod rust_only_number_format_tests;
#[path = "vector_math/vector2_tests.rs"]
mod vector2_tests;
#[path = "vector_math/vector3_tests.rs"]
mod vector3_tests;
