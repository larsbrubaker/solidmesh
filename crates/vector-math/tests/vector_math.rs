// Vector-math tests (CLAUDE.md "Testing framework", net 1): vectors, matrices, easing, the
// math-helper constants, the parallel helpers, and the BVH, estimator and track-ball checks.

#[path = "vector_math/bvh_build_quality_tests.rs"]
mod bvh_build_quality_tests;
#[path = "vector_math/easing_tests.rs"]
mod easing_tests;
#[path = "vector_math/math_helper_constant_tests.rs"]
mod math_helper_constant_tests;
#[path = "vector_math/parallel_inline_on_marked_thread_tests.rs"]
mod parallel_inline_on_marked_thread_tests;
#[path = "vector_math/parallel_sequential_equivalence_tests.rs"]
mod parallel_sequential_equivalence_tests;
#[path = "vector_math/plane_ray_and_helper_tests.rs"]
mod plane_ray_and_helper_tests;
#[path = "vector_math/rust_only_bvh_tests.rs"]
mod rust_only_bvh_tests;
#[path = "vector_math/rust_only_fixed_tie_sort_tests.rs"]
mod rust_only_fixed_tie_sort_tests;
#[path = "vector_math/rust_only_matrix_tests.rs"]
mod rust_only_matrix_tests;
#[path = "vector_math/rust_only_number_format_tests.rs"]
mod rust_only_number_format_tests;
#[path = "vector_math/rust_only_seeded_random_tests.rs"]
mod rust_only_seeded_random_tests;
#[path = "vector_math/vector2_tests.rs"]
mod vector2_tests;
#[path = "vector_math/vector3_tests.rs"]
mod vector3_tests;
