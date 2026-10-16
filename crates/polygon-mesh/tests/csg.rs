//! The CSG test binary: two-mesh and n-ary booleans, the Minkowski operations, implicit-surface
//! booleans and the manifold-rust backend, one file per test class.

#[path = "csg/boolean_kernel_yield_tests.rs"]
mod boolean_kernel_yield_tests;
#[path = "csg/boolean_processing_crash_tests.rs"]
mod boolean_processing_crash_tests;
#[path = "csg/boolean_reporter_routing_tests.rs"]
mod boolean_reporter_routing_tests;
#[path = "csg/csg_tests.rs"]
mod csg_tests;
#[path = "csg/implicit_boolean_tests.rs"]
mod implicit_boolean_tests;
#[path = "csg/kernel_call_count_tests.rs"]
mod kernel_call_count_tests;
#[path = "csg/manifold_rust_backend_cancel_tests.rs"]
mod manifold_rust_backend_cancel_tests;
#[path = "csg/manifold_rust_backend_tests.rs"]
mod manifold_rust_backend_tests;
#[path = "csg/minkowski_erosion_cancel_tests.rs"]
mod minkowski_erosion_cancel_tests;
#[path = "csg/minkowski_orientation_tests.rs"]
mod minkowski_orientation_tests;
#[path = "csg/minkowski_progress_model_tests.rs"]
mod minkowski_progress_model_tests;
#[path = "csg/minkowski_tests.rs"]
mod minkowski_tests;
#[path = "csg/single_precision_union_tests.rs"]
mod single_precision_union_tests;
