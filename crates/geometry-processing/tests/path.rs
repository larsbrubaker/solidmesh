//! Ported `R/Tests/GeometryProcessingTests` test classes for `Path/`, one file each,
//! with the shared `PathBevelTestGeometry` measurements as a module.

#[path = "path/flattened_curve_path.rs"]
mod flattened_curve_path;
#[path = "path/path_bevel_curved_run_tests.rs"]
mod path_bevel_curved_run_tests;
#[path = "path/path_bevel_platform_bits_tests.rs"]
mod path_bevel_platform_bits_tests;
#[path = "path/path_bevel_test_geometry.rs"]
mod path_bevel_test_geometry;
#[path = "path/path_bevel_tests.rs"]
mod path_bevel_tests;
#[path = "path/path_segment_run_tests.rs"]
mod path_segment_run_tests;
