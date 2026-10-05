//! Ported `R/Tests/GeometryProcessingTests` test classes for `Topology/`, one file each
//! (CLAUDE.md net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "topology/bevel_group_resolution_advisory_tests.rs"]
mod bevel_group_resolution_advisory_tests;
#[path = "topology/face_pick_interpretation_tests.rs"]
mod face_pick_interpretation_tests;
#[path = "topology/feature_edge_collinear_run_tests.rs"]
mod feature_edge_collinear_run_tests;
#[path = "topology/rust_only_bevel_group_tests.rs"]
mod rust_only_bevel_group_tests;
#[path = "topology/split_crease_test_meshes.rs"]
mod split_crease_test_meshes;
