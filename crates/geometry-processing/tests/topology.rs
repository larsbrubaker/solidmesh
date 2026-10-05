//! Ported `R/Tests/GeometryProcessingTests` test classes for `Topology/`, one file each
//! (CLAUDE.md net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "topology/feature_edge_collinear_run_tests.rs"]
mod feature_edge_collinear_run_tests;
#[path = "topology/split_crease_test_meshes.rs"]
mod split_crease_test_meshes;
