//! Ported `R/Tests/GeometryProcessingTests` test classes for `Bevel/`, one file each (CLAUDE.md
//! net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "bevel/bevel_arc_chording_tests.rs"]
mod bevel_arc_chording_tests;
#[path = "bevel/bevel_seam_ring_tests.rs"]
mod bevel_seam_ring_tests;
#[path = "bevel/rust_only_bevel_core_leaf_tests.rs"]
mod rust_only_bevel_core_leaf_tests;
#[path = "bevel/rust_only_bevel_slice2_tests.rs"]
mod rust_only_bevel_slice2_tests;
#[path = "bevel/rust_only_bevel_slice3_tests.rs"]
mod rust_only_bevel_slice3_tests;
#[path = "bevel/rust_only_bevel_slice4_tests.rs"]
mod rust_only_bevel_slice4_tests;
#[path = "bevel/rust_only_bevel_slice5_tests.rs"]
mod rust_only_bevel_slice5_tests;
#[path = "bevel/surface_mesh_closure_tests.rs"]
mod surface_mesh_closure_tests;
#[path = "bevel/topology_test_meshes.rs"]
mod topology_test_meshes;
