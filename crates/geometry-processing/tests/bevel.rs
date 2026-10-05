//! Ported `R/Tests/GeometryProcessingTests` test classes for `Bevel/`, one file each (CLAUDE.md
//! net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "bevel/bevel_arc_chording_tests.rs"]
mod bevel_arc_chording_tests;
#[path = "bevel/bevel_rebuild_advisories_tests.rs"]
mod bevel_rebuild_advisories_tests;
#[path = "bevel/bevel_seam_ring_tests.rs"]
mod bevel_seam_ring_tests;
#[path = "bevel/bevel_source_round_tests.rs"]
mod bevel_source_round_tests;
#[path = "bevel/bevel_void_wedge_section_tests.rs"]
mod bevel_void_wedge_section_tests;
#[path = "bevel/flat_hole_fill_tests.rs"]
mod flat_hole_fill_tests;
#[path = "bevel/hole_fill_tests.rs"]
mod hole_fill_tests;
#[path = "bevel/rust_only_bevel_core_leaf_tests.rs"]
mod rust_only_bevel_core_leaf_tests;
#[path = "bevel/rust_only_bevel_r1p01_mesh_tests.rs"]
mod rust_only_bevel_r1p01_mesh_tests;
#[path = "bevel/rust_only_bevel_r1p01_meter_tests.rs"]
mod rust_only_bevel_r1p01_meter_tests;
#[path = "bevel/rust_only_bevel_r1p01_tests.rs"]
mod rust_only_bevel_r1p01_tests;
#[path = "bevel/rust_only_bevel_r1p02_tests.rs"]
mod rust_only_bevel_r1p02_tests;
#[path = "bevel/rust_only_bevel_r1p03_tests.rs"]
mod rust_only_bevel_r1p03_tests;
#[path = "bevel/rust_only_bevel_r1p04_tests.rs"]
mod rust_only_bevel_r1p04_tests;
#[path = "bevel/rust_only_bevel_r1p05_tests.rs"]
mod rust_only_bevel_r1p05_tests;
#[path = "bevel/rust_only_bevel_r1p06_tests.rs"]
mod rust_only_bevel_r1p06_tests;
#[path = "bevel/rust_only_bevel_r1p07_tests.rs"]
mod rust_only_bevel_r1p07_tests;
#[path = "bevel/rust_only_bevel_slice10_tests.rs"]
mod rust_only_bevel_slice10_tests;
#[path = "bevel/rust_only_bevel_slice2_tests.rs"]
mod rust_only_bevel_slice2_tests;
#[path = "bevel/rust_only_bevel_slice3_tests.rs"]
mod rust_only_bevel_slice3_tests;
#[path = "bevel/rust_only_bevel_slice4_tests.rs"]
mod rust_only_bevel_slice4_tests;
#[path = "bevel/rust_only_bevel_slice5_tests.rs"]
mod rust_only_bevel_slice5_tests;
#[path = "bevel/rust_only_bevel_slice6_tests.rs"]
mod rust_only_bevel_slice6_tests;
#[path = "bevel/rust_only_bevel_slice7_tests.rs"]
mod rust_only_bevel_slice7_tests;
#[path = "bevel/rust_only_bevel_slice8_tests.rs"]
mod rust_only_bevel_slice8_tests;
#[path = "bevel/rust_only_bevel_slice9_tests.rs"]
mod rust_only_bevel_slice9_tests;
#[path = "bevel/source_round_test_meshes.rs"]
mod source_round_test_meshes;
#[path = "bevel/surface_crossed_end_carries_what_stands_before_the_foot_tests.rs"]
mod surface_crossed_end_carries_what_stands_before_the_foot_tests;
#[path = "bevel/surface_facet_cut_own_ends_tests.rs"]
mod surface_facet_cut_own_ends_tests;
#[path = "bevel/surface_fan_tests.rs"]
mod surface_fan_tests;
#[path = "bevel/surface_mesh_closure_tests.rs"]
mod surface_mesh_closure_tests;
#[path = "bevel/surface_ring_ears_bridged_ring_tests.rs"]
mod surface_ring_ears_bridged_ring_tests;
#[path = "bevel/topology_test_meshes.rs"]
mod topology_test_meshes;
