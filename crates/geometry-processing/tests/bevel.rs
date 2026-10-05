//! Ported `R/Tests/GeometryProcessingTests` test classes for `Bevel/`, one file each (CLAUDE.md
//! net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "bevel/bevel_arc_chording_tests.rs"]
mod bevel_arc_chording_tests;
#[path = "bevel/bevel_bead_batch_tests.rs"]
mod bevel_bead_batch_tests;
#[path = "bevel/bevel_cap_clip_tests.rs"]
mod bevel_cap_clip_tests;
#[path = "bevel/bevel_chain_linker_rim_closure_tests.rs"]
mod bevel_chain_linker_rim_closure_tests;
#[path = "bevel/bevel_mitre_split_tests.rs"]
mod bevel_mitre_split_tests;
#[path = "bevel/bevel_rebuild_advisories_tests.rs"]
mod bevel_rebuild_advisories_tests;
#[path = "bevel/bevel_seam_ring_tests.rs"]
mod bevel_seam_ring_tests;
#[path = "bevel/bevel_source_round_tests.rs"]
mod bevel_source_round_tests;
#[path = "bevel/bevel_sweep_back_depth_tests.rs"]
mod bevel_sweep_back_depth_tests;
#[path = "bevel/bevel_void_wedge_section_tests.rs"]
mod bevel_void_wedge_section_tests;
#[path = "bevel/faceted_boss_fade_tests.rs"]
mod faceted_boss_fade_tests;
#[path = "bevel/fillet_arc_seam_row_tests.rs"]
mod fillet_arc_seam_row_tests;
#[path = "bevel/flat_hole_fill_tests.rs"]
mod flat_hole_fill_tests;
#[path = "bevel/hole_fill_tests.rs"]
mod hole_fill_tests;
#[path = "bevel/opposite_sense_crease_tests.rs"]
mod opposite_sense_crease_tests;
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
#[path = "bevel/rust_only_bevel_r1p04_notch_tests.rs"]
mod rust_only_bevel_r1p04_notch_tests;
#[path = "bevel/rust_only_bevel_r1p04_tests.rs"]
mod rust_only_bevel_r1p04_tests;
#[path = "bevel/rust_only_bevel_r1p05_tests.rs"]
mod rust_only_bevel_r1p05_tests;
#[path = "bevel/rust_only_bevel_r1p06_tests.rs"]
mod rust_only_bevel_r1p06_tests;
#[path = "bevel/rust_only_bevel_r1p07_tests.rs"]
mod rust_only_bevel_r1p07_tests;
#[path = "bevel/rust_only_bevel_r1p08_join_tests.rs"]
mod rust_only_bevel_r1p08_join_tests;
#[path = "bevel/rust_only_bevel_r1p08_tests.rs"]
mod rust_only_bevel_r1p08_tests;
#[path = "bevel/rust_only_bevel_r1p08_walk_tests.rs"]
mod rust_only_bevel_r1p08_walk_tests;
#[path = "bevel/rust_only_bevel_r1p09_tests.rs"]
mod rust_only_bevel_r1p09_tests;
#[path = "bevel/rust_only_bevel_r1p10_tests.rs"]
mod rust_only_bevel_r1p10_tests;
#[path = "bevel/rust_only_bevel_r1p11_tests.rs"]
mod rust_only_bevel_r1p11_tests;
#[path = "bevel/rust_only_bevel_r1p13_switch_tests.rs"]
mod rust_only_bevel_r1p13_switch_tests;
#[path = "bevel/rust_only_bevel_r1p13_tests.rs"]
mod rust_only_bevel_r1p13_tests;
#[path = "bevel/rust_only_bevel_r1p14_roll_tests.rs"]
mod rust_only_bevel_r1p14_roll_tests;
#[path = "bevel/rust_only_bevel_r1p15_tests.rs"]
mod rust_only_bevel_r1p15_tests;
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
#[path = "bevel/spoke_centre_parameter_tests.rs"]
mod spoke_centre_parameter_tests;
#[path = "bevel/surface_crossed_end_carries_what_stands_before_the_foot_tests.rs"]
mod surface_crossed_end_carries_what_stands_before_the_foot_tests;
#[path = "bevel/surface_cut_ruler_tests.rs"]
mod surface_cut_ruler_tests;
#[path = "bevel/surface_end_gains_the_section_reading_tests.rs"]
mod surface_end_gains_the_section_reading_tests;
#[path = "bevel/surface_facet_cut_own_ends_tests.rs"]
mod surface_facet_cut_own_ends_tests;
#[path = "bevel/surface_fan_tests.rs"]
mod surface_fan_tests;
#[path = "bevel/surface_mesh_closure_tests.rs"]
mod surface_mesh_closure_tests;
#[path = "bevel/surface_ring_ears_bridged_ring_tests.rs"]
mod surface_ring_ears_bridged_ring_tests;
#[path = "bevel/surface_room_crossing_tests.rs"]
mod surface_room_crossing_tests;
#[path = "bevel/surface_run_out_column_top_tests.rs"]
mod surface_run_out_column_top_tests;
#[path = "bevel/topology_test_meshes.rs"]
mod topology_test_meshes;
