//! Ported `R/Tests/GeometryProcessingTests` test classes for `Bevel/`, one file each (CLAUDE.md
//! net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "bevel/band_cut_tests.rs"]
mod band_cut_tests;
#[path = "bevel/bevel_arc_chording_tests.rs"]
mod bevel_arc_chording_tests;
#[path = "bevel/bevel_bead_batch_tests.rs"]
mod bevel_bead_batch_tests;
#[path = "bevel/bevel_bead_closes_over_stand_off_tests.rs"]
mod bevel_bead_closes_over_stand_off_tests;
#[path = "bevel/bevel_bead_exit_tests.rs"]
mod bevel_bead_exit_tests;
#[path = "bevel/bevel_bead_runs_off_its_wall_tests.rs"]
mod bevel_bead_runs_off_its_wall_tests;
#[path = "bevel/bevel_cap_clip_tests.rs"]
mod bevel_cap_clip_tests;
#[path = "bevel/bevel_cap_stand_off_tests.rs"]
mod bevel_cap_stand_off_tests;
#[path = "bevel/bevel_chain_linker_rim_closure_tests.rs"]
mod bevel_chain_linker_rim_closure_tests;
#[path = "bevel/bevel_face_fit_cap_tests.rs"]
mod bevel_face_fit_cap_tests;
#[path = "bevel/bevel_face_fit_concave_collision_tests.rs"]
mod bevel_face_fit_concave_collision_tests;
#[path = "bevel/bevel_face_fit_corner_tests.rs"]
mod bevel_face_fit_corner_tests;
#[path = "bevel/bevel_face_run_out_tests.rs"]
mod bevel_face_run_out_tests;
#[path = "bevel/bevel_folded_flap_repair_tests.rs"]
mod bevel_folded_flap_repair_tests;
#[path = "bevel/bevel_mitre_split_tests.rs"]
mod bevel_mitre_split_tests;
#[path = "bevel/bevel_obstruction_test_meshes.rs"]
mod bevel_obstruction_test_meshes;
#[path = "bevel/bevel_pair_corner_recognition_tests.rs"]
mod bevel_pair_corner_recognition_tests;
#[path = "bevel/bevel_partner_tube_clip_tests.rs"]
mod bevel_partner_tube_clip_tests;
#[path = "bevel/bevel_partnered_bead_tests.rs"]
mod bevel_partnered_bead_tests;
#[path = "bevel/bevel_patch_refinement_tests.rs"]
mod bevel_patch_refinement_tests;
#[path = "bevel/bevel_rebuild_advisories_tests.rs"]
mod bevel_rebuild_advisories_tests;
#[path = "bevel/bevel_run_partners_tests.rs"]
mod bevel_run_partners_tests;
#[path = "bevel/bevel_runs_out_under_the_wall_tests.rs"]
mod bevel_runs_out_under_the_wall_tests;
#[path = "bevel/bevel_seam_ring_tests.rs"]
mod bevel_seam_ring_tests;
#[path = "bevel/bevel_source_hull_tests.rs"]
mod bevel_source_hull_tests;
#[path = "bevel/bevel_source_round_tests.rs"]
mod bevel_source_round_tests;
#[path = "bevel/bevel_sweep_back_depth_tests.rs"]
mod bevel_sweep_back_depth_tests;
#[path = "bevel/bevel_t_vertex_sliver_repair_tests.rs"]
mod bevel_t_vertex_sliver_repair_tests;
#[path = "bevel/bevel_void_wedge_section_tests.rs"]
mod bevel_void_wedge_section_tests;
#[path = "bevel/bore_test_meshes.rs"]
mod bore_test_meshes;
#[path = "bevel/chain_field_fixtures.rs"]
mod chain_field_fixtures;
#[path = "bevel/crease_run_out_tests.rs"]
mod crease_run_out_tests;
#[path = "bevel/degenerate_triangle_refinement_tests.rs"]
mod degenerate_triangle_refinement_tests;
#[path = "bevel/end_plane_trim_corner_tests.rs"]
mod end_plane_trim_corner_tests;
#[path = "bevel/exact_refinement_tests.rs"]
mod exact_refinement_tests;
#[path = "bevel/faceted_boss_fade_tests.rs"]
mod faceted_boss_fade_tests;
#[path = "bevel/fillet_arc_seam_row_tests.rs"]
mod fillet_arc_seam_row_tests;
#[path = "bevel/flat_hole_fill_tests.rs"]
mod flat_hole_fill_tests;
#[path = "bevel/hole_fill_tests.rs"]
mod hole_fill_tests;
#[path = "bevel/hole_fill_validity_tests.rs"]
mod hole_fill_validity_tests;
#[path = "bevel/island_split_order_tests.rs"]
mod island_split_order_tests;
#[path = "bevel/level_line_fixture.rs"]
mod level_line_fixture;
#[path = "bevel/level_line_test_geometry.rs"]
mod level_line_test_geometry;
#[path = "bevel/lobed_prism_meshes.rs"]
mod lobed_prism_meshes;
#[path = "bevel/longest_edge_bisection_tests.rs"]
mod longest_edge_bisection_tests;
#[path = "bevel/meter_lock.rs"]
mod meter_lock;
#[path = "bevel/no_duplicate_vertex_tests.rs"]
mod no_duplicate_vertex_tests;
#[path = "bevel/nose_in_notch_test_meshes.rs"]
mod nose_in_notch_test_meshes;
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
#[path = "bevel/rust_only_bevel_r1p11_junction_tests.rs"]
mod rust_only_bevel_r1p11_junction_tests;
#[path = "bevel/rust_only_bevel_r1p11_tests.rs"]
mod rust_only_bevel_r1p11_tests;
#[path = "bevel/rust_only_bevel_r1p12_tests.rs"]
mod rust_only_bevel_r1p12_tests;
#[path = "bevel/rust_only_bevel_r1p13_field_tests.rs"]
mod rust_only_bevel_r1p13_field_tests;
#[path = "bevel/rust_only_bevel_r1p13_switch_tests.rs"]
mod rust_only_bevel_r1p13_switch_tests;
#[path = "bevel/rust_only_bevel_r1p13_tests.rs"]
mod rust_only_bevel_r1p13_tests;
#[path = "bevel/rust_only_bevel_r1p14_roll_tests.rs"]
mod rust_only_bevel_r1p14_roll_tests;
#[path = "bevel/rust_only_bevel_r1p14_tests.rs"]
mod rust_only_bevel_r1p14_tests;
#[path = "bevel/rust_only_bevel_r1p15_tests.rs"]
mod rust_only_bevel_r1p15_tests;
#[path = "bevel/rust_only_bevel_r2p01_tests.rs"]
mod rust_only_bevel_r2p01_tests;
#[path = "bevel/rust_only_bevel_r2p02_field_tests.rs"]
mod rust_only_bevel_r2p02_field_tests;
#[path = "bevel/rust_only_bevel_r2p02_fill_tests.rs"]
mod rust_only_bevel_r2p02_fill_tests;
#[path = "bevel/rust_only_bevel_r2p02_tests.rs"]
mod rust_only_bevel_r2p02_tests;
#[path = "bevel/rust_only_bevel_r2p02_walk_tests.rs"]
mod rust_only_bevel_r2p02_walk_tests;
#[path = "bevel/rust_only_bevel_r2p03_tests.rs"]
mod rust_only_bevel_r2p03_tests;
#[path = "bevel/rust_only_bevel_r2p04_flow_tests.rs"]
mod rust_only_bevel_r2p04_flow_tests;
#[path = "bevel/rust_only_bevel_r2p04_pass_b_tests.rs"]
mod rust_only_bevel_r2p04_pass_b_tests;
#[path = "bevel/rust_only_bevel_r2p04_tests.rs"]
mod rust_only_bevel_r2p04_tests;
#[path = "bevel/rust_only_bevel_r2p05_counter_tests.rs"]
mod rust_only_bevel_r2p05_counter_tests;
#[path = "bevel/rust_only_bevel_r2p05_crease_tests.rs"]
mod rust_only_bevel_r2p05_crease_tests;
#[path = "bevel/rust_only_bevel_r2p05_from_polyline_tests.rs"]
mod rust_only_bevel_r2p05_from_polyline_tests;
#[path = "bevel/rust_only_bevel_r2p05_mesh_tests.rs"]
mod rust_only_bevel_r2p05_mesh_tests;
#[path = "bevel/rust_only_bevel_r2p08_tests.rs"]
mod rust_only_bevel_r2p08_tests;
#[path = "bevel/rust_only_bevel_r2p09_tests.rs"]
mod rust_only_bevel_r2p09_tests;
#[path = "bevel/rust_only_bevel_r3p02_tests.rs"]
mod rust_only_bevel_r3p02_tests;
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
#[path = "bevel/section_loop_entry_tests.rs"]
mod section_loop_entry_tests;
#[path = "bevel/section_room_split_crease_tests.rs"]
mod section_room_split_crease_tests;
#[path = "bevel/section_surface_walk_fold_tests.rs"]
mod section_surface_walk_fold_tests;
#[path = "bevel/section_surface_walk_roof.rs"]
mod section_surface_walk_roof;
#[path = "bevel/source_round_test_meshes.rs"]
mod source_round_test_meshes;
#[path = "bevel/spoke_centre_parameter_tests.rs"]
mod spoke_centre_parameter_tests;
#[path = "bevel/surface_crossed_end_carries_what_stands_before_the_foot_tests.rs"]
mod surface_crossed_end_carries_what_stands_before_the_foot_tests;
#[path = "bevel/surface_cut_ruler_tests.rs"]
mod surface_cut_ruler_tests;
#[path = "bevel/surface_end_cut_leaves_the_die_points_chord_alone_tests.rs"]
mod surface_end_cut_leaves_the_die_points_chord_alone_tests;
#[path = "bevel/surface_end_gains_the_section_reading_tests.rs"]
mod surface_end_gains_the_section_reading_tests;
#[path = "bevel/surface_facet_cut_own_ends_tests.rs"]
mod surface_facet_cut_own_ends_tests;
#[path = "bevel/surface_fan_sliver_ring_tests.rs"]
mod surface_fan_sliver_ring_tests;
#[path = "bevel/surface_fan_tests.rs"]
mod surface_fan_tests;
#[path = "bevel/surface_giving_up_the_section_sews_by_index_tests.rs"]
mod surface_giving_up_the_section_sews_by_index_tests;
#[path = "bevel/surface_mesh_closure_tests.rs"]
mod surface_mesh_closure_tests;
#[path = "bevel/surface_ring_ears_bridged_ring_tests.rs"]
mod surface_ring_ears_bridged_ring_tests;
#[path = "bevel/surface_room_crossing_tests.rs"]
mod surface_room_crossing_tests;
#[path = "bevel/surface_run_out_column_top_tests.rs"]
mod surface_run_out_column_top_tests;
#[path = "bevel/surface_run_out_strip_top_rim_tests.rs"]
mod surface_run_out_strip_top_rim_tests;
#[path = "bevel/surface_sewing_bowed_side_tests.rs"]
mod surface_sewing_bowed_side_tests;
#[path = "bevel/surface_splits_on_a_segment_tests.rs"]
mod surface_splits_on_a_segment_tests;
#[path = "bevel/topology_test_meshes.rs"]
mod topology_test_meshes;
#[path = "bevel/warped_wall_test_meshes.rs"]
mod warped_wall_test_meshes;
