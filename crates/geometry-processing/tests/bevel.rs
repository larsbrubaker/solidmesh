//! Ported `R/Tests/GeometryProcessingTests` test classes for `Bevel/`, one file each (CLAUDE.md
//! net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.

#[path = "bevel/annulus_plate_builder.rs"]
mod annulus_plate_builder;
#[path = "bevel/around_center_kink_measure_tests.rs"]
mod around_center_kink_measure_tests;
#[path = "bevel/around_center_mitre_knife_tests.rs"]
mod around_center_mitre_knife_tests;
#[path = "bevel/band_cut_tests.rs"]
mod band_cut_tests;
#[path = "bevel/band_gap_tests.rs"]
mod band_gap_tests;
#[path = "bevel/band_side_end_tag_tests.rs"]
mod band_side_end_tag_tests;
#[path = "bevel/bead_sequence.rs"]
mod bead_sequence;
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
#[path = "bevel/bevel_blocked_crease_end_tests.rs"]
mod bevel_blocked_crease_end_tests;
#[path = "bevel/bevel_canal_over_the_round_tests.rs"]
mod bevel_canal_over_the_round_tests;
#[path = "bevel/bevel_cap_clip_tests.rs"]
mod bevel_cap_clip_tests;
#[path = "bevel/bevel_cap_stand_off_tests.rs"]
mod bevel_cap_stand_off_tests;
#[path = "bevel/bevel_center_bottom_corner_tests.rs"]
mod bevel_center_bottom_corner_tests;
#[path = "bevel/bevel_centre_flows_through_tests.rs"]
mod bevel_centre_flows_through_tests;
#[path = "bevel/bevel_chain_linker_rim_closure_tests.rs"]
mod bevel_chain_linker_rim_closure_tests;
#[path = "bevel/bevel_collinear_run_fold_tests.rs"]
mod bevel_collinear_run_fold_tests;
#[path = "bevel/bevel_corner_void_tests.rs"]
mod bevel_corner_void_tests;
#[path = "bevel/bevel_crease_over_a_rounded_ridge_tests.rs"]
mod bevel_crease_over_a_rounded_ridge_tests;
#[path = "bevel/bevel_drive_through_guard_tests.rs"]
mod bevel_drive_through_guard_tests;
#[path = "bevel/bevel_ends_measure_tests.rs"]
mod bevel_ends_measure_tests;
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
#[path = "bevel/bevel_gusset_poke_through_tests.rs"]
mod bevel_gusset_poke_through_tests;
#[path = "bevel/bevel_invariants_measure_tests.rs"]
mod bevel_invariants_measure_tests;
#[path = "bevel/bevel_knife_edge_rim_tests.rs"]
mod bevel_knife_edge_rim_tests;
#[path = "bevel/bevel_mitre_split_tests.rs"]
mod bevel_mitre_split_tests;
#[path = "bevel/bevel_obstruction_test_meshes.rs"]
mod bevel_obstruction_test_meshes;
#[path = "bevel/bevel_oversize_run_out_tests.rs"]
mod bevel_oversize_run_out_tests;
#[path = "bevel/bevel_pair_corner_recognition_tests.rs"]
mod bevel_pair_corner_recognition_tests;
#[path = "bevel/bevel_pair_corner_trim_tests.rs"]
mod bevel_pair_corner_trim_tests;
#[path = "bevel/bevel_partner_tube_clip_tests.rs"]
mod bevel_partner_tube_clip_tests;
#[path = "bevel/bevel_partnered_bead_tests.rs"]
mod bevel_partnered_bead_tests;
#[path = "bevel/bevel_patch_refinement_tests.rs"]
mod bevel_patch_refinement_tests;
#[path = "bevel/bevel_published_mesh_sliver_tests.rs"]
mod bevel_published_mesh_sliver_tests;
#[path = "bevel/bevel_published_mesh_tests.rs"]
mod bevel_published_mesh_tests;
#[path = "bevel/bevel_rebuild_advisories_tests.rs"]
mod bevel_rebuild_advisories_tests;
#[path = "bevel/bevel_recess_apex_run_out_tests.rs"]
mod bevel_recess_apex_run_out_tests;
#[path = "bevel/bevel_run_partners_tests.rs"]
mod bevel_run_partners_tests;
#[path = "bevel/bevel_run_taper_tests.rs"]
mod bevel_run_taper_tests;
#[path = "bevel/bevel_runs_out_under_the_wall_tests.rs"]
mod bevel_runs_out_under_the_wall_tests;
#[path = "bevel/bevel_seam_ring_tests.rs"]
mod bevel_seam_ring_tests;
#[path = "bevel/bevel_second_pass_merge_tests.rs"]
mod bevel_second_pass_merge_tests;
#[path = "bevel/bevel_shadow_body_tests.rs"]
mod bevel_shadow_body_tests;
#[path = "bevel/bevel_shadow_clip_tests.rs"]
mod bevel_shadow_clip_tests;
#[path = "bevel/bevel_source_hull_tests.rs"]
mod bevel_source_hull_tests;
#[path = "bevel/bevel_source_round_tests.rs"]
mod bevel_source_round_tests;
#[path = "bevel/bevel_sweep_back_depth_tests.rs"]
mod bevel_sweep_back_depth_tests;
#[path = "bevel/bevel_sweep_batches_tests.rs"]
mod bevel_sweep_batches_tests;
#[path = "bevel/bevel_t_vertex_sliver_repair_tests.rs"]
mod bevel_t_vertex_sliver_repair_tests;
#[path = "bevel/bevel_taper_fallback_tests.rs"]
mod bevel_taper_fallback_tests;
#[path = "bevel/bevel_thin_sliver_conditioning_tests.rs"]
mod bevel_thin_sliver_conditioning_tests;
#[path = "bevel/bevel_three_run_corner_tests.rs"]
mod bevel_three_run_corner_tests;
#[path = "bevel/bevel_to_center_blend_crease_tests.rs"]
mod bevel_to_center_blend_crease_tests;
#[path = "bevel/bevel_to_center_pair_corner_tests.rs"]
mod bevel_to_center_pair_corner_tests;
#[path = "bevel/bevel_void_wedge_section_tests.rs"]
mod bevel_void_wedge_section_tests;
#[path = "bevel/bevel_volume_probes.rs"]
mod bevel_volume_probes;
#[path = "bevel/boolean_kernel_gate.rs"]
mod boolean_kernel_gate;
#[path = "bevel/bore_test_meshes.rs"]
mod bore_test_meshes;
#[path = "bevel/both_sided_member_tests.rs"]
mod both_sided_member_tests;
#[path = "bevel/bridge_diagnosis_tests.rs"]
mod bridge_diagnosis_tests;
#[path = "bevel/bridge_fixture_tests.rs"]
mod bridge_fixture_tests;
#[path = "bevel/canonical_maximum_tests.rs"]
mod canonical_maximum_tests;
#[path = "bevel/center_slanted_chain_tests.rs"]
mod center_slanted_chain_tests;
#[path = "bevel/chain_field_end_clip_tests.rs"]
mod chain_field_end_clip_tests;
#[path = "bevel/chain_field_end_ray_tests.rs"]
mod chain_field_end_ray_tests;
#[path = "bevel/chain_field_fixtures.rs"]
mod chain_field_fixtures;
#[path = "bevel/chain_field_mitre_tests.rs"]
mod chain_field_mitre_tests;
#[path = "bevel/chain_field_speed_tests.rs"]
mod chain_field_speed_tests;
#[path = "bevel/chain_field_tests.rs"]
mod chain_field_tests;
#[path = "bevel/chain_to_chain_room_tests.rs"]
mod chain_to_chain_room_tests;
#[path = "bevel/chamfer_turn_tests.rs"]
mod chamfer_turn_tests;
#[path = "bevel/clipped_cell_bound_tests.rs"]
mod clipped_cell_bound_tests;
#[path = "bevel/closed_loop_defect_tests.rs"]
mod closed_loop_defect_tests;
#[path = "bevel/concave_corner_rim_fill_tests.rs"]
mod concave_corner_rim_fill_tests;
#[path = "bevel/concave_edge_end_face_tests.rs"]
mod concave_edge_end_face_tests;
#[path = "bevel/concave_end_cap_rows_tests.rs"]
mod concave_end_cap_rows_tests;
#[path = "bevel/concave_rim_fill_tests.rs"]
mod concave_rim_fill_tests;
#[path = "bevel/convex_cell_bound_tests.rs"]
mod convex_cell_bound_tests;
#[path = "bevel/convex_end_ray_room_tests.rs"]
mod convex_end_ray_room_tests;
#[path = "bevel/coplanar_diagonal_flip.rs"]
mod coplanar_diagonal_flip;
#[path = "bevel/corner_pick_with_mouth_tests.rs"]
mod corner_pick_with_mouth_tests;
#[path = "bevel/corner_setback_measure_tests.rs"]
mod corner_setback_measure_tests;
#[path = "bevel/corner_volume_measure_tests.rs"]
mod corner_volume_measure_tests;
#[path = "bevel/crease_dies_out_fixtures.rs"]
mod crease_dies_out_fixtures;
#[path = "bevel/crease_dies_out_tests.rs"]
mod crease_dies_out_tests;
#[path = "bevel/crease_run_out_tests.rs"]
mod crease_run_out_tests;
#[path = "bevel/cube_corner_ball_tests.rs"]
mod cube_corner_ball_tests;
#[path = "bevel/cube_corner_fill_tests.rs"]
mod cube_corner_fill_tests;
#[path = "bevel/cube_corner_measure_tests.rs"]
mod cube_corner_measure_tests;
#[path = "bevel/cube_corner_tests.rs"]
mod cube_corner_tests;
#[path = "bevel/cube_edge_end_cap_tests.rs"]
mod cube_edge_end_cap_tests;
#[path = "bevel/cube_edge_union_tests.rs"]
mod cube_edge_union_tests;
#[path = "bevel/curling_side_run_concavity_tests.rs"]
mod curling_side_run_concavity_tests;
#[path = "bevel/degenerate_triangle_refinement_tests.rs"]
mod degenerate_triangle_refinement_tests;
#[path = "bevel/edge_loop_and_face_test_mesh.rs"]
mod edge_loop_and_face_test_mesh;
#[path = "bevel/encroaching_side_room_tests.rs"]
mod encroaching_side_room_tests;
#[path = "bevel/end_face_builds.rs"]
mod end_face_builds;
#[path = "bevel/end_face_carry_tests.rs"]
mod end_face_carry_tests;
#[path = "bevel/end_face_measures.rs"]
mod end_face_measures;
#[path = "bevel/end_face_room_tests.rs"]
mod end_face_room_tests;
#[path = "bevel/end_plane_trim_corner_tests.rs"]
mod end_plane_trim_corner_tests;
#[path = "bevel/end_ray_reach_tests.rs"]
mod end_ray_reach_tests;
#[path = "bevel/end_region_shapes_tests.rs"]
mod end_region_shapes_tests;
#[path = "bevel/exact_refinement_tests.rs"]
mod exact_refinement_tests;
#[path = "bevel/face_rim_corner_mirror_tests.rs"]
mod face_rim_corner_mirror_tests;
#[path = "bevel/faceted_boss_fade_tests.rs"]
mod faceted_boss_fade_tests;
#[path = "bevel/faceted_wall_chamfer_flow_tests.rs"]
mod faceted_wall_chamfer_flow_tests;
#[path = "bevel/fade_end_room_tests.rs"]
mod fade_end_room_tests;
#[path = "bevel/fade_end_row_fold_tests.rs"]
mod fade_end_row_fold_tests;
#[path = "bevel/fifteen_mm_room_reading_tests.rs"]
mod fifteen_mm_room_reading_tests;
#[path = "bevel/figure_eight_rim_tests.rs"]
mod figure_eight_rim_tests;
#[path = "bevel/fill_closes_or_refuses_tests.rs"]
mod fill_closes_or_refuses_tests;
#[path = "bevel/fillet_arc_carry_tests.rs"]
mod fillet_arc_carry_tests;
#[path = "bevel/fillet_arc_seam_row_tests.rs"]
mod fillet_arc_seam_row_tests;
#[path = "bevel/fillet_strip_sag_tests.rs"]
mod fillet_strip_sag_tests;
#[path = "bevel/fillet_thin_front_fin.rs"]
mod fillet_thin_front_fin;
#[path = "bevel/five_hole_plate_fixture.rs"]
mod five_hole_plate_fixture;
#[path = "bevel/five_hole_plate_tests.rs"]
mod five_hole_plate_tests;
#[path = "bevel/flat_ends_clip_tests.rs"]
mod flat_ends_clip_tests;
#[path = "bevel/flat_ends_combination_tests.rs"]
mod flat_ends_combination_tests;
#[path = "bevel/flat_ends_every_chamfer_seam_tests.rs"]
mod flat_ends_every_chamfer_seam_tests;
#[path = "bevel/flat_hole_fill_tests.rs"]
mod flat_hole_fill_tests;
#[path = "bevel/foreign_edge_index_tests.rs"]
mod foreign_edge_index_tests;
#[path = "bevel/gap_fill_tests.rs"]
mod gap_fill_tests;
#[path = "bevel/goes_to_sides_end_tests.rs"]
mod goes_to_sides_end_tests;
#[path = "bevel/graze_fixture.rs"]
mod graze_fixture;
#[path = "bevel/graze_fixture_tests.rs"]
mod graze_fixture_tests;
#[path = "bevel/grid_hole_plate_fixture.rs"]
mod grid_hole_plate_fixture;
#[path = "bevel/growing_crease_room_source_tests.rs"]
mod growing_crease_room_source_tests;
#[path = "bevel/gusset_test_meshes.rs"]
mod gusset_test_meshes;
#[path = "bevel/handover_corner_test_mesh.rs"]
mod handover_corner_test_mesh;
#[path = "bevel/hole_fill_corner_tests.rs"]
mod hole_fill_corner_tests;
#[path = "bevel/hole_fill_density_tests.rs"]
mod hole_fill_density_tests;
#[path = "bevel/hole_fill_flat_wall_tests.rs"]
mod hole_fill_flat_wall_tests;
#[path = "bevel/hole_fill_free_end_tests.rs"]
mod hole_fill_free_end_tests;
#[path = "bevel/hole_fill_one_loop_corner_tests.rs"]
mod hole_fill_one_loop_corner_tests;
#[path = "bevel/hole_fill_tests.rs"]
mod hole_fill_tests;
#[path = "bevel/hole_fill_validity_tests.rs"]
mod hole_fill_validity_tests;
#[path = "bevel/hole_row_bridge_fixture.rs"]
mod hole_row_bridge_fixture;
#[path = "bevel/hole_row_bridge_tests.rs"]
mod hole_row_bridge_tests;
#[path = "bevel/hourglass_beside_square_fixture.rs"]
mod hourglass_beside_square_fixture;
#[path = "bevel/hourglass_beside_square_tests.rs"]
mod hourglass_beside_square_tests;
#[path = "bevel/hull_clip_tests.rs"]
mod hull_clip_tests;
#[path = "bevel/inner_face_bore_carry_tests.rs"]
mod inner_face_bore_carry_tests;
#[path = "bevel/inner_face_test_mesh.rs"]
mod inner_face_test_mesh;
#[path = "bevel/input_needle_strip_tests.rs"]
mod input_needle_strip_tests;
#[path = "bevel/insert_polyline_tests.rs"]
mod insert_polyline_tests;
#[path = "bevel/island_split_order_tests.rs"]
mod island_split_order_tests;
#[path = "bevel/kernel_cut_helpers.rs"]
mod kernel_cut_helpers;
#[path = "bevel/knife_edge_rim_measures.rs"]
mod knife_edge_rim_measures;
#[path = "bevel/known_room_refuses_first_tests.rs"]
mod known_room_refuses_first_tests;
#[path = "bevel/land_sliver_joins_two_bands_tests.rs"]
mod land_sliver_joins_two_bands_tests;
#[path = "bevel/level_line_cost_gate.rs"]
mod level_line_cost_gate;
#[path = "bevel/level_line_feature_edges.rs"]
mod level_line_feature_edges;
#[path = "bevel/level_line_fixture.rs"]
mod level_line_fixture;
#[path = "bevel/level_line_room_geometry.rs"]
mod level_line_room_geometry;
#[path = "bevel/level_line_test_geometry.rs"]
mod level_line_test_geometry;
#[path = "bevel/level_line_test_solids.rs"]
mod level_line_test_solids;
#[path = "bevel/level_line_triangulation_invariance_tests.rs"]
mod level_line_triangulation_invariance_tests;
#[path = "bevel/line_to_line_fixture.rs"]
mod line_to_line_fixture;
#[path = "bevel/lobed_prism_meshes.rs"]
mod lobed_prism_meshes;
#[path = "bevel/longest_edge_bisection_tests.rs"]
mod longest_edge_bisection_tests;
#[path = "bevel/many_chain_build_cost_tests.rs"]
mod many_chain_build_cost_tests;
#[path = "bevel/many_chain_foreign_cost_tests.rs"]
mod many_chain_foreign_cost_tests;
#[path = "bevel/many_holes_mouth_room_tests.rs"]
mod many_holes_mouth_room_tests;
#[path = "bevel/many_holes_rim_cost_tests.rs"]
mod many_holes_rim_cost_tests;
#[path = "bevel/mesh_measurements.rs"]
mod mesh_measurements;
#[path = "bevel/mesh_surface_distance.rs"]
mod mesh_surface_distance;
#[path = "bevel/meter_lock.rs"]
mod meter_lock;
#[path = "bevel/micro_step_seam_tests.rs"]
mod micro_step_seam_tests;
#[path = "bevel/mirrored_hips_and_ridge_tests.rs"]
mod mirrored_hips_and_ridge_tests;
#[path = "bevel/mirrored_pick_tests.rs"]
mod mirrored_pick_tests;
#[path = "bevel/mixed_end_fade_tests.rs"]
mod mixed_end_fade_tests;
#[path = "bevel/name_every_tight_chain_tests.rs"]
mod name_every_tight_chain_tests;
#[path = "bevel/needle_cascade_meter_tests.rs"]
mod needle_cascade_meter_tests;
#[path = "bevel/needle_repair_tests.rs"]
mod needle_repair_tests;
#[path = "bevel/no_duplicate_vertex_tests.rs"]
mod no_duplicate_vertex_tests;
#[path = "bevel/nose_in_notch_cut.rs"]
mod nose_in_notch_cut;
#[path = "bevel/nose_in_notch_mixed_corners.rs"]
mod nose_in_notch_mixed_corners;
#[path = "bevel/nose_in_notch_test_meshes.rs"]
mod nose_in_notch_test_meshes;
#[path = "bevel/oblique_end_face_owner_tests.rs"]
mod oblique_end_face_owner_tests;
#[path = "bevel/oblique_end_face_tests.rs"]
mod oblique_end_face_tests;
#[path = "bevel/one_room_reading_tests.rs"]
mod one_room_reading_tests;
#[path = "bevel/open_chain_fade_tests.rs"]
mod open_chain_fade_tests;
#[path = "bevel/open_chain_fixtures.rs"]
mod open_chain_fixtures;
#[path = "bevel/open_mouth_arc_walk.rs"]
mod open_mouth_arc_walk;
#[path = "bevel/opposite_sense_crease_tests.rs"]
mod opposite_sense_crease_tests;
#[path = "bevel/opposite_sense_end_face_tests.rs"]
mod opposite_sense_end_face_tests;
#[path = "bevel/outer_faces_corner_tests.rs"]
mod outer_faces_corner_tests;
#[path = "bevel/own_setback_per_edge_tests.rs"]
mod own_setback_per_edge_tests;
#[path = "bevel/parallel_edges_room_tests.rs"]
mod parallel_edges_room_tests;
#[path = "bevel/pass_one_pinch_tests.rs"]
mod pass_one_pinch_tests;
#[path = "bevel/pick_field_tests.rs"]
mod pick_field_tests;
#[path = "bevel/picture_hanger_test_mesh.rs"]
mod picture_hanger_test_mesh;
#[path = "bevel/pinned_chain_end_tests.rs"]
mod pinned_chain_end_tests;
#[path = "bevel/point_bevel_tests.rs"]
mod point_bevel_tests;
#[path = "bevel/port_mouth_carry_tests.rs"]
mod port_mouth_carry_tests;
#[path = "bevel/pyramid_recess_test_meshes.rs"]
mod pyramid_recess_test_meshes;
#[path = "bevel/rail_monotone_tests.rs"]
mod rail_monotone_tests;
#[path = "bevel/raked_box_fixture.rs"]
mod raked_box_fixture;
#[path = "bevel/ridge_search_cost_tests.rs"]
mod ridge_search_cost_tests;
#[path = "bevel/ridge_survives_tests.rs"]
mod ridge_survives_tests;
#[path = "bevel/rigid_motion_meshes.rs"]
mod rigid_motion_meshes;
#[path = "bevel/room_ceiling_tests.rs"]
mod room_ceiling_tests;
#[path = "bevel/room_pairs_cost_tests.rs"]
mod room_pairs_cost_tests;
#[path = "bevel/room_read_cost_tests.rs"]
mod room_read_cost_tests;
#[path = "bevel/room_rigid_motion_tests.rs"]
mod room_rigid_motion_tests;
#[path = "bevel/room_scales_with_the_part_tests.rs"]
mod room_scales_with_the_part_tests;
#[path = "bevel/round_mouth_fillet_crossings.rs"]
mod round_mouth_fillet_crossings;
#[path = "bevel/round_mouth_fillet_tests.rs"]
mod round_mouth_fillet_tests;
#[path = "bevel/rounds_first_grow_over_end_tests.rs"]
mod rounds_first_grow_over_end_tests;
#[path = "bevel/rounds_first_tests.rs"]
mod rounds_first_tests;
#[path = "bevel/run_out_field_tests.rs"]
mod run_out_field_tests;
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
#[path = "bevel/rust_only_bevel_r2p08_mixed_corner_tests.rs"]
mod rust_only_bevel_r2p08_mixed_corner_tests;
#[path = "bevel/rust_only_bevel_r2p08_tests.rs"]
mod rust_only_bevel_r2p08_tests;
#[path = "bevel/rust_only_bevel_r2p09_tests.rs"]
mod rust_only_bevel_r2p09_tests;
#[path = "bevel/rust_only_bevel_r3p02_tests.rs"]
mod rust_only_bevel_r3p02_tests;
#[path = "bevel/rust_only_bevel_r3p03_tests.rs"]
mod rust_only_bevel_r3p03_tests;
#[path = "bevel/rust_only_bevel_r4p01_tests.rs"]
mod rust_only_bevel_r4p01_tests;
#[path = "bevel/rust_only_bevel_s18_tests.rs"]
mod rust_only_bevel_s18_tests;
#[path = "bevel/rust_only_bevel_section_routing_tests.rs"]
mod rust_only_bevel_section_routing_tests;
#[path = "bevel/rust_only_bevel_slice10_tests.rs"]
mod rust_only_bevel_slice10_tests;
#[path = "bevel/rust_only_bevel_slice12_tests.rs"]
mod rust_only_bevel_slice12_tests;
#[path = "bevel/rust_only_bevel_slice17_tests.rs"]
mod rust_only_bevel_slice17_tests;
#[path = "bevel/rust_only_bevel_slice19_tests.rs"]
mod rust_only_bevel_slice19_tests;
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
#[path = "bevel/rust_only_bevel_sliver_collapse_tests.rs"]
mod rust_only_bevel_sliver_collapse_tests;
#[path = "bevel/rust_only_section_smooth_seam_tests.rs"]
mod rust_only_section_smooth_seam_tests;
#[path = "bevel/saved_pick_self_intersection_tests.rs"]
mod saved_pick_self_intersection_tests;
#[path = "bevel/second_column_land_join_tests.rs"]
mod second_column_land_join_tests;
#[path = "bevel/section_assembly_tests.rs"]
mod section_assembly_tests;
#[path = "bevel/section_face_rebuild_tests.rs"]
mod section_face_rebuild_tests;
#[path = "bevel/section_hanger_top_rim_tests.rs"]
mod section_hanger_top_rim_tests;
#[path = "bevel/section_loft_junction_tests.rs"]
mod section_loft_junction_tests;
#[path = "bevel/section_loft_tests.rs"]
mod section_loft_tests;
#[path = "bevel/section_loop_entry_tests.rs"]
mod section_loop_entry_tests;
#[path = "bevel/section_pierced_face_junction_tests.rs"]
mod section_pierced_face_junction_tests;
#[path = "bevel/section_room_split_crease_tests.rs"]
mod section_room_split_crease_tests;
#[path = "bevel/section_routing_opt_in_tests.rs"]
mod section_routing_opt_in_tests;
#[path = "bevel/section_routing_partition_tests.rs"]
mod section_routing_partition_tests;
#[path = "bevel/section_smooth_side_bridged_holes_tests.rs"]
mod section_smooth_side_bridged_holes_tests;
#[path = "bevel/section_smooth_side_tests.rs"]
mod section_smooth_side_tests;
#[path = "bevel/section_straight_bore_tests.rs"]
mod section_straight_bore_tests;
#[path = "bevel/section_surface_walk_arc_join_tests.rs"]
mod section_surface_walk_arc_join_tests;
#[path = "bevel/section_surface_walk_fold_tests.rs"]
mod section_surface_walk_fold_tests;
#[path = "bevel/section_surface_walk_roof.rs"]
mod section_surface_walk_roof;
#[path = "bevel/section_tapered_barrel_tests.rs"]
mod section_tapered_barrel_tests;
#[path = "bevel/section_walked_end_vertex_tests.rs"]
mod section_walked_end_vertex_tests;
#[path = "bevel/seeded_search_cost_tests.rs"]
mod seeded_search_cost_tests;
#[path = "bevel/segment_cell_bound_tests.rs"]
mod segment_cell_bound_tests;
#[path = "bevel/self_crossing_chain_tests.rs"]
mod self_crossing_chain_tests;
#[path = "bevel/self_room_varying_setback_tests.rs"]
mod self_room_varying_setback_tests;
#[path = "bevel/several_chains_fill_tests.rs"]
mod several_chains_fill_tests;
#[path = "bevel/several_chains_fixtures.rs"]
mod several_chains_fixtures;
#[path = "bevel/several_chains_support.rs"]
mod several_chains_support;
#[path = "bevel/several_chains_tests.rs"]
mod several_chains_tests;
#[path = "bevel/side_witness_tests.rs"]
mod side_witness_tests;
#[path = "bevel/skew_row_cross_tangent_tests.rs"]
mod skew_row_cross_tangent_tests;
#[path = "bevel/slanted_face_rim_tests.rs"]
mod slanted_face_rim_tests;
#[path = "bevel/source_round_test_meshes.rs"]
mod source_round_test_meshes;
#[path = "bevel/spoke_centre_parameter_tests.rs"]
mod spoke_centre_parameter_tests;
#[path = "bevel/stair_step_chain_tests.rs"]
mod stair_step_chain_tests;
#[path = "bevel/step_corner_rim_tests.rs"]
mod step_corner_rim_tests;
#[path = "bevel/strip_clip_rib_meshes.rs"]
mod strip_clip_rib_meshes;
#[path = "bevel/strip_clip_rib_tests.rs"]
mod strip_clip_rib_tests;
#[path = "bevel/surface_bevel_plan_tests.rs"]
mod surface_bevel_plan_tests;
#[path = "bevel/surface_chain_opens_at_the_junction_tests.rs"]
mod surface_chain_opens_at_the_junction_tests;
#[path = "bevel/surface_clip_classifies_one_point_one_way_tests.rs"]
mod surface_clip_classifies_one_point_one_way_tests;
#[path = "bevel/surface_concave_end_where_the_face_carries_on_tests.rs"]
mod surface_concave_end_where_the_face_carries_on_tests;
#[path = "bevel/surface_convex_bite_tests.rs"]
mod surface_convex_bite_tests;
#[path = "bevel/surface_crease_faces_are_one_surface_tests.rs"]
mod surface_crease_faces_are_one_surface_tests;
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
#[path = "bevel/surface_facet_knits_across_its_cuts_tests.rs"]
mod surface_facet_knits_across_its_cuts_tests;
#[path = "bevel/surface_fan_sliver_ring_tests.rs"]
mod surface_fan_sliver_ring_tests;
#[path = "bevel/surface_fan_tests.rs"]
mod surface_fan_tests;
#[path = "bevel/surface_giving_up_the_section_sews_by_index_tests.rs"]
mod surface_giving_up_the_section_sews_by_index_tests;
#[path = "bevel/surface_mesh_closure_tests.rs"]
mod surface_mesh_closure_tests;
#[path = "bevel/surface_mixed_corner_dual_canal_tests.rs"]
mod surface_mixed_corner_dual_canal_tests;
#[path = "bevel/surface_mixed_corner_dual_grid_tests.rs"]
mod surface_mixed_corner_dual_grid_tests;
#[path = "bevel/surface_mixed_corner_dual_on_a_square_corner_tests.rs"]
mod surface_mixed_corner_dual_on_a_square_corner_tests;
#[path = "bevel/surface_mixed_corner_reading_tests.rs"]
mod surface_mixed_corner_reading_tests;
#[path = "bevel/surface_mixed_corner_run_out_second_wall_tests.rs"]
mod surface_mixed_corner_run_out_second_wall_tests;
#[path = "bevel/surface_oblique_mixed_corner_bites_tests.rs"]
mod surface_oblique_mixed_corner_bites_tests;
#[path = "bevel/surface_oblique_mixed_corner_wall_spine_tests.rs"]
mod surface_oblique_mixed_corner_wall_spine_tests;
#[path = "bevel/surface_plan_cost_tests.rs"]
mod surface_plan_cost_tests;
#[path = "bevel/surface_raked_bore_band_outline_tests.rs"]
mod surface_raked_bore_band_outline_tests;
#[path = "bevel/surface_raked_bore_mouth_tests.rs"]
mod surface_raked_bore_mouth_tests;
#[path = "bevel/surface_rim_closes_at_a_knife_edge_tests.rs"]
mod surface_rim_closes_at_a_knife_edge_tests;
#[path = "bevel/surface_ring_ears_bridged_ring_tests.rs"]
mod surface_ring_ears_bridged_ring_tests;
#[path = "bevel/surface_room_across_a_curved_wall_tests.rs"]
mod surface_room_across_a_curved_wall_tests;
#[path = "bevel/surface_room_corner_wall_tests.rs"]
mod surface_room_corner_wall_tests;
#[path = "bevel/surface_room_crossing_tests.rs"]
mod surface_room_crossing_tests;
#[path = "bevel/surface_room_half_disc_rim_tests.rs"]
mod surface_room_half_disc_rim_tests;
#[path = "bevel/surface_room_notch_reach_tests.rs"]
mod surface_room_notch_reach_tests;
#[path = "bevel/surface_room_partial_run_off_tests.rs"]
mod surface_room_partial_run_off_tests;
#[path = "bevel/surface_room_standing_feature_tests.rs"]
mod surface_room_standing_feature_tests;
#[path = "bevel/surface_room_tests.rs"]
mod surface_room_tests;
#[path = "bevel/surface_run_out_column_top_tests.rs"]
mod surface_run_out_column_top_tests;
#[path = "bevel/surface_run_out_lens_readings_tests.rs"]
mod surface_run_out_lens_readings_tests;
#[path = "bevel/surface_run_out_strip_top_rim_tests.rs"]
mod surface_run_out_strip_top_rim_tests;
#[path = "bevel/surface_seam_window_tests.rs"]
mod surface_seam_window_tests;
#[path = "bevel/surface_sewing_bowed_side_tests.rs"]
mod surface_sewing_bowed_side_tests;
#[path = "bevel/surface_splits_on_a_segment_tests.rs"]
mod surface_splits_on_a_segment_tests;
#[path = "bevel/surface_standing_feature_stretch_tests.rs"]
mod surface_standing_feature_stretch_tests;
#[path = "bevel/surface_standing_feature_tail_and_cap_tests.rs"]
mod surface_standing_feature_tail_and_cap_tests;
#[path = "bevel/surface_strip_bitten_by_a_feature_tests.rs"]
mod surface_strip_bitten_by_a_feature_tests;
#[path = "bevel/surface_strip_ends_on_the_hull_tests.rs"]
mod surface_strip_ends_on_the_hull_tests;
#[path = "bevel/surface_strip_mitre_rails_tests.rs"]
mod surface_strip_mitre_rails_tests;
#[path = "bevel/surface_strip_oblique_end_tests.rs"]
mod surface_strip_oblique_end_tests;
#[path = "bevel/surface_tangent_run_ends_tests.rs"]
mod surface_tangent_run_ends_tests;
#[path = "bevel/surface_test_chains.rs"]
mod surface_test_chains;
#[path = "bevel/surface_two_crease_junction_reading_tests.rs"]
mod surface_two_crease_junction_reading_tests;
#[path = "bevel/surface_two_crease_junction_rings_tests.rs"]
mod surface_two_crease_junction_rings_tests;
#[path = "bevel/surface_walkback_checks.rs"]
mod surface_walkback_checks;
#[path = "bevel/surface_walkback_fold_gate_tests.rs"]
mod surface_walkback_fold_gate_tests;
#[path = "bevel/surface_walkback_guard_tests.rs"]
mod surface_walkback_guard_tests;
#[path = "bevel/swallowed_side_tests.rs"]
mod swallowed_side_tests;
#[path = "bevel/sweep_mesh_measures.rs"]
mod sweep_mesh_measures;
#[path = "bevel/tapered_barrel_test_meshes.rs"]
mod tapered_barrel_test_meshes;
#[path = "bevel/tapered_ledge_test_meshes.rs"]
mod tapered_ledge_test_meshes;
#[path = "bevel/tessellated_cylinder.rs"]
mod tessellated_cylinder;
#[path = "bevel/thread_cpu_time.rs"]
mod thread_cpu_time;
#[path = "bevel/top_and_round_corner_room_tests.rs"]
mod top_and_round_corner_room_tests;
#[path = "bevel/top_center_face_tests.rs"]
mod top_center_face_tests;
#[path = "bevel/top_center_nose_control_measure_tests.rs"]
mod top_center_nose_control_measure_tests;
#[path = "bevel/top_center_nose_run_out_measure_tests.rs"]
mod top_center_nose_run_out_measure_tests;
#[path = "bevel/top_center_nose_run_out_tests.rs"]
mod top_center_nose_run_out_tests;
#[path = "bevel/topology_only_probe_tests.rs"]
mod topology_only_probe_tests;
#[path = "bevel/topology_test_meshes.rs"]
mod topology_test_meshes;
#[path = "bevel/trebled_end_geometry.rs"]
mod trebled_end_geometry;
#[path = "bevel/trebled_end_seam_tests.rs"]
mod trebled_end_seam_tests;
#[path = "bevel/two_hole_bridge_fixture.rs"]
mod two_hole_bridge_fixture;
#[path = "bevel/two_mouths_in_one_facet_mesh.rs"]
mod two_mouths_in_one_facet_mesh;
#[path = "bevel/union_of_balls_field_tests.rs"]
mod union_of_balls_field_tests;
#[path = "bevel/unstoppable_build_tests.rs"]
mod unstoppable_build_tests;
#[path = "bevel/usable_room_tests.rs"]
mod usable_room_tests;
#[path = "bevel/virtual_rail_tests.rs"]
mod virtual_rail_tests;
#[path = "bevel/waisted_side_tests.rs"]
mod waisted_side_tests;
#[path = "bevel/wall_consumed_to_its_rim_tests.rs"]
mod wall_consumed_to_its_rim_tests;
#[path = "bevel/warped_wall_test_meshes.rs"]
mod warped_wall_test_meshes;
#[path = "bevel/work_budget_tests.rs"]
mod work_budget_tests;
