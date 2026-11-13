//! The ray-tracer test binary: triangle and shape hits, view-space CSG, the trace API, and the
//! BVH built in parallel against the sequential build, one file per test class.

#[path = "ray_tracer/boolean_tests.rs"]
mod boolean_tests;
#[path = "ray_tracer/bvh_flat_mesh_depth_tests.rs"]
mod bvh_flat_mesh_depth_tests;
#[path = "ray_tracer/flat_tree_precision_tests.rs"]
mod flat_tree_precision_tests;
#[path = "ray_tracer/flat_tree_shader_tests.rs"]
mod flat_tree_shader_tests;
#[path = "ray_tracer/flat_tree_support.rs"]
mod flat_tree_support;
#[path = "ray_tracer/flat_tree_tests.rs"]
mod flat_tree_tests;
#[path = "ray_tracer/floor_rest_agreement_tests.rs"]
mod floor_rest_agreement_tests;
#[path = "ray_tracer/glass_on_floor_opacity_tests.rs"]
mod glass_on_floor_opacity_tests;
#[path = "ray_tracer/glass_on_floor_tests.rs"]
mod glass_on_floor_tests;
#[path = "ray_tracer/glass_on_floor_tie_measure_tests.rs"]
mod glass_on_floor_tie_measure_tests;
#[path = "ray_tracer/glass_rest_search_tests.rs"]
mod glass_rest_search_tests;
#[path = "ray_tracer/glass_tray_support.rs"]
mod glass_tray_support;
#[path = "ray_tracer/gpu_glass_contact_grid_tests.rs"]
mod gpu_glass_contact_grid_tests;
#[path = "ray_tracer/gpu_glass_contact_pack_tests.rs"]
mod gpu_glass_contact_pack_tests;
#[path = "ray_tracer/gpu_view_tests.rs"]
mod gpu_view_tests;
#[path = "ray_tracer/minimal_triangle_tests.rs"]
mod minimal_triangle_tests;
#[path = "ray_tracer/parallel_sequential_equivalence_tests.rs"]
mod parallel_sequential_equivalence_tests;
#[path = "ray_tracer/part_opacity_tests.rs"]
mod part_opacity_tests;
#[path = "ray_tracer/path_camera_ray_tests.rs"]
mod path_camera_ray_tests;
#[path = "ray_tracer/path_shader_inlining_tests.rs"]
mod path_shader_inlining_tests;
#[path = "ray_tracer/path_shader_seam_tests.rs"]
mod path_shader_seam_tests;
#[path = "ray_tracer/path_shader_tests.rs"]
mod path_shader_tests;
#[path = "ray_tracer/path_tracer_glass_contact_measure_tests.rs"]
mod path_tracer_glass_contact_measure_tests;
#[path = "ray_tracer/path_tracer_glass_contact_search_tests.rs"]
mod path_tracer_glass_contact_search_tests;
#[path = "ray_tracer/path_tracer_glass_contact_tests.rs"]
mod path_tracer_glass_contact_tests;
#[path = "ray_tracer/path_tracer_glass_shadow_tests.rs"]
mod path_tracer_glass_shadow_tests;
#[path = "ray_tracer/path_tracer_golden_tests.rs"]
mod path_tracer_golden_tests;
#[path = "ray_tracer/path_tracer_measure_tests.rs"]
mod path_tracer_measure_tests;
#[path = "ray_tracer/path_tracer_partial_glass_tests.rs"]
mod path_tracer_partial_glass_tests;
#[path = "ray_tracer/path_tracer_render_tests.rs"]
mod path_tracer_render_tests;
#[path = "ray_tracer/path_tracer_scene_tests.rs"]
mod path_tracer_scene_tests;
#[path = "ray_tracer/path_tracer_speckle_measure_tests.rs"]
mod path_tracer_speckle_measure_tests;
#[path = "ray_tracer/path_tracer_sun_glass_frosted_tests.rs"]
mod path_tracer_sun_glass_frosted_tests;
#[path = "ray_tracer/path_tracer_sun_glass_partition_tests.rs"]
mod path_tracer_sun_glass_partition_tests;
#[path = "ray_tracer/path_tracer_sun_glass_rules_tests.rs"]
mod path_tracer_sun_glass_rules_tests;
#[path = "ray_tracer/path_tracer_sun_glass_sweep_tests.rs"]
mod path_tracer_sun_glass_sweep_tests;
#[path = "ray_tracer/path_tracer_sun_through_glass_tests.rs"]
mod path_tracer_sun_through_glass_tests;
#[path = "ray_tracer/path_tracer_support.rs"]
mod path_tracer_support;
#[path = "ray_tracer/path_tracer_touching_glass_tests.rs"]
mod path_tracer_touching_glass_tests;
#[path = "ray_tracer/path_tracer_transport_tests.rs"]
mod path_tracer_transport_tests;
#[path = "ray_tracer/polygon_trace_tests.rs"]
mod polygon_trace_tests;
#[path = "ray_tracer/ray_bundle_contract_tests.rs"]
mod ray_bundle_contract_tests;
#[path = "ray_tracer/ray_bundle_tests.rs"]
mod ray_bundle_tests;
#[path = "ray_tracer/rust_only_primary_ray_tests.rs"]
mod rust_only_primary_ray_tests;
#[path = "ray_tracer/rust_only_ray_tracer_tests.rs"]
mod rust_only_ray_tracer_tests;
#[path = "ray_tracer/rust_only_shape_tests.rs"]
mod rust_only_shape_tests;
#[path = "ray_tracer/rust_only_triangle_edge_tests.rs"]
mod rust_only_triangle_edge_tests;
#[path = "ray_tracer/scatter_shader_tests.rs"]
mod scatter_shader_tests;
#[path = "ray_tracer/scattering_energy_tests.rs"]
mod scattering_energy_tests;
#[path = "ray_tracer/scattering_support.rs"]
mod scattering_support;
#[path = "ray_tracer/scattering_tests.rs"]
mod scattering_tests;
#[path = "ray_tracer/see_through_coverage_end_tests.rs"]
mod see_through_coverage_end_tests;
#[path = "ray_tracer/see_through_coverage_tests.rs"]
mod see_through_coverage_tests;
#[path = "ray_tracer/see_through_glass_on_floor_tests.rs"]
mod see_through_glass_on_floor_tests;
#[path = "ray_tracer/see_through_on_floor_tests.rs"]
mod see_through_on_floor_tests;
#[path = "ray_tracer/sun_line_shader_tests.rs"]
mod sun_line_shader_tests;
#[path = "ray_tracer/sun_line_text_tests.rs"]
mod sun_line_text_tests;
#[path = "ray_tracer/sun_through_see_through_rest_tests.rs"]
mod sun_through_see_through_rest_tests;
#[path = "ray_tracer/sun_weights_tests.rs"]
mod sun_weights_tests;
#[path = "ray_tracer/sun_weights_text_tests.rs"]
mod sun_weights_text_tests;
#[path = "ray_tracer/surface_builder_tests.rs"]
mod surface_builder_tests;
#[path = "ray_tracer/surface_hit_continuation_tests.rs"]
mod surface_hit_continuation_tests;
#[path = "ray_tracer/surface_hit_placed_tests.rs"]
mod surface_hit_placed_tests;
#[path = "ray_tracer/surface_hit_support.rs"]
mod surface_hit_support;
#[path = "ray_tracer/surface_hit_tests.rs"]
mod surface_hit_tests;
#[path = "ray_tracer/trace_api_tests.rs"]
mod trace_api_tests;
