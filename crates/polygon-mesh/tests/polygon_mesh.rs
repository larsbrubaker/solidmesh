//! The polygon-mesh test binary: the mesh, its face colours and textures, measuring, splitting,
//! convex hulls, marching cubes, distance fields and STL, one file per test class.

#[path = "polygon_mesh/closed_solid_repair_tests.rs"]
mod closed_solid_repair_tests;
#[path = "polygon_mesh/face_color_boolean_scenarios.rs"]
mod face_color_boolean_scenarios;
#[path = "polygon_mesh/face_color_tests.rs"]
mod face_color_tests;
#[path = "polygon_mesh/face_normal_tests.rs"]
mod face_normal_tests;
#[path = "polygon_mesh/face_texture_hash_tests.rs"]
mod face_texture_hash_tests;
#[path = "polygon_mesh/face_texture_tests.rs"]
mod face_texture_tests;
#[path = "polygon_mesh/grid_marching_cubes_tests.rs"]
mod grid_marching_cubes_tests;
#[path = "polygon_mesh/invariant_culture_tests.rs"]
mod invariant_culture_tests;
#[path = "polygon_mesh/mesh_convex_hull_tests.rs"]
mod mesh_convex_hull_tests;
#[path = "polygon_mesh/mesh_copy_independence_tests.rs"]
mod mesh_copy_independence_tests;
#[path = "polygon_mesh/mesh_edge_graph_steps_tests.rs"]
mod mesh_edge_graph_steps_tests;
#[path = "polygon_mesh/mesh_edge_graph_tests.rs"]
mod mesh_edge_graph_tests;
#[path = "polygon_mesh/mesh_measure_tests.rs"]
mod mesh_measure_tests;
#[path = "polygon_mesh/mesh_sdf_grid_async_tests.rs"]
mod mesh_sdf_grid_async_tests;
#[path = "polygon_mesh/mesh_sdf_grid_prepared_tests.rs"]
mod mesh_sdf_grid_prepared_tests;
#[path = "polygon_mesh/mesh_sdf_grid_tests.rs"]
mod mesh_sdf_grid_tests;
#[path = "polygon_mesh/mesh_split_manifold_tests.rs"]
mod mesh_split_manifold_tests;
#[path = "polygon_mesh/mesh_tests.rs"]
mod mesh_tests;
#[path = "polygon_mesh/outline_to_mesh_tests.rs"]
mod outline_to_mesh_tests;
#[path = "polygon_mesh/reduce_face_data_tests.rs"]
mod reduce_face_data_tests;
#[path = "polygon_mesh/reduce_seam_tests.rs"]
mod reduce_seam_tests;
#[path = "polygon_mesh/reduce_test_support.rs"]
mod reduce_test_support;
#[path = "polygon_mesh/repair_texture_caps_tests.rs"]
mod repair_texture_caps_tests;
#[path = "polygon_mesh/rust_only_mesh_editing_tests.rs"]
mod rust_only_mesh_editing_tests;
#[path = "polygon_mesh/rust_only_platonic_solids_tests.rs"]
mod rust_only_platonic_solids_tests;
#[path = "polygon_mesh/rust_only_stl_tests.rs"]
mod rust_only_stl_tests;
#[path = "polygon_mesh/sdf_and_rounded_box_tests.rs"]
mod sdf_and_rounded_box_tests;
#[path = "polygon_mesh/solid_checks.rs"]
mod solid_checks;
#[path = "polygon_mesh/split_volumes_tests.rs"]
mod split_volumes_tests;
#[path = "polygon_mesh/stl_parse_yield_tests.rs"]
mod stl_parse_yield_tests;
#[path = "polygon_mesh/test_picture.rs"]
mod test_picture;
#[path = "polygon_mesh/transformed_bounds_exact_tests.rs"]
mod transformed_bounds_exact_tests;
#[path = "polygon_mesh/uniform_face_color_tests.rs"]
mod uniform_face_color_tests;
