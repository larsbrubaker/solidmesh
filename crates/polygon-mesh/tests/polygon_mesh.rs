//! The polygon-mesh test binary: the mesh, its face colours and textures, measuring, splitting,
//! convex hulls, revolve and extrude, marching cubes, distance fields and STL, one file per test
//! class.

#[path = "polygon_mesh/clean_extrude_tests.rs"]
mod clean_extrude_tests;
#[path = "polygon_mesh/face_color_boolean_scenarios.rs"]
mod face_color_boolean_scenarios;
#[path = "polygon_mesh/face_color_tests.rs"]
mod face_color_tests;
#[path = "polygon_mesh/face_normal_tests.rs"]
mod face_normal_tests;
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
#[path = "polygon_mesh/mesh_sdf_grid_tests.rs"]
mod mesh_sdf_grid_tests;
#[path = "polygon_mesh/mesh_split_manifold_tests.rs"]
mod mesh_split_manifold_tests;
#[path = "polygon_mesh/mesh_tests.rs"]
mod mesh_tests;
#[path = "polygon_mesh/revolve_tests.rs"]
mod revolve_tests;
#[path = "polygon_mesh/room_scale_offset_tolerance_tests.rs"]
mod room_scale_offset_tolerance_tests;
#[path = "polygon_mesh/rust_only_extrude_tests.rs"]
mod rust_only_extrude_tests;
#[path = "polygon_mesh/rust_only_mesh_editing_tests.rs"]
mod rust_only_mesh_editing_tests;
#[path = "polygon_mesh/rust_only_platonic_solids_tests.rs"]
mod rust_only_platonic_solids_tests;
#[path = "polygon_mesh/rust_only_stl_tests.rs"]
mod rust_only_stl_tests;
#[path = "polygon_mesh/sdf_and_rounded_box_tests.rs"]
mod sdf_and_rounded_box_tests;
#[path = "polygon_mesh/stl_parse_yield_tests.rs"]
mod stl_parse_yield_tests;
#[path = "polygon_mesh/transformed_bounds_exact_tests.rs"]
mod transformed_bounds_exact_tests;
