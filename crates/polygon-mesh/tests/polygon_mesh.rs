//! Ported agg-sharp `Tests/Agg.Tests/Agg.PolygonMesh` test classes, one file each (CLAUDE.md
//! net 1). Only the cases that exercise code ported so far are here; the rest are listed in
//! `docs/PORTING_PLAN.md` with the slice that brings them.

#[path = "polygon_mesh/face_normal_tests.rs"]
mod face_normal_tests;
#[path = "polygon_mesh/mesh_copy_independence_tests.rs"]
mod mesh_copy_independence_tests;
#[path = "polygon_mesh/mesh_edge_graph_tests.rs"]
mod mesh_edge_graph_tests;
#[path = "polygon_mesh/mesh_measure_tests.rs"]
mod mesh_measure_tests;
#[path = "polygon_mesh/mesh_tests.rs"]
mod mesh_tests;
#[path = "polygon_mesh/rust_only_stl_tests.rs"]
mod rust_only_stl_tests;
#[path = "polygon_mesh/stl_parse_yield_tests.rs"]
mod stl_parse_yield_tests;
