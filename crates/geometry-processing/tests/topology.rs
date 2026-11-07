//! The topology test binary: the tests of `topology/`, one file per test class, with the fixture
//! builders they need as modules of their own.

#[path = "topology/bevel_edge_group_collinear_tests.rs"]
mod bevel_edge_group_collinear_tests;
#[path = "topology/bevel_group_resolution_advisory_tests.rs"]
mod bevel_group_resolution_advisory_tests;
#[path = "topology/edge_picking_tests.rs"]
mod edge_picking_tests;
#[path = "topology/face_group_tests.rs"]
mod face_group_tests;
#[path = "topology/face_pick_interpretation_tests.rs"]
mod face_pick_interpretation_tests;
#[path = "topology/feature_edge_chain_tests.rs"]
mod feature_edge_chain_tests;
#[path = "topology/feature_edge_collinear_run_tests.rs"]
mod feature_edge_collinear_run_tests;
#[path = "topology/feature_edge_detection_tests.rs"]
mod feature_edge_detection_tests;
#[path = "topology/rust_only_bevel_group_tests.rs"]
mod rust_only_bevel_group_tests;
#[path = "topology/split_crease_test_meshes.rs"]
mod split_crease_test_meshes;
// Shared with the bevel binary, which uses builders this one does not.
#[allow(dead_code)]
#[path = "bevel/path_solids.rs"]
mod path_solids;
// Shared with the bevel binary, which uses builders this one does not.
#[allow(dead_code)]
#[path = "bevel/topology_test_meshes.rs"]
mod topology_test_meshes;
