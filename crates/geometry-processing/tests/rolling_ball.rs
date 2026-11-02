//! The rolling-ball fillet's geometry (`geometry_processing::bevel::rolling_ball`): one file per
//! subject, with the meshes they share in `rolling_ball/surfaces.rs`. Each test holds the solve to
//! an answer worked out by hand - a corner of flat faces, a cylinder or sphere meeting a plate, a
//! face that runs out - so the module is checked against geometry, not against itself.

#[path = "rolling_ball/ball_pierce_room_tests.rs"]
mod ball_pierce_room_tests;
#[path = "rolling_ball/ball_station_tests.rs"]
mod ball_station_tests;
#[path = "rolling_ball/band_route_tests.rs"]
mod band_route_tests;
#[path = "rolling_ball/band_setback_tests.rs"]
mod band_setback_tests;
#[path = "rolling_ball/carried_regions_tests.rs"]
mod carried_regions_tests;
#[path = "rolling_ball/contact_tracking_tests.rs"]
mod contact_tracking_tests;
#[path = "rolling_ball/corner_setback_tests.rs"]
mod corner_setback_tests;
#[path = "rolling_ball/end_face_plane_tests.rs"]
mod end_face_plane_tests;
#[path = "rolling_ball/end_face_rim_tests.rs"]
mod end_face_rim_tests;
#[path = "rolling_ball/fine_front_sides_tests.rs"]
mod fine_front_sides_tests;
#[path = "rolling_ball/free_end_room_tests.rs"]
mod free_end_room_tests;
#[path = "rolling_ball/free_end_taper_tests.rs"]
mod free_end_taper_tests;
#[path = "rolling_ball/free_end_tests.rs"]
mod free_end_tests;
#[path = "rolling_ball/growing_ridge_room_tests.rs"]
mod growing_ridge_room_tests;
#[path = "rolling_ball/in_turn_side_regions_tests.rs"]
mod in_turn_side_regions_tests;
#[path = "rolling_ball/inside_turn_tests.rs"]
mod inside_turn_tests;
#[path = "rolling_ball/own_side_reach_tests.rs"]
mod own_side_reach_tests;
#[path = "rolling_ball/round_segment_tests.rs"]
mod round_segment_tests;
#[path = "rolling_ball/rounded_on_past_the_end_tests.rs"]
mod rounded_on_past_the_end_tests;
#[path = "rolling_ball/run_out_tests.rs"]
mod run_out_tests;
#[path = "rolling_ball/side_regions_tests.rs"]
mod side_regions_tests;
#[path = "rolling_ball/side_surface_tests.rs"]
mod side_surface_tests;
#[path = "rolling_ball/size_past_the_room_tests.rs"]
mod size_past_the_room_tests;
#[path = "rolling_ball/slanted_end_cap_tests.rs"]
mod slanted_end_cap_tests;
#[path = "rolling_ball/surfaces.rs"]
mod surfaces;
#[path = "rolling_ball/walk_tests.rs"]
mod walk_tests;
