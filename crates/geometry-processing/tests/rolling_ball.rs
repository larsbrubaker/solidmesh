//! The rolling-ball fillet's geometry (`geometry_processing::bevel::rolling_ball`): one file per
//! subject, with the meshes they share in `rolling_ball/surfaces.rs`. Each test holds the solve to
//! an answer worked out by hand - a corner of flat faces, a cylinder or sphere meeting a plate, a
//! face that runs out - so the module is checked against geometry, not against itself.

#[path = "rolling_ball/ball_station_tests.rs"]
mod ball_station_tests;
#[path = "rolling_ball/band_route_tests.rs"]
mod band_route_tests;
#[path = "rolling_ball/band_setback_tests.rs"]
mod band_setback_tests;
#[path = "rolling_ball/contact_tracking_tests.rs"]
mod contact_tracking_tests;
#[path = "rolling_ball/run_out_tests.rs"]
mod run_out_tests;
#[path = "rolling_ball/side_surface_tests.rs"]
mod side_surface_tests;
#[path = "rolling_ball/surfaces.rs"]
mod surfaces;
#[path = "rolling_ball/walk_tests.rs"]
mod walk_tests;
