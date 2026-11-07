// Tests of the Clipper 6 port and the integer polygon helpers on it, one file per test class.

#[path = "clipper/clean_polygon_tests.rs"]
mod clean_polygon_tests;
#[path = "clipper/clipper6_tests.rs"]
mod clipper6_tests;
#[path = "clipper/clipper_library_tests.rs"]
mod clipper_library_tests;
#[path = "clipper/curve_tolerance_tests.rs"]
mod curve_tolerance_tests;
#[path = "clipper/invariant_culture_tests.rs"]
mod invariant_culture_tests;
