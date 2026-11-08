//! `Path/`: the 2D side of the kernel. Rounding (beveling) the corners of a path
//! ([`path_bevel`], with the arc construction in `path_bevel_arcs`) and growing a seed segment of a
//! contour into its run of curved-but-close neighbours ([`path_segment_run`]).
//!
//! Angles and arc points are worked out with the pure-Rust `libm` crate, never std's `sin`, `cos`,
//! `tan`, `acos` or `atan2`, which call each platform's own maths library and differ in the last
//! bits: with `libm` a design gives the same curves on Windows, Linux, macOS and the browser
//! (`tests/platform_math_tests.rs` refuses a std call here and in `topology`).

pub mod path_bevel;
pub mod path_bevel_arcs;
pub mod path_segment_run;

pub use path_bevel::{Contour, PathBevelSizeMode};
pub use path_segment_run::{PathSegmentRunError, RunStopReason, SegmentRun};
