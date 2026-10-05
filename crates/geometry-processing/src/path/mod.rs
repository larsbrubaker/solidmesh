//! `Path/`: the 2D side of the kernel. Rounding (beveling) the corners of a path
//! ([`path_bevel`], with the arc construction in `path_bevel_arcs`) and growing a seed segment of a
//! contour into its run of curved-but-close neighbours ([`path_segment_run`]).

pub mod path_bevel;
pub mod path_bevel_arcs;
pub mod path_segment_run;

pub use path_bevel::{Contour, PathBevelSizeMode};
pub use path_segment_run::{PathSegmentRunError, RunStopReason, SegmentRun};
