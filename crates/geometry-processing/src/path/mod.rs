//! The 2D side of the kernel: rounding (beveling) the corners of flattened contours
//! ([`bevel`], [`bevel_contour`], [`Contour`], [`PathBevelSizeMode`]; [`path_bevel`] holds the
//! method, with the arc construction in `path_bevel_arcs`), the corners a bevel would round
//! ([`PathCornerSet`] of [`PathCorner`]s) and a picked corner stored so it survives a rebuild
//! ([`BevelCornerGroup`], matched by [`corner_group_from`] and [`resolve_corner_groups`]), and
//! growing a seed segment of a contour into its run of curved-but-close neighbours
//! ([`path_segment_run`]).
//!
//! ```
//! use solidmesh::math::Vector2;
//! use solidmesh::path::{bevel, Contour, PathBevelSizeMode};
//!
//! // A 10 by 10 square, every corner rounded to a 2 mm radius in 4 segments.
//! let square = Contour {
//!     points: vec![
//!         Vector2::new(0.0, 0.0),
//!         Vector2::new(10.0, 0.0),
//!         Vector2::new(10.0, 10.0),
//!         Vector2::new(0.0, 10.0),
//!     ],
//!     is_closed: true,
//! };
//! let rounded = bevel(&[square], 2.0, 4, 20.0, None, PathBevelSizeMode::Radius);
//! // Each corner's one point becomes the five points of its arc.
//! assert_eq!(rounded[0].points.len(), 4 * 5);
//! ```
//!
//! Angles and arc points are worked out with the pure-Rust `libm` crate, never std's `sin`, `cos`,
//! `tan`, `acos` or `atan2`, which call each platform's own maths library and differ in the last
//! bits: with `libm` a design gives the same curves on Windows, Linux, macOS and the browser
//! (`tests/platform_math_tests.rs` refuses a std call here and in `topology`).

pub mod path_bevel;
#[doc(hidden)]
pub mod path_bevel_arcs;
pub mod path_segment_run;

pub use crate::topology::bevel_corner_group_resolver::{
    create_from_corner as corner_group_from, resolve_all as resolve_corner_groups,
    BevelCornerGroupResolution,
};
pub use crate::topology::{BevelCornerGroup, PathCorner, PathCornerSet};
pub use path_bevel::{bevel, bevel_contour, Contour, PathBevelSizeMode, VertexSelection};
pub use path_segment_run::{PathSegmentRunError, RunStopReason, SegmentRun};
