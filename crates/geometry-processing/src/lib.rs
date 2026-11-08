//! **solidmesh**: fillets and chamfers on triangle meshes, in pure Rust, over a mesh kernel that
//! keeps closed solids closed. It builds for `wasm32-unknown-unknown`, contains no C or C++, and
//! gives the same mesh on every platform.
//!
//! This is the one crate to depend on. It re-exports the three crates beneath it:
//!
//! - [`math`] ([`solidmesh-math`](https://docs.rs/solidmesh-math)): `f64` vectors, matrices,
//!   quaternions, planes, rays, bounds, a BVH and colours;
//! - [`clipper`] ([`solidmesh-clipper`](https://docs.rs/solidmesh-clipper)): integer polygon
//!   booleans, offsets and simplification;
//! - [`mesh`] ([`solidmesh-mesh`](https://docs.rs/solidmesh-mesh)): the triangle [`mesh::Mesh`],
//!   booleans, repair, signed distance fields, convex hulls, Minkowski sums, extrusion and
//!   revolution of 2D outlines, slicing and STL.
//!
//! # Fillets and chamfers
//!
//! A part's fillet or chamfer is one call, [`bevel_part`]: a [`BevelPartRequest`] (the mesh, the
//! picks, one size for all of them, round or flat, and the constructions it may take,
//! [`BevelRouting`]) in, a [`BevelPartOutcome`] out. The outcome carries the beveled mesh (or
//! none), one [`BevelGroupResult`] per pick and the part's advisories, all as codes
//! ([`BevelAdvisoryCode`] with its [`BevelAdvisoryArg`]s) for the caller to word in its own
//! language. A pick that does not fit is refused, never an error, and where a smaller size builds
//! the refusal carries the largest size that does ([`the_shown_size`] is how to show it). Only a
//! cancel or a build that could not run at all is a [`BevelPartError`].
//!
//! A pick is a [`BevelEdgeGroup`]: a geometric signature of what was picked (an anchor edge's
//! midpoint, direction and face planes, or a corner's point, by [`BevelSelectionKind`]) rather
//! than mesh indices, so it survives the part being rebuilt. [`pick`] turns a click into one:
//! [`BevelPickData`] prepares a part for picking, [`pick::pick_feature_edge`] finds the
//! [`FeatureEdge`] under the cursor, and [`pick::interpretations_of_edge`] reads it as the edge
//! alone, its tangent chain or a face rim.
//!
//! ```
//! use std::sync::Arc;
//!
//! use solidmesh::mesh::solids::platonic_solids::create_cube;
//! use solidmesh::mesh::CancelToken;
//! use solidmesh::{
//!     bevel_part, pick, BevelPartRequest, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
//! };
//!
//! // A 20 mm cube and its sharp edges, read at a 20 degree bend threshold.
//! let cube = Arc::new(create_cube(20.0, 20.0, 20.0));
//! let edges = FeatureEdgeSet::create_with_angle(&cube, 20.0);
//!
//! // Pick the first edge on its own, and round it with a 3 mm fillet of 8 segments.
//! let picked = pick::group_from_edge(
//!     &edges,
//!     0,
//!     BevelSelectionKind::Edge,
//!     FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
//! )?;
//! let request = BevelPartRequest::new(Some(cube), vec![picked], 3.0, 8, true, 20.0);
//! let outcome = bevel_part(request, &CancelToken::new(), None)?;
//!
//! let row = outcome.group_results[0].as_ref().expect("a row for the pick");
//! assert!(!row.failed());
//! let rounded = outcome.mesh.expect("the rounded cube");
//! assert!(rounded.get_volume() < 8000.0);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # 2D contours
//!
//! [`path`] rounds or chamfers the corners of flattened 2D contours ([`path::bevel`]), sharing
//! the room on a short edge fairly between the corners at its ends; [`path::BevelCornerGroup`]
//! stores a picked path corner the way [`BevelEdgeGroup`] stores a mesh edge.
//!
//! # What these docs leave out
//!
//! The crate also has public modules hidden from these docs (`bevel`, `topology`, and items such
//! as `path::path_bevel_arcs`): the kernel's constructions, their test seams and the readings
//! MatterCAD, the application this kernel was written for, builds on. They are not part of the
//! semver promise: hidden items are for MatterCAD and may change in any release.
//!
//! # How the crate is laid out
//!
//! `topology` holds feature edges, chains, face groups, the region cache, edge picking, the bevel
//! edge/corner group records, the edge and corner group resolvers, path corners and the readings
//! of a pick or stored group; `path` the path bevel and segment runs; and `bevel` the whole
//! kernel: advisories, the level-line, section and surface builders, the section and run readings
//! they share, and a whole part's fillet or chamfer through its one entry point,
//! `bevel::part::bevel_part`. Nothing here may depend on a scene, a UI, an application or a 2D
//! path model, and `tests/kernel_boundary_tests.rs` reads the crate's `Cargo.toml` to guard that
//! split. `path` and the path corners work on plain contours, already flattened. The crate root
//! and [`pick`] re-export the documented surface from those modules.

pub use solidmesh_clipper as clipper;
pub use solidmesh_math as math;
pub use solidmesh_mesh as mesh;

#[doc(hidden)]
pub mod bevel;
pub mod path;
pub mod pick;
#[doc(hidden)]
pub mod topology;

pub use bevel::part::{
    bevel_part, the_shown_size, BevelPartError, BevelPartOutcome, BevelPartRequest, BevelPickMemo,
    BevelProgress, BevelProgressPhase, BevelProgressReport, BevelRouting, BevelSizeSearchRung,
};
pub use bevel::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelGroupResult, BevelPickData,
    BevelResultGroup,
};
pub use topology::{
    BevelEdgeGroup, BevelSelectionKind, ChainEdge, ChainEnd, ChainStopReason, EdgeConvexity,
    FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeChain, FeatureEdgeSet, TopologyError,
};
