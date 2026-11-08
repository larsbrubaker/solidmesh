//! **solidmesh-math**: deterministic `f64` geometry math in pure Rust: vectors, matrices,
//! quaternions, planes, rays, bounds, a BVH, colours and rectangles, under the
//! [solidmesh](https://docs.rs/solidmesh) mesh kernel. It builds for `wasm32-unknown-unknown` and
//! gives the same bits on every platform.
//!
//! The value types are [`Vector2`], [`Vector3`] and [`Vector4`] (`f64`; [`Vector2Float`] and
//! [`Vector3Float`] in `f32`), [`Matrix4X4`], [`Quaternion`], [`AxisAlignedBoundingBox`],
//! [`Plane`] and [`Ray`]; [`bvh`] is a bounding volume hierarchy for ray and box queries, and
//! [`Color`]/[`ColorF`] the colours a mesh face carries. [`Matrix4X4`] is row-major with row
//! vectors: a point is transformed as `p * M`, so `A * B` applies `A` first.
//!
//! ```
//! use solidmesh_math::{Matrix4X4, Vector3};
//!
//! // Scale by 2, then move 10 along x: the matrices compose left to right.
//! let m = Matrix4X4::create_scale_uniform(2.0) * Matrix4X4::create_translation(10.0, 0.0, 0.0);
//! let p = Vector3::new(1.0, 2.0, 3.0).transform(m);
//! assert_eq!(p, Vector3::new(12.0, 4.0, 6.0));
//! assert_eq!((p - Vector3::new(12.0, 0.0, 0.0)).length(), 52f64.sqrt());
//! ```
//!
//! solidmesh-math is one of four crates: this one,
//! [solidmesh-clipper](https://docs.rs/solidmesh-clipper) (integer polygon clipping),
//! [solidmesh-mesh](https://docs.rs/solidmesh-mesh) (triangle meshes, booleans, repair) and
//! [solidmesh](https://docs.rs/solidmesh) (fillets and chamfers), which re-exports the other
//! three; depend on `solidmesh` alone to get them all.
//!
//! # How the crate is laid out
//!
//! The value types and their helpers: `math_helper`, `Vector2`/`Vector3`/`Vector4` (f64),
//! `Vector2Float`/`Vector3Float` (f32), the FNV-1a `long_hash` behind every
//! `get_long_hash_code`, and the number formatting and parsing the vectors' text form needs
//! (`number_format`, `ieee_math`). `Matrix4X4` (row-major, row vectors: `p * M`), `Quaternion`,
//! `AxisAlignedBoundingBox`, `Plane`/`PlaneFloat`, `Ray`, `Frustum`, the vectors' matrix and
//! quaternion transforms (`vector_transforms`), `std::hash::Hash` for the value types
//! (`float_hash`) and their serde JSON in the shape saved files hold (`serde_impls` plus
//! derives). `WorldView` (camera and projection), `TrackBallController`, `easing`, the BVH
//! (`bvh`: `BvhTree`, its two builders, Morton codes, `RayHitInfo`, with
//! `Ray::get_closest_intersection`), the `parallel` loop wrapper (sequential on every host,
//! since wasm32 has no threads) and the fixed-order introsort (`fixed_tie_sort`) the builders need so
//! ties always land in the same place. `seeded_random` is the seeded generator shared by the
//! crates above that must draw the same numbers for the same seed. There is no smallest
//! enclosing circle here: the known implementation is LGPL-3 (Project Nayuki).
//! The 2D value types the mesh kernel and the 2D paths share live here too: the colours
//! (`color`: the 8-bit `Color`, the float `ColorF`, the `ColorType` trait, colour names and
//! `Color`'s JSON shape) and the axis-aligned rectangles (`RectangleDouble`, `RectangleInt`).
//! MatterCAD's 2D path crate (agg-path) re-exports them under the same paths
//! (`agg_path::Color`, `agg_path::color::...`, `agg_path::rectangle_double::...`), so code above
//! it names them either way.
//! The job progress channel is here for the same reason: `progress` (`ProgressReporter`, its
//! throttled UI yield and `YIELD_THROTTLE_MS`, on web-time's clock so the browser build can read
//! it), which MatterCAD's agg-path re-exports as `agg_path::progress_reporter`. A mesh face's
//! picture is a `texture::TexturePixels` (its size and bytes for the mesh hash, shared by `Arc`
//! identity), which the caller's image type implements (MatterCAD's is agg-path's byte image).
//!
//! Conventions: constructors and static helpers are associated functions, read-only values are
//! methods (`length()`), in-place and by-value variants that compute the same thing are one
//! function, multiple results come back as tuples, and fields are lowercase `x/y/z/w`.

mod axis_aligned_bounding_box;
pub mod bvh;
pub mod color;
pub mod easing;
pub mod fixed_tie_sort;
pub mod float_hash;
mod frustum;
pub mod ieee_math;
pub mod long_hash;
pub mod math_helper;
mod matrix4x4;
mod matrix4x4_create;
pub mod number_format;
pub mod parallel;
mod plane;
mod plane_float;
pub mod progress;
mod quaternion;
mod ray;
pub mod rectangle_double;
pub mod rectangle_int;
pub mod seeded_random;
mod serde_impls;
pub mod texture;
mod track_ball_controller;
mod vector2;
pub mod vector2_ex;
mod vector2_float;
mod vector3;
mod vector3_float;
mod vector4;
mod vector_transforms;
mod world_view;

pub use axis_aligned_bounding_box::AxisAlignedBoundingBox;
pub use color::{Color, ColorF, ColorType};
pub use frustum::{Frustum, FrustumIntersection};
pub use math_helper::MathError;
pub use matrix4x4::Matrix4X4;
pub use plane::Plane;
pub use plane_float::PlaneFloat;
pub use quaternion::Quaternion;
pub use ray::{IntersectionType, Ray, RayBoundsHit, Sign};
pub use rectangle_double::{OutCode, RectangleDouble};
pub use rectangle_int::RectangleInt;
pub use texture::TexturePixels;
pub use track_ball_controller::{TrackBallController, TrackBallError, TrackBallTransformType};
pub use vector2::Vector2;
pub use vector2_float::Vector2Float;
pub use vector3::{to_float_array, Vector3};
pub use vector3_float::{to_float_array_f32, Vector3Float};
pub use vector4::Vector4;
pub use vector_transforms::{transform_array, transform_array_by_quaternion, transform_array_f32};
pub use world_view::WorldView;

/// The FNV-1a 64-bit offset basis, the usual starting `hash` of every `get_long_hash_code`.
pub const LONG_HASH_SEED: u64 = 14695981039346656037;
