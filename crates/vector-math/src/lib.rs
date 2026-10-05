//! vector-math: a port of agg-sharp's `VectorMath` (`R/Submodules/agg-sharp/VectorMath`).
//!
//! Phase 1a: the vector value types and their helpers: `MathHelper` (`math_helper`),
//! `Vector2`/`Vector3`/`Vector4` (f64), `Vector2Float`/`Vector3Float` (f32), the FNV-1a
//! `LongHash` used by every `GetLongHashCode`, and the C#-compatible number formatting/parsing
//! the vectors' `ToString`/`Parse` need (`number_format`, `cs_math`).
//! Phase 1b: `Matrix4X4` (row-major, row vectors: `p * M`), `Quaternion`,
//! `AxisAlignedBoundingBox`, `Plane`/`PlaneFloat`, `Ray`, `Frustum`, the vectors' matrix and
//! quaternion transforms (`vector_transforms`), `std::hash::Hash` for the value types
//! (`float_hash`) and their Newtonsoft-shaped serde JSON (`serde_impls` plus derives).
//! Phase 1c: `WorldView` (camera and projection), `TrackBallController`, `easing`,
//! `estimator`, the BVH (`bvh`: `BvhTree`, its two builders, Morton codes, `RayHitInfo`, with
//! `Ray::get_closest_intersection`), the `parallel` loop wrapper (sequential on every host,
//! divergence 30) and .NET's introsort (`cs_sort`) the builders need to order ties as the C#
//! does. `dotnet_random` is .NET's seeded `Random`, shared by the crates above that must draw
//! the numbers the C# draws. `SmallestEnclosingCircle.cs` is not ported: it is LGPL-3 (Project
//! Nayuki).
//! `RectangleDouble`/`RectangleInt` are not in VectorMath (agg-sharp `agg/Primitives`).
//!
//! Conventions: C# static methods are associated functions, C# properties are methods
//! (`Length` → `length()`), C# `ref`/`out` overloads fold into the value overload when they
//! compute the same thing, multiple `out` results come back as tuples, and fields are lowercase
//! `x/y/z/w`.

mod axis_aligned_bounding_box;
pub mod bvh;
pub mod cs_math;
pub mod cs_sort;
pub mod dotnet_random;
pub mod easing;
pub mod estimator;
pub mod float_hash;
mod frustum;
pub mod long_hash;
pub mod math_helper;
mod matrix4x4;
mod matrix4x4_create;
pub mod number_format;
pub mod parallel;
mod plane;
mod plane_float;
mod quaternion;
mod ray;
mod serde_impls;
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
pub use frustum::{Frustum, FrustumIntersection};
pub use math_helper::MathError;
pub use matrix4x4::Matrix4X4;
pub use plane::Plane;
pub use plane_float::PlaneFloat;
pub use quaternion::Quaternion;
pub use ray::{IntersectionType, Ray, RayBoundsHit, Sign};
pub use track_ball_controller::{TrackBallController, TrackBallError, TrackBallTransformType};
pub use vector2::Vector2;
pub use vector2_float::Vector2Float;
pub use vector3::{to_float_array, Vector3};
pub use vector3_float::{to_float_array_f32, Vector3Float};
pub use vector4::Vector4;
pub use vector_transforms::{transform_array, transform_array_by_quaternion, transform_array_f32};
pub use world_view::WorldView;

/// The FNV-1a 64-bit offset basis, the default `hash` argument of every C#
/// `GetLongHashCode(ulong hash = 14695981039346656037)`.
pub const LONG_HASH_SEED: u64 = 14695981039346656037;
