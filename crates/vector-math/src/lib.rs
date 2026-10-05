//! vector-math: a port of agg-sharp's `VectorMath` (`R/Submodules/agg-sharp/VectorMath`).
//!
//! This step (Phase 1a) holds the vector value types and their helpers: `MathHelper`
//! (`math_helper`), `Vector2`/`Vector3`/`Vector4` (f64), `Vector2Float`/`Vector3Float` (f32),
//! the FNV-1a `LongHash` used by every `GetLongHashCode`, and the C#-compatible number
//! formatting/parsing the vectors' `ToString`/`Parse` need (`number_format`, `cs_math`).
//! Matrix4X4, Quaternion and everything that depends on them (vector `Transform*`) land in
//! Phase 1b.
//!
//! Conventions: C# static methods are associated functions, C# properties are methods
//! (`Length` → `length()`), C# `ref`/`out` overloads fold into the value overload when they
//! compute the same thing, and fields are lowercase `x/y/z/w`.

pub mod cs_math;
pub mod long_hash;
pub mod math_helper;
pub mod number_format;
mod vector2;
pub mod vector2_ex;
mod vector2_float;
mod vector3;
mod vector3_float;
mod vector4;

pub use math_helper::MathError;
pub use vector2::Vector2;
pub use vector2_float::Vector2Float;
pub use vector3::{to_float_array, Vector3};
pub use vector3_float::{to_float_array_f32, Vector3Float};
pub use vector4::Vector4;

/// The FNV-1a 64-bit offset basis, the default `hash` argument of every C#
/// `GetLongHashCode(ulong hash = 14695981039346656037)`.
pub const LONG_HASH_SEED: u64 = 14695981039346656037;
