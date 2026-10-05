//! The surface plan's reading of one picked edge. Ports `Bevel/Surface/SurfacePlanEdgeReading.cs`:
//! so far its finiteness tests ([`is_finite`], [`is_finite_vector`]); the reading lands with later
//! packages.

use vector_math::Vector3;

/// True when the number is neither NaN nor infinite.
pub fn is_finite(value: f64) -> bool {
    !value.is_nan() && !value.is_infinite()
}

/// True when every component of the vector is finite (C#'s `IsFinite(Vector3)` overload).
pub fn is_finite_vector(value: Vector3) -> bool {
    is_finite(value.x) && is_finite(value.y) && is_finite(value.z)
}
