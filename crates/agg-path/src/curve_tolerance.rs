//! Curve flattening tolerance scaled to a shape's size. Ports agg-sharp
//! `agg/VertexSource/CurveTolerance.cs`: tolerances tuned for desk-sized parts (up to 300 units)
//! stay as they are; larger shapes scale them up so they are not over-tessellated.

/// The largest size the base tolerances are tuned for.
pub const DESK_SIZE: f64 = 300.0;

/// How much to scale a tolerance for a shape `size` across (1 up to desk size, NaN/infinity).
pub fn scale_for(size: f64) -> f64 {
    if size.is_finite() && size > DESK_SIZE {
        size / DESK_SIZE
    } else {
        1.0
    }
}

pub fn for_size(base_tolerance: f64, size: f64) -> f64 {
    base_tolerance * scale_for(size)
}
