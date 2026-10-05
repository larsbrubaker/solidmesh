//! The single-precision grid parts are stored on: how far apart two neighbouring float values are
//! at a magnitude. A stored vertex stands anywhere within one step of where its shape puts it.
//! Ports `Bevel/FloatGrid.cs`.

use vector_math::{cs_math, Vector3};

/// The float step at a magnitude; zero at zero. C# `MathF.BitIncrement(largest) - largest`, the
/// subtraction done in single precision as C# does it, then widened.
pub fn step_at(magnitude: f32) -> f64 {
    let largest = magnitude.abs();
    if largest == 0.0 {
        0.0
    } else {
        f64::from(largest.next_up() - largest)
    }
}

/// The float step at a point's largest coordinate.
pub fn step_at_point(point: Vector3) -> f64 {
    // C# `Math.Max`, which propagates NaN; the `(float)` narrowing is `as f32` (round to nearest).
    step_at(cs_math::max_f64(
        point.x.abs(),
        cs_math::max_f64(point.y.abs(), point.z.abs()),
    ) as f32)
}
