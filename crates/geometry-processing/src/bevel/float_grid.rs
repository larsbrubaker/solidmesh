//! The single-precision grid parts are stored on: how far apart two neighbouring float values are
//! at a magnitude. A stored vertex stands anywhere within one step of where its shape puts it.

use solidmesh_math::{ieee_math, Vector3};

/// The float step at a magnitude; zero at zero. The next float up less the magnitude, the
/// subtraction done in single precision, then widened.
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
    // `max_f64` propagates NaN; the narrowing is `as f32` (round to nearest).
    step_at(ieee_math::max_f64(
        point.x.abs(),
        ieee_math::max_f64(point.y.abs(), point.z.abs()),
    ) as f32)
}
