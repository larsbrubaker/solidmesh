//! A mixed corner's RUN-OUT: the blend carried on past the bead's own turn until the ball's
//! touch row dies into the sharp edge the part keeps (`Bevel/Surface/SurfaceMixedCornerRunOut.cs`,
//! a static class: free functions here). Its constant and the one-place test land here first; the
//! run-out's construction lands with later packages.

use vector_math::Vector3;

/// How far past the bead's own turn the run-out's touch row clears the wall on a flat wall:
/// sixty degrees, `acos(1/2)`, whatever the wedge.
pub fn clears_the_wall() -> f64 {
    0.5f64.acos()
}

/// Whether two columns stand at one place: their trim points, or their rail points, within
/// `near_enough`.
pub fn is_one_place_with(
    trim: Vector3,
    rail: Vector3,
    other_trim: Vector3,
    other_rail: Vector3,
    near_enough: f64,
) -> bool {
    (other_trim - trim).length() <= near_enough || (other_rail - rail).length() <= near_enough
}
