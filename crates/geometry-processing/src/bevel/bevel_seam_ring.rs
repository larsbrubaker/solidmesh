//! The equal-angle step along the great circle between two unit vectors ([`slerp`]): how a corner
//! patch samples the ball between two face normals, so the arc a corner and the strip ending on it
//! share is computed by one route (the surface corner's rows, `surface_corner_patch_rows`).

use vector_math::Vector3;

/// The point `fraction` of the way along the great circle from one unit vector to another - equal
/// angles, which is how a corner patch samples the ball.
pub fn slerp(from: Vector3, to: Vector3, fraction: f64) -> Vector3 {
    let angle = from.dot(to).clamp(-1.0, 1.0).acos();

    if angle < 1e-9 {
        return from;
    }

    ((from * ((1.0 - fraction) * angle).sin()) + (to * (fraction * angle).sin())) / angle.sin()
}
