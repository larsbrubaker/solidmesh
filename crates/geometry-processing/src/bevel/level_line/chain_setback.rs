//! How far back from a chain edge a size reaches - the one place a bevel's size becomes a distance
//! in the level-line field, read from the angle between the two sides rather than from any
//! per-station construction.

use solidmesh_math::Vector3;

/// The setback a fillet of `radius` asks of an edge whose two sides carry outward normals
/// `n_left` and `n_right`: `radius * tan(phi / 2)`, phi being the angle between those normals. A
/// square (90 degree) edge asks for exactly the radius; a blunt edge asks less and a sharp one
/// more, which is what lets an arc of that radius meet both sides tangentially at the boundary.
pub fn for_fillet(radius: f64, n_left: Vector3, n_right: Vector3) -> f64 {
    // An explicit clamp, which passes NaN through so a broken normal stays visible.
    let cos_phi = n_left
        .get_normal()
        .dot(n_right.get_normal())
        .clamp(-1.0, 1.0);

    radius * (cos_phi.acos() / 2.0).tan()
}

/// The setback a chamfer asks for, which is the size itself.
pub fn for_chamfer(size: f64) -> f64 {
    size
}

/// Whether setbacks running from `lowest` to `highest` are ONE setback per unit of size - every
/// square-cornered rim, and every chamfer. The one place that rule is written: a reading that
/// scales a distance by a setback is exact exactly when this is true, and a room reading and a
/// pair reading must not disagree about which case they are in.
///
/// A few bits of room, relative to the setbacks themselves, because two ways of arriving at the
/// same angle do not give the same last bit. Setbacks that are not all POSITIVE are not uniform
/// either: a zero setback divides into no size at all, so a reading must take the careful branch
/// rather than trust one number.
pub fn is_uniform(lowest: f64, highest: f64) -> bool {
    lowest > 0.0 && highest - lowest <= 1e-12 * highest
}
