//! The inside corner of one chain's own SHARP JOINT, made a mesh vertex before the refinement
//! runs: on a face holding both of the joint's legs, the point standing the joint's setback from
//! each leg - where the level line turns. Ports `Bevel/LevelLine/SharpJointCorners.cs` (a C#
//! static class, so a module of functions); the insertion and the corner itself land with their
//! method packages.
//!
//! Without it the cut meets that corner only when the face's own edges pass through it: on the
//! knife-edge prism the cut chorded a corner off, the rail had no vertex standing for the joint's
//! whole parameter jump, and the strip fanned the back round's rows from the wrong end of the
//! chord.

use vector_math::{cs_math, Vector3};

use super::LevelLineMesh;

/// The noise allowance a corner is read against at `at`, on a face of size `scale`.
pub fn off_the_line(at: Vector3, scale: f64) -> f64 {
    1e-6 * cs_math::max_f64(cs_math::max_f64(1.0, scale), at.length())
}

/// True when `triangle` lies in the plane of the joint's two legs (`legs_plane`, a unit normal)
/// and holds `point` strictly inside, clear of every edge by the noise allowance.
pub fn holds(mesh: &LevelLineMesh, triangle: i32, point: Vector3, legs_plane: Vector3) -> bool {
    let t = mesh.triangles()[triangle as usize];
    let a = mesh.position(t.a);
    let b = mesh.position(t.b);
    let c = mesh.position(t.c);
    let mut normal = (b - a).cross(c - a);
    let area = normal.length();
    if area <= 0.0 {
        return false;
    }

    normal /= area;

    // A face the two legs lie in: the corner is the level line's turn ON that face. Where the
    // joint's legs leave the face (Around Center's knife tips) the line's turn is not a plane
    // offset corner, and inserted there it built one triangle count at the origin and another
    // placed through floats
    // (ChainFieldMitreTests.AroundCenterRotatedAndMovedThroughFloatsBuildsTheSameFill).
    if 1.0 - normal.dot(legs_plane).abs() > 1e-6 {
        return false;
    }

    let scale = cs_math::max_f64(
        (b - a).length(),
        cs_math::max_f64((c - b).length(), (a - c).length()),
    );
    if (point - a).dot(normal).abs() > off_the_line(point, scale) {
        return false;
    }

    // Strictly inside, clear of every edge by the noise allowance: a corner ON an edge (Around
    // Center's knife joints at the origin) is already met by the cut, which splits that edge where
    // the line crosses it, and inserted there by this rule as well it split the edge at the origin
    // but a triangle once the part was placed through floats - two triangle counts for one part.
    let clear = off_the_line(point, scale);
    let side = |p: Vector3, q: Vector3| (q - p).cross(point - p).dot(normal) / (q - p).length();
    side(a, b) > clear && side(b, c) > clear && side(c, a) > clear
}
