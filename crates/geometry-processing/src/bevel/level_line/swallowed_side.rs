//! A side the band swallows whole. Ports `Bevel/LevelLine/SwallowedSide.cs` (a C# static class):
//! so far its mesh readings - whether a triangle touches a point ([`touches`]), a side's vertices
//! ([`side_vertices`]), the direction away from a segment on a side ([`away_on_side`]) and the
//! planes' tolerance ([`tolerance`]); the swallow tests land with later packages.

use std::collections::HashSet;

use vector_math::Vector3;

use super::{LevelLineMesh, LevelLineTriangle};

/// True when a corner of `tri` stands within the mesh's same-place backstop of `point`.
pub fn touches(mesh: &LevelLineMesh, tri: LevelLineTriangle, point: Vector3) -> bool {
    (mesh.position(tri.a) - point).length() <= mesh.same_place()
        || (mesh.position(tri.b) - point).length() <= mesh.same_place()
        || (mesh.position(tri.c) - point).length() <= mesh.same_place()
}

/// The vertices of a side's live triangles.
pub fn side_vertices(mesh: &LevelLineMesh, side: i32) -> HashSet<i32> {
    let mut vertices = HashSet::new();
    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        if !mesh.is_removed(t) && tri.tag == side {
            vertices.insert(tri.a);
            vertices.insert(tri.b);
            vertices.insert(tri.c);
        }
    }

    vertices
}

/// The unit direction, square to segment `a b`, from the segment into the first live triangle of
/// `side` with an edge along it (within `tolerance`) and its third corner off it; `None` where no
/// such triangle stands.
pub fn away_on_side(
    mesh: &LevelLineMesh,
    a: Vector3,
    b: Vector3,
    side: i32,
    tolerance: f64,
) -> Option<Vector3> {
    let direction = (b - a).get_normal();
    let on_segment = |q: Vector3| {
        let along = (q - a).dot(direction);
        along >= -tolerance
            && along <= (b - a).length() + tolerance
            && ((q - a) - (direction * along)).length() <= tolerance
    };

    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        if mesh.is_removed(t) || tri.tag != side {
            continue;
        }

        for corner in 0..3 {
            let p = mesh.position(tri.corner(corner));
            let q = mesh.position(tri.corner((corner + 1) % 3));
            let third = mesh.position(tri.corner((corner + 2) % 3));
            if (p - q).length() > tolerance && on_segment(p) && on_segment(q) && !on_segment(third)
            {
                let off = third - a;
                return Some((off - (direction * off.dot(direction))).get_normal());
            }
        }
    }

    None
}

/// C# `SwallowedSide.OnPlanePerLength`: the planes' tolerance per unit of length: the fixture's
/// walls are float (a few 1e-7 off).
pub const ON_PLANE_PER_LENGTH: f64 = 1e-5;

/// C# `SwallowedSide.Tolerance`: the planes' tolerance for this chain, scaled by its largest
/// setback (at least 1).
pub fn tolerance(chain: &super::ChainField) -> f64 {
    ON_PLANE_PER_LENGTH * vector_math::cs_math::max_f64(1.0, chain.max_setback)
}
