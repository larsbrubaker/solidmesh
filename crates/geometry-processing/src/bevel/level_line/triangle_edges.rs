//! The input triangle list read as EDGES: which triangles stand on an edge, the corners a
//! triangle traverses and in what direction. The plumbing every step of the band build reads the
//! mesh through, and the one place the (low, high) edge key is made. Ports
//! `Bevel/LevelLine/TriangleEdges.cs` (a C# static class, so a module of functions).

use indexmap::IndexMap;
use vector_math::Vector3;

use super::fast_hash::FastBuildHasher;
use super::LevelLineTriangle;

/// Per edge key (low, high), the triangles standing on it, in triangle order; the map keeps the
/// order the edges were first met in (C# `Dictionary` insertion order), hashed by the cheap
/// [`FastBuildHasher`] since the order never depends on the hash.
pub type StandingMap = IndexMap<(i32, i32), Vec<i32>, FastBuildHasher>;

/// Which triangles of `indices` stand on each edge.
pub fn standing_map(indices: &[(i32, i32, i32)]) -> StandingMap {
    let mut standing =
        StandingMap::with_capacity_and_hasher(indices.len() * 3 / 2, FastBuildHasher::default());
    for (i, &(a, b, c)) in indices.iter().enumerate() {
        let i = i as i32;
        standing_on(&mut standing, a, b).push(i);
        standing_on(&mut standing, b, c).push(i);
        standing_on(&mut standing, c, a).push(i);
    }

    standing
}

/// The list of triangles standing on edge `a b`, made empty on first ask (C# `Standing`).
fn standing_on(map: &mut StandingMap, a: i32, b: i32) -> &mut Vec<i32> {
    map.entry(key(a, b))
        .or_insert_with(|| Vec::with_capacity(2))
}

/// The directed corners - (from, to) - a working-mesh triangle traverses, in winding order (C#
/// `Corners(LevelLineTriangle)`).
pub fn level_line_triangle_corners(t: LevelLineTriangle) -> [(i32, i32); 3] {
    [(t.a, t.b), (t.b, t.c), (t.c, t.a)]
}

/// The (low, high) key of an edge.
pub fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// True when triangle `t` traverses the directed edge `from -> to`.
pub fn traverses(t: (i32, i32, i32), from: i32, to: i32) -> bool {
    (t.0 == from && t.1 == to) || (t.1 == from && t.2 == to) || (t.2 == from && t.0 == to)
}

/// The directed corners - (from, to) - an index triangle traverses, in winding order (C#
/// `Corners((int A, int B, int C))`).
pub fn corners(t: (i32, i32, i32)) -> [(i32, i32); 3] {
    [(t.0, t.1), (t.1, t.2), (t.2, t.0)]
}

/// C# `TriangleEdges.Normal`: the index triangle's unit normal by its winding.
pub fn normal(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    let a = positions[t.0 as usize];
    (positions[t.1 as usize] - a)
        .cross(positions[t.2 as usize] - a)
        .get_normal()
}
