//! The input triangle list read as EDGES: which triangles stand on an edge, the corners a
//! triangle traverses and in what direction, and the normal an edge of an input triangle reads its
//! turn with ([`normal_an_edge_reads`], by `topology::sliver_normal`'s rule). The plumbing every
//! step of the band build reads the mesh through, and the one place the (low, high) edge key is
//! made.

use indexmap::IndexMap;
use solidmesh_math::Vector3;

use crate::topology::sliver_normal::{
    normal_of_corners, trusted_normal_across_slivers, SINGLE_PRECISION_PLACEMENT,
};

use super::fast_hash::FastBuildHasher;
use super::LevelLineTriangle;

/// Per edge key (low, high), the triangles standing on it, in triangle order; the map keeps the
/// order the edges were first met in, so the build never depends on hashing, hashed by the cheap
/// `FastBuildHasher` since the order never depends on the hash.
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

/// The list of triangles standing on edge `a b`, made empty on first ask.
fn standing_on(map: &mut StandingMap, a: i32, b: i32) -> &mut Vec<i32> {
    map.entry(key(a, b))
        .or_insert_with(|| Vec::with_capacity(2))
}

/// The directed corners - (from, to) - a working-mesh triangle traverses, in winding order.
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

/// The directed corners - (from, to) - an index triangle traverses, in winding order.
pub fn corners(t: (i32, i32, i32)) -> [(i32, i32); 3] {
    [(t.0, t.1), (t.1, t.2), (t.2, t.0)]
}

/// The index triangle's unit normal by its winding.
pub fn normal(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    let a = positions[t.0 as usize];
    (positions[t.1 as usize] - a)
        .cross(positions[t.2 as usize] - a)
        .get_normal()
}

/// The unit normal an edge of input triangle `t` reads its turn with, by the one rule the feature
/// edges are read by (`topology::trusted_normal_across_slivers`): a sliver too thin for its
/// corners to give it a normal reads the triangle it lies along. Where none is reached, the
/// triangle's own corners' normal, all there is to read.
///
/// The corners are taken as placed to single precision. That is exact for the part as it comes
/// in, which the model always hands over from float storage; after an earlier chain's build the
/// list also holds points the build placed in doubles, for which the bound is generous: a
/// triangle the build made thinner than single precision can place reads the triangle it lies
/// along instead of its own corners - the reading a sliver of the input gets, and never one that
/// trusts a direction the corners cannot settle. The list carries no record of which points the
/// build placed, so the bound cannot be narrowed per corner here.
pub fn normal_an_edge_reads(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    t: i32,
) -> Vector3 {
    let corners_of = |at: i32| {
        let tri = indices[at as usize];
        [tri.0, tri.1, tri.2]
    };
    let position = |v: i32| positions[v as usize];
    trusted_normal_across_slivers(
        t,
        &|at| {
            let [a, b, c] = corners_of(at);
            normal_of_corners(
                [position(a), position(b), position(c)],
                SINGLE_PRECISION_PLACEMENT,
            )
        },
        &corners_of,
        &position,
        &|at, (a, b)| {
            standing
                .get(&key(a, b))
                .filter(|on| on.len() == 2)
                .and_then(|on| on.iter().copied().find(|&o| o != at))
        },
    )
    .unwrap_or_else(|| normal(positions, indices[t as usize]))
}
