//! Flow rung 2: a side's rail carried on VIRTUALLY past a neighbour that grows under the blend.
//! Where the band has run past an opposite-sense crease, the cut leaves the side's rail as two runs
//! that stop on the crease; the rail between them is laid in the side's own plane at the setback
//! from the chain, as points minted as new vertices. Ports `Bevel/LevelLine/VirtualRail.cs` (a C#
//! static class, so a module of functions): the chain's fold, the opposite-sense crease's vertices
//! ([`opposite_sense_crease_vertices`]); the join itself lands with its method packages.

use std::collections::HashSet;

use indexmap::IndexMap;

use super::{end_face_sense, triangle_edges};
use super::{LevelLineMesh, LevelLineTriangle};

/// A side-1 and a side-2 triangle meeting on an edge - the chain's fold, whose sense the crease is
/// read against - removed or not, since the cut takes both away, as (near, far, vertex). The least
/// such edge, so the read is fixed.
pub fn chain_pair(mesh: &LevelLineMesh) -> Option<(i32, i32, i32)> {
    let mut side_one: IndexMap<(i32, i32), i32> = IndexMap::new();
    for t in 0..mesh.triangle_count() {
        let triangle = mesh.triangles()[t as usize];
        if triangle.tag == 1 {
            for (p, q) in triangle_edges::level_line_triangle_corners(triangle) {
                side_one.insert(triangle_edges::key(p, q), t);
            }
        }
    }

    let mut best = None;
    let mut best_key = (i32::MAX, i32::MAX);
    for t in 0..mesh.triangle_count() {
        let triangle = mesh.triangles()[t as usize];
        if triangle.tag != 2 {
            continue;
        }

        for (p, q) in triangle_edges::level_line_triangle_corners(triangle) {
            let key = triangle_edges::key(p, q);
            if let Some(&near) = side_one.get(&key) {
                if key < best_key {
                    best_key = key;
                    best = Some((near, t, p));
                }
            }
        }
    }

    best
}

/// C# `OppositeSenseCreaseVertices`: every vertex on a crease between a removed triangle of `side`
/// and a kept untagged one that folds opposite to the chain. The side's triangles there are gone
/// after the cut, so the sense is read off the removed triangles themselves, and the chain's off
/// its two sides' triangles on the chain's fold ([`chain_pair`]), removed or not. Only looked up.
pub fn opposite_sense_crease_vertices(mesh: &LevelLineMesh, side: i32) -> HashSet<i32> {
    let corners = |t: LevelLineTriangle| (t.a, t.b, t.c);
    let mut found = HashSet::new();

    let Some((near, far, vertex)) = chain_pair(mesh) else {
        return found;
    };

    for r in 0..mesh.triangle_count() {
        let removed = mesh.triangles[r as usize];

        if !mesh.is_removed(r) || removed.tag != side {
            continue;
        }

        for (p, q) in triangle_edges::corners(corners(removed)) {
            let standing = mesh.triangles_on(p, q);

            if standing.len() != 1 || mesh.triangles[standing[0] as usize].tag != 0 {
                continue;
            }

            if end_face_sense::is_opposite_sense_crease(
                mesh.vertices(),
                corners(mesh.triangles[near as usize]),
                corners(mesh.triangles[far as usize]),
                vertex,
                corners(removed),
                corners(mesh.triangles[standing[0] as usize]),
                p,
            ) {
                found.insert(p);
                found.insert(q);
            }
        }
    }

    found
}
