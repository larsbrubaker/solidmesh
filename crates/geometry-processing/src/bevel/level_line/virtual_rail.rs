//! Flow rung 2: a side's rail carried on VIRTUALLY past a neighbour that grows under the blend.
//! Where the band has run past an opposite-sense crease, the cut leaves the side's rail as two runs
//! that stop on the crease; the rail between them is laid in the side's own plane at the setback
//! from the chain, as points minted as new vertices. Ports `Bevel/LevelLine/VirtualRail.cs` (a C#
//! static class, so a module of functions); the join itself lands with its method packages.

use indexmap::IndexMap;

use super::triangle_edges;
use super::LevelLineMesh;

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
