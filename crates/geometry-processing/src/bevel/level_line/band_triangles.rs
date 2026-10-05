//! Which of a side's triangles the build has to look at: the ones the level line can reach, the
//! edges its roots may be found on, and the band the cut then takes out. Ports
//! `Bevel/LevelLine/BandTriangles.cs` (a C# static class, so a module of functions); the reach
//! test (`Candidates`) lands with its method package.

use std::collections::HashSet;

use super::triangle_edges;
use super::LevelLineMesh;

/// The edges of a side's live triangles, each once as (low, high), in ascending order.
///
/// Gathered from the SIDE'S triangles rather than asked of every edge in the mesh - the same
/// set, since an edge carries a triangle of the side exactly when one of the side's triangles
/// names it - but read at the size of the band. The roots are found and split in this order and a
/// different order splits into different vertex indices, so the order is CANONICAL: ascending
/// (low, high) over the side's own edge set, which depends on the set and on nothing else the
/// build did first.
pub fn edges_of_side(mesh: &LevelLineMesh, side: i32) -> Vec<(i32, i32)> {
    let mut of_side = HashSet::new();
    for i in 0..mesh.triangle_count() {
        let t = mesh.triangles()[i as usize];
        if t.tag == side && !mesh.is_removed(i) {
            for (a, b) in triangle_edges::level_line_triangle_corners(t) {
                of_side.insert(triangle_edges::key(a, b));
            }
        }
    }

    let mut edges: Vec<(i32, i32)> = of_side.into_iter().collect();
    // Distinct keys: the sort has no ties, so the set's own order never reaches the output.
    edges.sort_unstable();
    edges
}
