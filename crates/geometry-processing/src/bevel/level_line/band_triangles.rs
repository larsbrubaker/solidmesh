//! Which of a side's triangles the build has to look at: the ones the level line can reach, the
//! edges its roots may be found on, and the band the cut then takes out. Ports
//! `Bevel/LevelLine/BandTriangles.cs` (a C# static class, so a module of functions), with the
//! reach test ([`candidates`]).

use std::collections::HashSet;

use super::triangle_edges;
use vector_math::Vector3;

use super::{ChainDistanceField, LevelLineMesh, WorkError};

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

/// C# `BandTriangles.Candidates`: the side's triangles the line can reach - exactly those whose
/// least field value over the WHOLE triangle, interior included, is at or below `snap`. Seeding by
/// the corners plus one ring of neighbours misses a long triangle whose corners all stand outside
/// the line while its middle dips across it; the exact minimum answers it once. Two boxes say "no"
/// first for the many triangles nowhere near the chain, never turning away one that reaches it.
pub fn candidates(
    mesh: &LevelLineMesh,
    field: &dyn ChainDistanceField,
    side: i32,
    snap: f64,
) -> Result<Vec<i32>, WorkError> {
    let mut near = Vec::new();
    for i in 0..mesh.triangle_count() {
        let t = mesh.triangles()[i as usize];
        if t.tag != side || mesh.is_removed(i) {
            continue;
        }

        let a = mesh.position(t.a);
        let b = mesh.position(t.b);
        let c = mesh.position(t.c);
        let min = Vector3::component_min(Vector3::component_min(a, b), c);
        let max = Vector3::component_max(Vector3::component_max(a, b), c);
        if field.nothing_reaches_below(min, max, snap) {
            continue;
        }

        if field.min_over_triangle(a, b, c)?.value <= snap {
            near.push(i);
        }
    }

    Ok(near)
}
