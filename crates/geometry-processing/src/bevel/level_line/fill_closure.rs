//! The fill checking the claim it makes: the strip it just laid closes the gap. Not a repair pass -
//! it changes nothing, is linear in the triangles the fill laid, and either confirms the
//! construction's promise or names the first edge that breaks it.

use indexmap::{IndexMap, IndexSet};

use super::{GapLoop, LevelLineMesh};

/// True when more triangles run `edge` A to B than the fill laid that way
/// (`laid_along_it`): the rest are kept. Linear in the triangles on the one edge.
pub fn stands_on_kept(mesh: &LevelLineMesh, edge: (i32, i32), laid_along_it: i32) -> bool {
    let mut along = 0;

    for &t in mesh.triangles_on(edge.0, edge.1) {
        let face = mesh.triangles[t as usize];

        for corner in 0..3 {
            if face.corner(corner) == edge.0 && face.corner((corner + 1) % 3) == edge.1 {
                along += 1;
            }
        }
    }

    along > laid_along_it
}

/// Every directed edge of a fill
/// triangle in `added_triangles` must be answered exactly once - by the opposite fill edge inside
/// the strip, or, along the gap's own boundary (`loops`, as the KEPT surface traverses it), by the
/// kept triangle that edge stands on. The first edge that is not is handed back; `(-1, -1)` where
/// every one is.
pub fn first_open_edge(
    mesh: &LevelLineMesh,
    loops: &[GapLoop],
    added_triangles: &[i32],
) -> (i32, i32) {
    let mut kept = Vec::new();

    for gap_loop in loops {
        let count = gap_loop.vertices.len();

        for i in 0..count {
            kept.push((gap_loop.vertices[i], gap_loop.vertices[(i + 1) % count]));
        }
    }

    first_open_edge_of(mesh, &kept, added_triangles)
}

/// The same check over a
/// boundary given directly, which is what an OPEN gap needs - its two runs chain into one cycle
/// through their shared ends.
///
/// The edge counts and the edge set are walked in insertion order to find the FIRST open edge, so
/// both are insertion-ordered: the edge named is the same on every run.
pub fn first_open_edge_of(
    mesh: &LevelLineMesh,
    kept_edges: &[(i32, i32)],
    added_triangles: &[i32],
) -> (i32, i32) {
    let kept: IndexSet<(i32, i32)> = kept_edges.iter().copied().collect();
    let mut laid: IndexMap<(i32, i32), i32> = IndexMap::new();

    for &triangle in added_triangles {
        let face = mesh.triangles[triangle as usize];

        for corner in 0..3 {
            let edge = (face.corner(corner), face.corner((corner + 1) % 3));
            *laid.entry(edge).or_insert(0) += 1;
        }
    }

    for (&edge, &count) in &laid {
        let back = (edge.1, edge.0);
        let kept_back = i32::from(kept.contains(&back));
        let opposite = laid.get(&back).copied().unwrap_or(0);

        if count != 1 || kept_back + opposite != 1 {
            return edge;
        }

        // Counting answers is not enough: a fill triangle laid as the MIRROR of a kept triangle
        // answers its strip neighbours edge for edge while the kept one still stands on the same
        // edges - a doubled flap. Nothing but the fill's own triangles, and on the boundary the one
        // kept triangle, may stand on an edge the fill laid.
        if mesh.triangles_on(edge.0, edge.1).len() as i32 != count + opposite + kept_back {
            return edge;
        }
    }

    for &edge in &kept {
        // A kept edge must truly stand on a kept triangle: a rail point the kept surface never
        // reaches (a virtual rail, an unclosed hole) is not answered by claiming it.
        let answered = laid.get(&(edge.1, edge.0)).copied();
        let along = laid.get(&edge).copied().unwrap_or(0);

        if answered != Some(1) || !stands_on_kept(mesh, edge, along) {
            return edge;
        }
    }

    (-1, -1)
}
