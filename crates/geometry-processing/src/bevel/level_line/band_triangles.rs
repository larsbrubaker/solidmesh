//! Which of a side's triangles the build has to look at: the ones the level line can reach, the
//! edges its roots may be found on, and the band the cut then takes out, with the reach test
//! ([`candidates`]).

use std::collections::HashSet;

use super::triangle_edges;
use solidmesh_math::Vector3;

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

/// The side's triangles the line can reach - exactly those whose
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

        if field.triangle_reaches(a, b, c, snap)? {
            near.push(i);
        }
    }

    Ok(near)
}

/// The band of one side - its inside triangles that reach the chain
/// through inside triangles of that same side - read on the reading's own mesh. A vertex ON the
/// chain reads the field as minus its setback, so it is inside by the field's own rule.
#[allow(clippy::needless_range_loop)]
pub fn band_of_side(
    reading: &super::VertexFieldReading,
    side: i32,
    on_the_chain: &HashSet<i32>,
) -> Result<Vec<i32>, super::band_cut::BandCutError> {
    let mesh = reading.mesh.borrow();
    let count = mesh.triangle_count() as usize;

    // A triangle already OUT of the mesh is asked nothing: its edges are detached and no root was
    // ever found on them, so it would read as straddling the line.
    let mut inside = vec![false; count];
    for i in 0..count {
        if mesh.triangles()[i].tag == side && !mesh.is_removed(i as i32) {
            inside[i] =
                super::band_cut::side_of_triangle(reading, i as i32)? == super::VertexSide::Inside;
        }
    }

    let mut band = Vec::new();
    let mut seen = vec![false; count];
    let mut pending = std::collections::VecDeque::new();
    for i in 0..count {
        let t = mesh.triangles()[i];
        if inside[i]
            && !mesh.is_removed(i as i32)
            && (on_the_chain.contains(&t.a)
                || on_the_chain.contains(&t.b)
                || on_the_chain.contains(&t.c))
        {
            seen[i] = true;
            pending.push_back(i as i32);
        }
    }

    while let Some(triangle) = pending.pop_front() {
        super::LevelLineWork::charge(1)?;
        band.push(triangle);
        for (a, b) in
            triangle_edges::level_line_triangle_corners(mesh.triangles()[triangle as usize])
        {
            for &neighbour in mesh.triangles_on(a, b) {
                if !seen[neighbour as usize] && inside[neighbour as usize] {
                    seen[neighbour as usize] = true;
                    pending.push_back(neighbour);
                }
            }
        }
    }

    Ok(band)
}
