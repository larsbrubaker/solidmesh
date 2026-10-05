//! The part of geometry3Sharp's `mesh/MeshEditor.cs` the mesh repair and the simple hole filler
//! reach: removing lists of triangles (and the "loner" triangles with no neighbours), the
//! ordered-vertex-loop triangle fan, and splitting bowtie vertices apart.

// The mesh repair (geometry3Sharp `MeshAutoRepair`) is the consumer of these and lands in a
// later slice; until then only the tests reach some of them.
#![allow(dead_code)]

use super::dmesh3::{DMesh3, G3Exception, MeshResult, INVALID_ID};
use super::mesh_iterators::bowtie_vertices;

/// C# `MeshEditor.RemoveTriangles(IEnumerable<int>, bRemoveIsolatedVerts)` (and its static
/// form): dead ids count as a failure but do not stop the rest.
pub(crate) fn remove_triangles(
    mesh: &mut DMesh3,
    triangles: impl IntoIterator<Item = i32>,
    remove_isolated_verts: bool,
) -> bool {
    let mut all_ok = true;
    for tid in triangles {
        if !mesh.is_triangle(tid) {
            all_ok = false;
            continue;
        }
        if mesh.remove_triangle(tid, remove_isolated_verts, false) != MeshResult::Ok {
            all_ok = false;
        }
    }
    all_ok
}

/// C# `MeshEditor.RemoveTriangles(Func<int, bool> selectorF, bRemoveIsolatedVerts)`: ids
/// `0..MaxTriangleID` in order, each tested as the walk reaches it.
fn remove_triangles_where(
    mesh: &mut DMesh3,
    selector: impl Fn(&DMesh3, i32) -> bool,
    remove_isolated_verts: bool,
) -> bool {
    let mut all_ok = true;
    let nt = mesh.max_triangle_id();
    for ti in 0..nt {
        if !mesh.is_triangle(ti) || !selector(mesh, ti) {
            continue;
        }
        if mesh.remove_triangle(ti, remove_isolated_verts, false) != MeshResult::Ok {
            all_ok = false;
        }
    }
    all_ok
}

/// C# `MeshEditor.RemoveIsolatedTriangles`: removes triangles with no neighbour across any edge.
pub(crate) fn remove_isolated_triangles(mesh: &mut DMesh3) -> bool {
    remove_triangles_where(
        mesh,
        |mesh, tid| mesh.get_tri_neighbour_tris(tid) == [INVALID_ID; 3],
        true,
    )
}

/// C# `MeshEditor.remove_triangles(int[], count)`: the back-out of a failed fan.
fn remove_triangles_backout(mesh: &mut DMesh3, tri_list: &[i32]) -> bool {
    for &tid in tri_list {
        if !mesh.is_triangle(tid) {
            continue;
        }
        if mesh.remove_triangle(tid, false, false) != MeshResult::Ok {
            return false;
        }
    }
    true
}

/// C# `MeshEditor.AddTriangleFan_OrderedVertexLoop`: triangles `[center, b, a]` for each
/// consecutive `[a, b]` of the loop. `Ok(None)` when one is refused (the ones added so far are
/// removed again); `Err` when that back-out itself fails, which C# throws.
pub(crate) fn add_triangle_fan_ordered_vertex_loop(
    mesh: &mut DMesh3,
    center: i32,
    vertex_loop: &[i32],
    group_id: i32,
) -> Result<Option<Vec<i32>>, G3Exception> {
    let n = vertex_loop.len();
    let mut new_tris = Vec::with_capacity(n);
    for i in 0..n {
        let a = vertex_loop[i];
        let b = vertex_loop[(i + 1) % n];
        let new_tid = mesh.append_triangle_g(center, b, a, group_id);
        if new_tid < 0 {
            // remove what we added so far
            if i > 0 && !remove_triangles_backout(mesh, &new_tris) {
                return Err(G3Exception(
                    "MeshEditor.AddTriangleFan_OrderedVertexLoop: failed to add fan, and also falied to back out changes.",
                ));
            }
            return Ok(None);
        }
        new_tris.push(new_tid);
    }
    Ok(Some(new_tris))
}

/// C# `MeshEditor.DisconnectBowtie`: groups the vertex's triangles into edge-connected fans
/// (greedily, in one-ring order, as C# does) and gives every fan but the largest its own copy
/// of the vertex.
pub(crate) fn disconnect_bowtie(mesh: &mut DMesh3, vid: i32) {
    let mut sets: Vec<Vec<i32>> = Vec::new();
    for tid in mesh.vtx_triangles(vid) {
        let nbrs = mesh.get_tri_neighbour_tris(tid);
        match sets
            .iter_mut()
            .find(|set| set.contains(&nbrs[0]) || set.contains(&nbrs[1]) || set.contains(&nbrs[2]))
        {
            Some(set) => set.push(tid),
            None => sets.push(vec![tid]),
        }
    }
    if sets.len() == 1 {
        return; // not a bowtie!
    }

    dotnet_sort_by_size_descending(&mut sets);
    for tris in sets.iter().skip(1) {
        let copy_vid = mesh.append_vertex_copy(vid);
        for &tid in tris {
            let mut t = mesh.get_triangle(tid);
            if t[0] == vid {
                t[0] = copy_vid;
            } else if t[1] == vid {
                t[1] = copy_vid;
            } else {
                t[2] = copy_vid;
            }
            mesh.set_triangle(tid, t, false);
        }
    }
}

/// C# `MeshEditor.DisconnectAllBowties(nMaxIters)`: repeats until no bowtie is left or the
/// iterations run out; returns how many remain.
pub(crate) fn disconnect_all_bowties(mesh: &mut DMesh3, max_iters: i32) -> usize {
    let mut bowties = bowtie_vertices(mesh);
    let mut iter = 0;
    while !bowties.is_empty() && {
        let go = iter < max_iters;
        iter += 1;
        go
    } {
        for &vid in &bowties {
            disconnect_bowtie(mesh, vid);
        }
        bowties = bowtie_vertices(mesh);
    }
    bowties.len()
}

/// C# `sets.Sort(bowtie_sorter)` (largest first, equal sizes compare equal), reproducing
/// .NET's `Array.Sort` for up to 16 elements: two and three elements go through its fixed
/// `SwapIfGreater` sequence (which can reorder equal fans), longer lists through its insertion
/// sort (which keeps them). Beyond 16 .NET switches to its introsort; this keeps inserting,
/// which can order equal-sized fans differently there (divergence 790).
fn dotnet_sort_by_size_descending(sets: &mut [Vec<i32>]) {
    let greater = |sets: &[Vec<i32>], i: usize, j: usize| sets[i].len() < sets[j].len();
    let swap_if_greater = |sets: &mut [Vec<i32>], i: usize, j: usize| {
        if greater(sets, i, j) {
            sets.swap(i, j);
        }
    };
    match sets.len() {
        0 | 1 => {}
        2 => swap_if_greater(sets, 0, 1),
        3 => {
            swap_if_greater(sets, 0, 1);
            swap_if_greater(sets, 0, 2);
            swap_if_greater(sets, 1, 2);
        }
        n => {
            for i in 0..n - 1 {
                let mut j = i as isize;
                let t = sets[i + 1].clone();
                while j >= 0 && t.len() > sets[j as usize].len() {
                    sets[(j + 1) as usize] = sets[j as usize].clone();
                    j -= 1;
                }
                sets[(j + 1) as usize] = t;
            }
        }
    }
}
