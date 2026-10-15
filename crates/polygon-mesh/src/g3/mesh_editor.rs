//! Mesh edits (derived from geometry3Sharp) the mesh repair and the simple hole filler use:
//! removing lists of triangles (and the "loner" triangles with no neighbours), the
//! ordered-vertex-loop triangle fan, splitting bowtie vertices apart, and re-inserting an
//! edited submesh (`reinsert_submesh`, for `RegionOperator`).

use super::dmesh3::{DMesh3, G3Exception, MeshResult, INVALID_ID};
use super::dsubmesh3::{DSubmesh3, INVALID_INDEX};
use super::mesh_iterators::bowtie_vertices;

/// Removes `triangles`, false if any could not be removed: dead ids count as a failure but do
/// not stop the rest.
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

/// Removes the triangles of an id list: [`INVALID_ID`] entries are skipped, any other id (dead
/// ones too) goes to `remove_triangle`, whose failures count against the result.
pub(crate) fn remove_triangles_list(
    mesh: &mut DMesh3,
    triangles: &[i32],
    remove_isolated_verts: bool,
) -> bool {
    let mut all_ok = true;
    for &tid in triangles {
        if tid == INVALID_ID {
            continue;
        }
        if mesh.remove_triangle(tid, remove_isolated_verts, false) != MeshResult::Ok {
            all_ok = false;
        }
    }
    all_ok
}

/// Re-inserts an edited submesh for `RegionOperator`; a duplicate triangle is not checked
/// for, it is simply refused by the mesh like any other. The old region must already be
/// removed from `mesh`. Submesh boundary vertices that map to a base border vertex still on
/// the mesh boundary are reused; every other submesh vertex is appended. `new_tris` gets the
/// new triangle ids in submesh id order (a refused triangle's negative id included); the
/// result is false if any triangle was refused, and `Err` when the submesh has no boundary
/// info.
pub(crate) fn reinsert_submesh(
    mesh: &mut DMesh3,
    sub: &DSubmesh3,
    new_tris: &mut [i32],
) -> Result<bool, G3Exception> {
    if sub.base_border_v.is_none() {
        return Err(G3Exception(
            "MeshEditor.ReinsertSubmesh: Submesh does not have required boundary info. Call ComputeBoundaryInfo()!",
        ));
    }

    let submesh = &sub.sub_mesh;
    let mut all_ok = true;

    let max_vid = submesh.max_vertex_id() as usize;
    let mut done_v = vec![false; max_vid];
    let mut sub_to_new_v = vec![INVALID_INDEX; max_vid];

    let mut nti = 0;
    let nt = submesh.max_triangle_id();
    for ti in 0..nt {
        if !submesh.is_triangle(ti) {
            continue;
        }

        let sub_t = submesh.get_triangle(ti);
        let gid = submesh.get_triangle_group(ti);

        let mut new_t = [0; 3];
        for j in 0..3 {
            let sub_v = sub_t[j];
            let mut new_v = -1;
            if !done_v[sub_v as usize] {
                // first check if this is a boundary vtx on submesh and maps to a bdry vtx on base mesh
                if submesh.is_boundary_vertex(sub_v) {
                    let base_v = if (sub_v as usize) < sub.sub_to_base_v.len() {
                        sub.sub_to_base_v[sub_v as usize]
                    } else {
                        -1
                    };
                    if base_v >= 0
                        && mesh.is_vertex(base_v)
                        && sub.is_base_border_v(base_v)
                        && mesh.is_boundary_vertex(base_v)
                    {
                        new_v = base_v;
                    }
                }

                // if that didn't happen, append new vtx
                if new_v == -1 {
                    new_v = mesh.append_vertex(submesh.vertex(sub_v));
                }

                sub_to_new_v[sub_v as usize] = new_v;
                done_v[sub_v as usize] = true;
            } else {
                new_v = sub_to_new_v[sub_v as usize];
            }
            new_t[j] = new_v;
        }

        let new_tid = mesh.append_triangle_g(new_t[0], new_t[1], new_t[2], gid);
        if !mesh.is_triangle(new_tid) {
            all_ok = false;
        }
        new_tris[nti] = new_tid;
        nti += 1;
    }

    Ok(all_ok)
}

/// Removes the live triangles `selector` picks, walking ids `0..max_triangle_id()` in order and
/// testing each as the walk reaches it, so earlier removals can change later answers.
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

/// Removes triangles with no neighbour across any edge.
pub(crate) fn remove_isolated_triangles(mesh: &mut DMesh3) -> bool {
    remove_triangles_where(
        mesh,
        |mesh, tid| mesh.get_tri_neighbour_tris(tid) == [INVALID_ID; 3],
        true,
    )
}

/// The back-out of a failed fan.
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

/// Triangles `[center, b, a]` for each
/// consecutive `[a, b]` of the loop. `Ok(None)` when one is refused (the ones added so far are
/// removed again); `Err` when that back-out itself fails.
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

/// Groups the vertex's triangles into edge-connected fans
/// (greedily, in one-ring order) and gives every fan but the largest its own copy
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

/// Repeats until no bowtie is left or the
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

/// Sorts a bowtie's fans largest first, equal sizes comparing equal. Two and three fans go
/// through a fixed `swap_if_greater` sequence (which can reorder equal fans), longer lists
/// through an insertion sort (which keeps them). Which fan keeps the original vertex decides
/// the repaired mesh's vertex ids, so this order is pinned exactly; more than 16 fans on one
/// vertex is rare enough that the insertion sort simply continues there.
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
