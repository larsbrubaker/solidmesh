//! Collapsing the thin slivers a boolean leaves where two of a face's corners are one point at the
//! weld's tolerance: the vertices within the weld distance of each other along a face edge are
//! joined into one group (union-find, the lowest index naming it), each group moved to its own
//! middle, and every face two of whose corners joined dropped - unless that would leave the mesh
//! more open than it was.

use std::collections::HashMap;

use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::{FaceList, Mesh};

use super::bevel_working_mesh;

/// Collapses the mesh's thin slivers in place; returns how many faces went (0 where nothing was
/// joined, or where the repair would tear the part - every vertex then put back).
pub fn collapsed_from(mesh: &mut Mesh) -> i32 {
    let weld_distance = bevel_working_mesh::seam_tolerance_for(mesh);
    if weld_distance.is_nan() || weld_distance <= 0.0 {
        // Nothing to scale a tolerance against, so no distance at which to call two points one.
        return 0;
    }

    let (same_vertex, any_to_join) = the_groups_of_one_point(mesh, weld_distance);
    if !any_to_join {
        return 0;
    }

    let before = mesh.vertices.clone();
    move_each_group_to_its_own_middle(mesh, &same_vertex);
    let (rebuilt, source_face_index) = rebuilt(mesh, &same_vertex);

    if bevel_working_mesh::open_edge_count(&rebuilt)
        > bevel_working_mesh::open_edge_count(&mesh.faces)
    {
        // The repair would tear the part. Put every vertex back where it was and leave the mesh to
        // the constructions, which will refuse it by name as they did before.
        mesh.vertices = before;
        return 0;
    }

    let went = (mesh.faces.len() - rebuilt.len()) as i32;
    mesh.faces = rebuilt;
    bevel_working_mesh::carry_face_data_onto(mesh, &source_face_index);
    mesh.mark_as_changed();
    went
}

/// Per vertex, the group it joins (the group's lowest index), and whether any two joined.
pub fn the_groups_of_one_point(mesh: &Mesh, weld_distance: f64) -> (Vec<i32>, bool) {
    let mut one_of: Vec<i32> = (0..mesh.vertices.len() as i32).collect();
    let mut any_to_join = false;
    for face in mesh.faces.iter() {
        any_to_join |=
            join_if_they_are_one_point(mesh, &mut one_of, face.v0, face.v1, weld_distance);
        any_to_join |=
            join_if_they_are_one_point(mesh, &mut one_of, face.v1, face.v2, weld_distance);
        any_to_join |=
            join_if_they_are_one_point(mesh, &mut one_of, face.v2, face.v0, weld_distance);
    }

    for i in 0..one_of.len() {
        one_of[i] = the_one_it_is(&mut one_of, i as i32);
    }

    (one_of, any_to_join)
}

/// Joins `a` and `b`'s groups where they stand within the weld distance (the lower group naming
/// the union); true when two groups became one.
pub fn join_if_they_are_one_point(
    mesh: &Mesh,
    one_of: &mut [i32],
    a: i32,
    b: i32,
    weld_distance: f64,
) -> bool {
    if a == b
        || (Vector3::from_vector3_float(mesh.vertices[a as usize])
            - Vector3::from_vector3_float(mesh.vertices[b as usize]))
        .length()
            > weld_distance
    {
        return false;
    }

    let left = the_one_it_is(one_of, a);
    let right = the_one_it_is(one_of, b);
    if left == right {
        return false;
    }

    if left < right {
        one_of[right as usize] = left;
    } else {
        one_of[left as usize] = right;
    }

    true
}

/// The group `vertex` belongs to, halving the path as it walks.
pub fn the_one_it_is(one_of: &mut [i32], mut vertex: i32) -> i32 {
    while one_of[vertex as usize] != vertex {
        one_of[vertex as usize] = one_of[one_of[vertex as usize] as usize];
        vertex = one_of[vertex as usize];
    }

    vertex
}

/// Moves every vertex of a group of two or more to the group's mean position.
pub fn move_each_group_to_its_own_middle(mesh: &mut Mesh, same_vertex: &[i32]) {
    let mut middles: HashMap<i32, (Vector3, i32)> = HashMap::new();
    for (i, &group) in same_vertex.iter().enumerate() {
        let entry = middles.entry(group).or_insert((Vector3::ZERO, 0));
        entry.0 += Vector3::from_vector3_float(mesh.vertices[i]);
        entry.1 += 1;
    }

    for (i, group) in same_vertex.iter().enumerate() {
        let (sum, count) = middles[group];
        if count > 1 {
            mesh.vertices[i] = Vector3Float::from_vector3(sum / count as f64);
        }
    }
}

/// The faces re-indexed through the groups, every face two of whose corners joined dropped, with
/// the face each kept one came from.
pub fn rebuilt(mesh: &Mesh, same_vertex: &[i32]) -> (FaceList, Vec<i32>) {
    let mut kept = FaceList::new();
    let mut source_face_index = Vec::with_capacity(mesh.faces.len());
    for (face_index, face) in mesh.faces.iter().enumerate() {
        let v0 = same_vertex[face.v0 as usize];
        let v1 = same_vertex[face.v1 as usize];
        let v2 = same_vertex[face.v2 as usize];
        if v0 == v1 || v1 == v2 || v2 == v0 {
            continue;
        }

        kept.add(v0, v1, v2, &mesh.vertices);
        source_face_index.push(face_index as i32);
    }

    (kept, source_face_index)
}
