//! Rewinding a surface's backward patch and joining coincident vertices before a kernel import.
//! Called by `minkowski_processing`'s operand import and by `mesh_repair_rust`.
//!
//! The kernel import balances directed edges and refuses a surface where two neighbours run a
//! shared edge the same way (`NotClosed`), though a closed orientable surface with a backward
//! patch has exactly one consistent winding up to its sign. Each connected surface keeps the
//! winding most of its triangles have; only the minority flips, so a whole inside-out shell is
//! left alone. Connectivity is by exact vertex position, as the kernel welds. Face colours and
//! textures carry across, a flipped face's texture corners reversed with it.

use std::collections::{HashMap, VecDeque};

use solidmesh_math::Vector3Float;

use crate::mesh::f32_key;
use crate::{FaceList, FaceTextureData, Mesh};

/// A copy with each connected surface's minority winding flipped to its majority and exactly
/// coincident vertices joined, or `None` when there is nothing to flip or no consistent winding
/// exists (a Möbius-like strip).
///
/// Only edges with exactly two faces link. Faces that collapse onto a repeated position are
/// skipped, as the import drops them.
pub fn rewind(mesh: &Mesh) -> Option<Mesh> {
    let faces = &mesh.faces;
    let (positions, welded) = weld_by_position(mesh);
    let mut corners: Vec<Option<(i32, i32, i32)>> = vec![None; faces.len()];
    let mut edge_faces: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (face_index, face) in faces.iter().enumerate() {
        let a = welded[face.v0 as usize];
        let b = welded[face.v1 as usize];
        let c = welded[face.v2 as usize];
        if a == b || b == c || c == a {
            continue;
        }
        corners[face_index] = Some((a, b, c));
        for (start, end) in [(a, b), (b, c), (c, a)] {
            edge_faces
                .entry(key(start, end))
                .or_default()
                .push(face_index);
        }
    }

    // -1 unvisited; 0 keeps the seed's winding; 1 is reversed relative to it.
    let mut parity = vec![-1i32; faces.len()];
    let mut to_flip = Vec::new();
    let mut queue = VecDeque::new();
    let mut component = Vec::new();
    for seed in 0..faces.len() {
        if parity[seed] >= 0 || corners[seed].is_none() {
            continue;
        }
        parity[seed] = 0;
        queue.push_back(seed);
        component.clear();
        let mut reversed_count = 0usize;
        while let Some(face_index) = queue.pop_front() {
            component.push(face_index);
            reversed_count += parity[face_index] as usize;
            let (a, b, c) = corners[face_index]?;
            for (start, end) in [(a, b), (b, c), (c, a)] {
                let sharing = &edge_faces[&key(start, end)];
                if sharing.len() != 2 {
                    continue;
                }
                let neighbour = if sharing[0] == face_index {
                    sharing[1]
                } else {
                    sharing[0]
                };
                // Consistent neighbours run a shared edge in opposite directions, so a
                // neighbour running it the same way has the opposite parity.
                let neighbour_corners = corners[neighbour]?;
                let wanted = if runs_edge(neighbour_corners, start, end) {
                    1 - parity[face_index]
                } else {
                    parity[face_index]
                };
                if parity[neighbour] < 0 {
                    parity[neighbour] = wanted;
                    queue.push_back(neighbour);
                } else if parity[neighbour] != wanted {
                    return None;
                }
            }
        }
        // Ties keep the seed's winding; either is as good, and it is deterministic.
        let flip_parity = if reversed_count * 2 > component.len() {
            0
        } else {
            1
        };
        to_flip.extend(
            component
                .iter()
                .copied()
                .filter(|&face_index| parity[face_index] == flip_parity),
        );
    }

    if to_flip.is_empty() {
        return None;
    }
    let mut flip = vec![false; faces.len()];
    for face_index in to_flip {
        flip[face_index] = true;
    }
    Some(shared_vertex_copy(mesh, &positions, &welded, &flip))
}

/// A copy with exactly coincident vertices joined, or `None` when no two vertices share a position.
pub(crate) fn join_coincident_vertices(mesh: &Mesh) -> Option<Mesh> {
    let (positions, welded) = weld_by_position(mesh);
    if positions.len() == mesh.vertices.len() {
        None
    } else {
        Some(shared_vertex_copy(
            mesh,
            &positions,
            &welded,
            &vec![false; mesh.faces.len()],
        ))
    }
}

/// One id per distinct position (float equality, so -0 is 0) and each vertex's id.
fn weld_by_position(mesh: &Mesh) -> (Vec<Vector3Float>, Vec<i32>) {
    let mut position_ids: HashMap<(u32, u32, u32), i32> = HashMap::new();
    let mut positions = Vec::new();
    let mut welded = Vec::with_capacity(mesh.vertices.len());
    for vertex in &mesh.vertices {
        let k = (f32_key(vertex.x), f32_key(vertex.y), f32_key(vertex.z));
        let id = *position_ids.entry(k).or_insert_with(|| {
            positions.push(*vertex);
            (positions.len() - 1) as i32
        });
        welded.push(id);
    }
    (positions, welded)
}

/// The mesh on the welded ids with the flagged faces reversed, faces in their order.
fn shared_vertex_copy(
    mesh: &Mesh,
    positions: &[Vector3Float],
    welded: &[i32],
    flip: &[bool],
) -> Mesh {
    let mut faces = FaceList::new();
    for (face_index, face) in mesh.faces.iter().enumerate() {
        let a = welded[face.v0 as usize];
        let b = welded[face.v1 as usize];
        let c = welded[face.v2 as usize];
        if flip[face_index] {
            faces.add_with_normal(c, b, a, -face.normal);
        } else {
            faces.add_with_normal(a, b, c, face.normal);
        }
    }
    let mut rewound = Mesh::from_vertices_and_faces(positions, &faces);
    rewound.face_colors = mesh.face_colors.clone();
    for (&face_index, texture) in &mesh.face_textures {
        let texture = if flip[face_index as usize] {
            FaceTextureData::new(texture.image.clone(), texture.uv2, texture.uv1, texture.uv0)
        } else {
            texture.clone()
        };
        rewound.face_textures.insert(face_index, texture);
    }
    rewound
}

fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn runs_edge(face: (i32, i32, i32), start: i32, end: i32) -> bool {
    (face.0 == start && face.1 == end)
        || (face.1 == start && face.2 == end)
        || (face.2 == start && face.0 == end)
}
