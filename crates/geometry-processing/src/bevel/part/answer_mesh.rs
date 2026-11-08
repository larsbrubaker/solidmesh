//! A level-line answer as a [`Mesh`] ([`the_mesh_of`]): the kernel answers in positions and
//! triangles that still carry the band's orphaned vertices, and a published mesh (and the
//! self-intersection check, `self_intersection_check`) wants only the vertices its triangles
//! stand on.

use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::Mesh;

/// The kernel's answer as a [`Mesh`], carrying only the vertices some kept
/// triangle stands on - the band's own vertices are orphans by now and a published mesh has no
/// use for them. Vertices are numbered in the order the triangles first reach them.
pub fn the_mesh_of(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> Mesh {
    let mut mesh = Mesh::new();
    let mut moved = vec![-1i32; positions.len()];

    let mut place = |mesh: &mut Mesh, vertex: i32| -> i32 {
        let vertex = vertex as usize;
        if moved[vertex] < 0 {
            moved[vertex] = mesh.vertices.len() as i32;
            mesh.vertices
                .push(Vector3Float::from_vector3(positions[vertex]));
        }

        moved[vertex]
    };

    for &(a, b, c) in indices {
        // Placed in argument order: a, then b, then c, so new vertices are numbered in that order.
        let a = place(&mut mesh, a);
        let b = place(&mut mesh, b);
        let c = place(&mut mesh, c);
        mesh.faces.add(a, b, c, &mesh.vertices);
    }

    mesh
}
