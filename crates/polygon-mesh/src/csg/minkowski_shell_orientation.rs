//! Rewinding an operand's shells whose winding contradicts their nesting, so a part saved inside
//! out reads as the solid it looks like. Port of agg-sharp `Csg/MinkowskiShellOrientation.cs`
//! (`Repair`, `MayHaveInvertedShell`, `MeshMayNeedRepair`); called by `minkowski_processing` and
//! `sdf::mesh_sdf_grid`.

use std::collections::HashMap;

use manifold_rust::manifold::Manifold;
use manifold_rust::types::Error as KernelStatus;

use crate::{CancelToken, Mesh};

/// C# `Repair`: the operand with every mis-wound shell rewound (the kernel's
/// `repair_orientation`), or the operand itself when no shell can be. `None` when cancelled
/// (C# throws `OperationCanceledException`).
///
/// The repair costs about shells x triangles, so it runs only when a shell may be inverted. C#
/// passes no kernel token for a caller's token that can never be cancelled; every
/// [`CancelToken`] can be, so the kernel always polls it.
pub(crate) fn repair(imported: Manifold, cancel: &CancelToken) -> Option<Manifold> {
    if !may_have_inverted_shell(&imported) {
        return Some(imported);
    }
    let repaired = imported.repair_orientation_with_token(Some(cancel.kernel_token()));
    match repaired.status() {
        KernelStatus::Cancelled => None,
        // The repair only flips triangles; if it ever fails, the operand as imported is the
        // better answer than none.
        KernelStatus::NoError => Some(repaired),
        _ => Some(imported),
    }
}

/// C# `MayHaveInvertedShell`: false only when every edge-connected shell encloses a positive
/// volume, the case in which the repair provably changes nothing. A soup import, or one with
/// an unpaired halfedge, always takes the repair.
pub fn may_have_inverted_shell(manifold: &Manifold) -> bool {
    let imp = manifold.as_impl();
    if imp.is_soup {
        return true;
    }
    let halfedges = &imp.halfedge;
    let positions = &imp.vert_pos;
    let triangles = halfedges.len() / 3;
    let mut parent: Vec<usize> = (0..triangles).collect();
    for (h, halfedge) in halfedges.iter().enumerate() {
        let pair = halfedge.paired_halfedge;
        if pair < 0 {
            return true;
        }
        let a = find(&mut parent, h / 3);
        let b = find(&mut parent, pair as usize / 3);
        if a != b {
            parent[a] = b;
        }
    }

    // Six times the volume per shell: each triangle's tetrahedron with the origin.
    let mut volume = vec![0.0f64; triangles];
    for t in 0..triangles {
        let p0 = positions[halfedges[3 * t].start_vert as usize];
        let p1 = positions[halfedges[3 * t + 1].start_vert as usize];
        let p2 = positions[halfedges[3 * t + 2].start_vert as usize];
        let root = find(&mut parent, t);
        volume[root] += triple([p0.x, p0.y, p0.z], [p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]);
    }
    (0..triangles)
        .any(|t| parent[t] == t && volume[t].partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater))
}

/// C# `MeshMayNeedRepair`: `may_have_inverted_shell`'s rule read straight off a mesh, before
/// any import. False only when the mesh is closed and consistently wound (every directed edge
/// met exactly once, and its reverse exactly once) and every edge-connected shell encloses a
/// positive volume; such a mesh imports with nothing to join or rewind. O(triangles).
pub fn mesh_may_need_repair(mesh: &Mesh) -> bool {
    let faces = &mesh.faces;
    let count = faces.len();
    if count == 0 {
        return true;
    }

    let vertex_count = mesh.vertices.len() as i64;
    let mut edge_face: HashMap<i64, usize> = HashMap::with_capacity(count * 3);
    for (f, face) in faces.iter().enumerate() {
        for (from, to) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            if edge_face
                .insert(from as i64 * vertex_count + to as i64, f)
                .is_some()
            {
                // A directed edge used twice: a backward patch or a non-manifold edge.
                return true;
            }
        }
    }

    let mut parent: Vec<usize> = (0..count).collect();
    for (&key, &f) in &edge_face {
        let (from, to) = (key / vertex_count, key % vertex_count);
        let Some(&other) = edge_face.get(&(to * vertex_count + from)) else {
            // An unpaired edge: open, or a split seam the import would join.
            return true;
        };
        let a = find(&mut parent, f);
        let b = find(&mut parent, other);
        if a != b {
            parent[a] = b;
        }
    }

    let mut volume = vec![0.0f64; count];
    let vertices = &mesh.vertices;
    for (f, face) in faces.iter().enumerate() {
        let corner = |v: i32| {
            let p = vertices[v as usize];
            [p.x as f64, p.y as f64, p.z as f64]
        };
        let root = find(&mut parent, f);
        volume[root] += triple(corner(face.v0), corner(face.v1), corner(face.v2));
    }
    (0..count)
        .any(|f| parent[f] == f && volume[f].partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater))
}

/// The union-find root with path halving, as C#'s local `Find`.
fn find(parent: &mut [usize], mut t: usize) -> usize {
    while parent[t] != t {
        parent[t] = parent[parent[t]];
        t = parent[t];
    }
    t
}

/// p0 . (p1 x p2), in C#'s operation order.
fn triple(p0: [f64; 3], p1: [f64; 3], p2: [f64; 3]) -> f64 {
    (p0[0] * ((p1[1] * p2[2]) - (p1[2] * p2[1]))) - (p0[1] * ((p1[0] * p2[2]) - (p1[2] * p2[0])))
        + (p0[2] * ((p1[0] * p2[1]) - (p1[1] * p2[0])))
}
