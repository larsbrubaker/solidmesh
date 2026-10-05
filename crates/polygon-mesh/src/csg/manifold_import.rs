//! Getting a [`Mesh`] into the kernel: the robust import, the orientation repair and the
//! flattening into the kernel's `MeshGL64`. Port of agg-sharp `Csg/ManifoldImport.cs`; called by
//! `manifold_operand_batch` and `boolean_processing::classify_boolean_operand`.
//!
//! Not yet here: `WeldSeams`, the tolerance-weld retry for a mesh the kernel calls `NotClosed`.
//! It needs `Mesh::merge_vertices`, which lands with Phase 2c; until then such a mesh is
//! refused on the first import. `ImportAsOriginal` (the re-tag that attributes result triangles
//! to an operand) is only used by face-colour tracking and comes with it in Phase 2f.

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelStatus, MeshGL64};

use super::manifold_kernel;
use crate::Mesh;

/// The kernel refused a mesh (C# `MeshImportRejectedException`).
#[derive(Debug, Clone)]
pub(crate) struct ImportRejected {
    pub(crate) message: String,
    pub(crate) status: KernelStatus,
}

/// Uploads a mesh, rejecting one the kernel could not accept.
///
/// Always the robust import. For strictly manifold input it is the plain import - same result,
/// and the manifold is not marked soup, so the Auto engine still picks the fast exact
/// pipeline. Closed but non-manifold input is welded into a soup manifold instead of being
/// rejected. Only geometry that is not even closed fails, as `NotClosed`.
///
/// The refusal is returned rather than absorbed: a boolean swallows an error operand as empty
/// geometry and still reports success, which would show up as a part silently missing.
pub(crate) fn import(mesh: &Mesh, repair_orientation: bool) -> Result<Manifold, ImportRejected> {
    manifold_kernel::ensure_configured();

    let imported = Manifold::from_mesh_gl64_robust(&to_rust_mesh_data(mesh));
    let status = imported.status();
    if status != KernelStatus::NoError {
        return Err(ImportRejected {
            message: format!(
                "Manifold input has error status: {status:?} ({} vertices, {} faces)",
                mesh.vertices.len(),
                mesh.faces.len()
            ),
            status,
        });
    }

    if !repair_orientation {
        return Ok(imported);
    }

    // On the imported manifold rather than the mesh: the repair is a kernel operation over the
    // imported shells. A mesh that needs no repair comes back as a plain copy, so this is safe
    // unconditionally.
    let repaired = imported.repair_orientation();
    let status = repaired.status();
    if status != KernelStatus::NoError {
        return Err(ImportRejected {
            message: format!(
                "Manifold orientation repair has error status: {status:?} ({} vertices, {} faces)",
                mesh.vertices.len(),
                mesh.faces.len()
            ),
            status,
        });
    }
    Ok(repaired)
}

/// Flattens a mesh into the interleaved position list and triangle index list the kernel
/// takes. Positions widen from the mesh's f32 to f64 with no narrowing at the boundary; three
/// properties per vertex and nothing else set (no merge vectors, runs or tangents).
fn to_rust_mesh_data(mesh: &Mesh) -> MeshGL64 {
    let mut vert_properties = Vec::with_capacity(mesh.vertices.len() * 3);
    for vertex in &mesh.vertices {
        vert_properties.push(f64::from(vertex.x));
        vert_properties.push(f64::from(vertex.y));
        vert_properties.push(f64::from(vertex.z));
    }

    // Face indices are i32 as in C#; a negative one is out of bounds for the kernel either way,
    // so it is passed as the u64 it wraps to and the import reports VertexOutOfBounds.
    let mut tri_verts = Vec::with_capacity(mesh.faces.len() * 3);
    for face in mesh.faces.iter() {
        tri_verts.push(face.v0 as u64);
        tri_verts.push(face.v1 as u64);
        tri_verts.push(face.v2 as u64);
    }

    MeshGL64 {
        num_prop: 3,
        vert_properties,
        tri_verts,
        ..MeshGL64::default()
    }
}
