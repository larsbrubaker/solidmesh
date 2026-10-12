//! Getting a [`Mesh`] into the kernel: the robust import, the orientation repair and the
//! flattening into the kernel's `MeshGL64`. Port of agg-sharp `Csg/ManifoldImport.cs`; called by
//! `manifold_operand_batch` and `boolean_processing::classify_boolean_operand`.
//!
//! A mesh the kernel calls `NotClosed` gets one retry after `weld_seams` (a tolerance weld);
//! [`import_strict`] is the one-shot import with no retry, for callers that must read exactly
//! what the kernel makes of the mesh as given.
//! `ImportAsOriginal` (the re-tag that attributes result triangles
//! to an operand) is only used by face-colour tracking and comes with it in Phase 2f.

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelStatus, MeshGL64};

use vector_math::Vector3;

use super::manifold_kernel;
use crate::Mesh;

/// The kernel refused a mesh (C# `MeshImportRejectedException`).
#[derive(Debug, Clone)]
pub struct ImportRejected {
    pub message: String,
    pub status: KernelStatus,
}

/// Uploads a mesh, rejecting one the kernel could not accept.
///
/// Always the robust import. For strictly manifold input it is the plain import - same result,
/// and the manifold is not marked soup, so the Auto engine still picks the fast exact
/// pipeline. Closed but non-manifold input is welded into a soup manifold instead of being
/// rejected. Only geometry that is not even closed still fails, as `NotClosed` - and that one
/// gets a second chance through [`weld_seams`] before it is refused.
///
/// The refusal is returned rather than absorbed: a boolean swallows an error operand as empty
/// geometry and still reports success, which would show up as a part silently missing.
pub fn import(mesh: &Mesh, repair_orientation: bool) -> Result<Manifold, ImportRejected> {
    manifold_kernel::ensure_configured();

    let rejected = match try_import(mesh, repair_orientation) {
        Ok(imported) => return Ok(imported),
        Err(rejected) => rejected,
    };

    if rejected.status == KernelStatus::NotClosed {
        if let Some(welded) = weld_seams(mesh) {
            if let Ok(retried) = try_import(&welded, repair_orientation) {
                return Ok(retried);
            }
        }
    }

    Err(rejected)
}

/// Uploads a mesh exactly as given: C#'s `Manifold.FromMeshGL64Robust` over the same
/// marshalling, with no weld-and-retry and no orientation repair.
///
/// For callers that must see the kernel's own reading of the mesh - the bevel tests that check
/// a built fillet is a solid the kernel takes, where a tolerance weld would close a seam the
/// builder left open and hide it.
pub fn import_strict(mesh: &Mesh) -> Result<Manifold, ImportRejected> {
    manifold_kernel::ensure_configured();
    try_import(mesh, false)
}

/// Uploads a mesh, handing back the status that explains a failure so [`import`] can decide
/// whether it is worth a retry (C# `TryImport`).
fn try_import(mesh: &Mesh, repair_orientation: bool) -> Result<Manifold, ImportRejected> {
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

/// A tolerance-welded copy of a mesh, or `None` when there is no sane scale to weld at.
///
/// The kernel welds vertices by exact f64 position and has no tolerance of its own, so a seam
/// whose two sides differ in the last bits is a pair of boundary edges to it and a visually
/// closed solid reports `NotClosed`. Positions are stored as `Vector3Float` and every
/// transform re-rounds them, so a seam shared on disk can come apart in the last digit.
/// Welding with a tolerance taken from the bounding box - both its size and how far it sits
/// from the origin (the float grid step at that coordinate) - closes those seams without
/// moving anything a user could see. Only ever called on the failure path.
pub(crate) fn weld_seams(mesh: &Mesh) -> Option<Mesh> {
    let aabb = mesh.get_axis_aligned_bounding_box();
    let diagonal = aabb.size().length();
    // `partial_cmp` so a NaN diagonal (a non-finite vertex) is refused like zero, as `!(d > 0)`.
    if diagonal.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) || diagonal.is_infinite() {
        // No extent to scale a tolerance against. A non-finite vertex lands here too, and
        // welding is not the answer to that one anyway.
        return None;
    }

    // Whichever of the part's size and its distance from the origin is larger sets the scale
    // of a seam gap.
    let distance_from_origin = max_abs_component(aabb.min_xyz).max(max_abs_component(aabb.max_xyz));
    let tolerance = diagonal.max(distance_from_origin) * 1e-5;

    // Area rather than length, and well under the tolerance squared: this only drops triangles
    // the weld itself collapsed, not thin ones the model meant to have.
    let min_face_area = tolerance * tolerance / 10.0;
    let mut welded = mesh.copy();
    welded.merge_vertices(tolerance, min_face_area, None);
    welded.remove_degenerate_faces(min_face_area);
    welded.remove_unused_vertices();
    Some(welded)
}

/// How far the furthest of a corner's three coordinates is from zero.
fn max_abs_component(corner: Vector3) -> f64 {
    corner.x.abs().max(corner.y.abs().max(corner.z.abs()))
}

/// Flattens a mesh into the interleaved position list and triangle index list the kernel
/// takes. Positions widen from the mesh's f32 to f64 with no narrowing at the boundary; three
/// properties per vertex and nothing else set (no merge vectors, runs or tangents).
pub(crate) fn to_rust_mesh_data(mesh: &Mesh) -> MeshGL64 {
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
