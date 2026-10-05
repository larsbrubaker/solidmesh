//! Reading a kernel result back out as a [`Mesh`]. Port of agg-sharp
//! `Csg/ManifoldResultReader.cs` (`ReadResult`, `ToMesh`, `BuildMesh`); called by
//! `manifold_kernel`. The face-colour half (`ExtractFaceColorsFromRuns`) comes with face
//! colours in Phase 2f.

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelStatus, MeshGL64};
use vector_math::{Vector3, Vector3Float};

use super::BooleanError;
use crate::{Face, Mesh};

/// A result as a mesh, or the error its status names (`operation_name` goes in the message).
pub(crate) fn to_mesh(result: &Manifold, operation_name: &str) -> Result<Mesh, BooleanError> {
    if result.status() != KernelStatus::NoError {
        return Err(BooleanError::Kernel {
            message: format!(
                "Manifold {operation_name} result has error status: {:?}",
                result.status()
            ),
        });
    }
    // -1 is the export's "no property slot holds normals".
    build_mesh(&result.get_mesh_gl64(-1), operation_name)
}

fn build_mesh(exported: &MeshGL64, operation_name: &str) -> Result<Mesh, BooleanError> {
    let num_prop = exported.num_prop as usize;
    // The export promises at least x, y, z per vertex. Checked rather than assumed because
    // num_prop is the loop stride below, and a zero would spin.
    if num_prop < 3 {
        return Err(BooleanError::Kernel {
            message: format!(
                "Manifold {operation_name} result has {num_prop} properties per vertex, expected at least 3"
            ),
        });
    }

    let mut mesh = Mesh::new();
    let vertices = &exported.vert_properties;
    let mut i = 0;
    while i + 2 < vertices.len() {
        mesh.vertices.push(Vector3Float::from_vector3(Vector3::new(
            vertices[i],
            vertices[i + 1],
            vertices[i + 2],
        )));
        i += num_prop;
    }

    for tri in exported.tri_verts.chunks_exact(3) {
        let face = Face::new(tri[0] as i32, tri[1] as i32, tri[2] as i32, &mesh.vertices);
        mesh.faces.push(face);
    }

    Ok(mesh)
}
