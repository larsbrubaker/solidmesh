//! Reading a kernel result back out as a [`Mesh`], with per-face colours read exactly from the
//! result's runs and face ids; called by `manifold_kernel`, with the palettes
//! `manifold_operand_batch` recorded. A result carrying the UV channel is welded and textured by
//! `manifold_uv_channel::build_textured_mesh`; its face colours are read the same way.

use std::collections::HashMap;

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelStatus, MeshGL64};
use solidmesh_math::Color;
use solidmesh_math::{Vector3, Vector3Float};

use super::manifold_operand_batch::ManifoldOperandBatch;
use super::manifold_uv_channel;
use super::BooleanError;
use crate::{Face, Mesh};

/// A finished boolean as a mesh, its faces painted from the run data when the caller asked for
/// colour tracking.
pub(crate) fn read_result(
    bool_result: &Manifold,
    batch: &ManifoldOperandBatch,
) -> Result<Mesh, BooleanError> {
    throw_if_errored(bool_result, "boolean")?;
    let exported = bool_result.get_mesh_gl64(-1);
    // The channel is there only when a textured operand went in with it; a batch that carries
    // textures but whose textured operands all dropped out reads back as an untextured one.
    let mut result_mesh = if batch.carries_textures && exported.num_prop > 3 {
        manifold_uv_channel::build_textured_mesh(
            &exported,
            &batch.original_id_to_images,
            "boolean",
        )?
    } else {
        build_mesh(&exported, "boolean")?
    };
    if let Some(mesh_colors) = &batch.mesh_colors {
        if !result_mesh.faces.is_empty() {
            result_mesh.face_colors = extract_face_colors_from_runs(
                &exported,
                &result_mesh,
                &batch.original_id_to_palette,
                mesh_colors,
            );
        }
    }
    Ok(result_mesh)
}

fn throw_if_errored(result: &Manifold, operation_name: &str) -> Result<(), BooleanError> {
    if result.status() != KernelStatus::NoError {
        return Err(BooleanError::Kernel {
            message: format!(
                "Manifold {operation_name} result has error status: {:?}",
                result.status()
            ),
        });
    }
    Ok(())
}

/// A result as a mesh, or the error its status names (`operation_name` goes in the message).
pub fn to_mesh(result: &Manifold, operation_name: &str) -> Result<Mesh, BooleanError> {
    throw_if_errored(result, operation_name)?;
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

    for tri in exported.tri_verts.as_chunks::<3>().0 {
        let face = Face::new(tri[0] as i32, tri[1] as i32, tri[2] as i32, &mesh.vertices);
        mesh.faces.push(face);
    }

    Ok(mesh)
}

/// Per-face colors from a boolean result's run data.
///
/// Each run is a contiguous span of result triangles that came from one source mesh; the run's
/// original id says which, and each triangle's face id is the index, in that operand's palette,
/// of the colour of the input face it lies on. The kernel carries both through the boolean, so
/// every attributed triangle wears exactly the colour of the face it came from.
///
/// Both of the kernel's engines carry these through: the exact one and the robust one `Auto`
/// picks for non-manifold or self-intersecting operands. Not every run can be traced back to a
/// palette, though: an operand the caller supplied no colour for is imported untagged, so its
/// triangles come back under an id no palette is keyed by. Such a run is never painted
/// [`Mesh::UNKNOWN_FACE_COLOR`], a colour nothing in the scene is wearing. When nothing at all could be attributed this returns `None`, so the
/// object's own colour shows; when only some runs are unattributed they take the first
/// operand's colour - the base being cut or unioned into. A face id outside its operand's
/// palette is treated the same way.
fn extract_face_colors_from_runs(
    result_mesh_gl: &MeshGL64,
    result_mesh: &Mesh,
    original_id_to_palette: &HashMap<i32, Vec<Color>>,
    mesh_colors: &[Color],
) -> Option<Vec<Color>> {
    let face_count = result_mesh.faces.len();
    let run_index = &result_mesh_gl.run_index;
    let run_original_id = &result_mesh_gl.run_original_id;
    let face_id = &result_mesh_gl.face_id;

    // run_index carries a trailing end sentinel, so a usable one is at least a start and an end
    // for a single run.
    if run_index.len() < 2 || run_original_id.is_empty() {
        return None;
    }

    let mut face_colors = vec![Color::default(); face_count];
    let mut any_run_attributed = false;
    let unattributed_color = mesh_colors
        .first()
        .copied()
        .unwrap_or(Mesh::UNKNOWN_FACE_COLOR);

    for (run_idx, &original_id) in run_original_id.iter().enumerate() {
        let start_tri = (run_index[run_idx] / 3) as usize;
        let end_tri = if run_idx + 1 < run_index.len() {
            (run_index[run_idx + 1] / 3) as usize
        } else {
            face_count
        };
        let end_tri = end_tri.min(face_count);
        // Signed on the manifold, unsigned in the run data; the cast reinterprets the bits.
        let palette = original_id_to_palette.get(&(original_id as i32));
        if palette.is_some() {
            any_run_attributed = true;
        }
        for (tri, slot) in face_colors
            .iter_mut()
            .enumerate()
            .take(end_tri)
            .skip(start_tri)
        {
            *slot = palette
                .zip(face_id.get(tri))
                .and_then(|(palette, &index)| {
                    usize::try_from(index).ok().and_then(|i| palette.get(i))
                })
                .copied()
                .unwrap_or(unattributed_color);
        }
    }

    any_run_attributed.then_some(face_colors)
}
