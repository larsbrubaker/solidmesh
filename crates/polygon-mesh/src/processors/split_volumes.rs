//! Splitting a mesh into its separate volumes ([`split_volumes_into_meshes`]): faces are grouped
//! by walking the faces that share a vertex, so two volumes that only touch at a vertex come out
//! as one. MatterCAD's Ungroup runs it on a lone mesh.

use std::collections::HashSet;

use crate::{CancelToken, Mesh};

/// One mesh per connected volume of `mesh_to_split`, in the order of each volume's first face,
/// dropping slivers (a volume with fewer than three vertices, or no extent or face area above a
/// ten-thousandth of the whole mesh's bounds diagonal). `None` when cancelled.
/// `report_progress` is told the share of faces walked (0..1) after each face.
pub fn split_volumes_into_meshes(
    mesh_to_split: &Mesh,
    cancel: &CancelToken,
    report_progress: Option<&dyn Fn(f64)>,
) -> Option<Vec<Mesh>> {
    let mut discreet_volumes = Vec::new();
    let mut faces_that_have_been_added: HashSet<usize> = HashSet::new();
    let mut attached_faces: Vec<usize> = Vec::new();
    let face_count = mesh_to_split.faces.len();

    let total_bounds = mesh_to_split.get_axis_aligned_bounding_box();

    let faces_sharing_vertex = mesh_to_split.get_vertex_face_lists();

    for face_index in 0..face_count {
        // The cancel poll is every face, which is cheap; progress is reported at the end of each
        // face.
        if cancel.is_cancelled() {
            return None;
        }

        // If this face as not been added to any volume, create a new volume and add all of the
        // attached faces.
        if !faces_that_have_been_added.contains(&face_index) {
            attached_faces.push(face_index);
            let mut mesh_from_current_volume = Mesh::new();

            while let Some(face) = attached_faces.pop() {
                let face_to_add = &mesh_to_split.faces[face];
                let vertices = [face_to_add.v0, face_to_add.v1, face_to_add.v2];

                for attached_vertex in vertices {
                    for &shared in faces_sharing_vertex[attached_vertex as usize].faces() {
                        let shared_face_index = shared as usize;
                        // mark that this face has been taken care of
                        if faces_that_have_been_added.insert(shared_face_index) {
                            // add it to the list of faces we need to walk
                            attached_faces.push(shared_face_index);

                            // Add a new face to the new mesh we are creating.
                            let f = &mesh_to_split.faces[shared_face_index];
                            mesh_from_current_volume.create_face_f32(&[
                                mesh_to_split.vertices[f.v0 as usize],
                                mesh_to_split.vertices[f.v1 as usize],
                                mesh_to_split.vertices[f.v2 as usize],
                            ]);
                        }
                    }
                }
            }

            mesh_from_current_volume.clean_and_merge();
            let bounds = mesh_from_current_volume.get_axis_aligned_bounding_box();
            let one_ten_thousandth = total_bounds.size().length() / 10000.0;
            if mesh_from_current_volume.vertices.len() > 2
                && (bounds.x_size() > one_ten_thousandth
                    || bounds.y_size() > one_ten_thousandth
                    || bounds.z_size() > one_ten_thousandth)
                && mesh_from_current_volume
                    .faces
                    .iter()
                    .any(|f| f.get_area(&mesh_from_current_volume) > one_ten_thousandth)
            {
                discreet_volumes.push(mesh_from_current_volume);
            }
        }

        if let Some(report) = report_progress {
            report(face_index as f64 / face_count as f64);
        }
    }

    Some(discreet_volumes)
}
