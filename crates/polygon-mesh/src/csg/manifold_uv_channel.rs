//! Face textures through the kernel: each textured operand's corner UVs ride the boolean as two
//! extra vertex properties, and each of its faces' pictures is traced back through the face id.
//! `manifold_import` adds the channel and the picture face ids to a textured operand's
//! `MeshGL64`, `manifold_operand_batch` records the pictures each operand's face ids index, and
//! `manifold_result_reader` reads the result back through [`build_textured_mesh`].
//!
//! The kernel keeps properties on property vertices: a position vertex where corners disagree
//! (a UV seam, an untextured face beside a textured one) becomes several property vertices, tied
//! back to the one position vertex by the merge vectors so the solid stays closed. The kernel
//! interpolates the properties onto every vertex a cut makes from the face the vertex lies on,
//! and it treats coplanar triangles with one face id as one face. So pictured faces share an id
//! only within one UV chart (`manifold_uv_charts`): faces grown from a seed face whose every
//! corner lies in the seed's plane and within four f32 steps of UV of the seed's own map. A cut
//! face then keeps its mapping to within about twice that, however large the chart.
//!
//! The channel is only added when some operand has a picture on a face: an untextured boolean
//! hands the kernel the same data it always did and gets the same result back, bit for bit.

use std::collections::HashMap;
use std::sync::Arc;

use manifold_rust::types::MeshGL64;
use solidmesh_math::texture::TexturePixels;
use solidmesh_math::{Color, Vector3, Vector3Float};

use super::{manifold_uv_charts, BooleanError};
use crate::{Face, FaceTextureData, Mesh};

/// Position plus u and v.
const TEXTURED_NUM_PROP: usize = 5;

/// Per face id, the picture the faces with that id wear (`None` for an untextured face).
pub(crate) type ImagePalette = Vec<Option<Arc<dyn TexturePixels>>>;

/// What a face id stands for: a face's colour, its picture's address and its UV chart.
type FaceLook = (Option<Color>, Option<usize>, Option<usize>);

/// Whether any face of `mesh` wears a picture.
pub(crate) fn has_textures(mesh: &Mesh) -> bool {
    !mesh.faces.is_empty() && mesh.face_textures.values().any(|t| t.image.is_some())
}

/// The face ids of a textured operand and the palettes they index: one id per distinct
/// (colour, picture, UV chart) a face wears, in first-seen face order. The colour palette is
/// `None` when the operand has no colour at all (`colour_of` gives `None` for every face).
///
/// Pictured faces share an id only within one UV chart (`manifold_uv_charts`): the kernel merges
/// coplanar faces of one id and spreads one triangle's UVs over the merged face, which is exact
/// only where they share one affine map. Untextured faces share an id per colour, as they do
/// without textures.
pub(crate) fn picture_face_ids(
    mesh: &Mesh,
    colour_of: impl Fn(usize) -> Option<Color>,
) -> (Vec<u64>, Option<Vec<Color>>, ImagePalette) {
    let address_of =
        |face: usize| face_image(mesh, face).map(|image| Arc::as_ptr(image) as *const () as usize);
    let charts =
        manifold_uv_charts::uv_charts(mesh, |face| corner_uvs(mesh, face), address_of, &colour_of);
    let mut colours = Vec::new();
    let mut images: ImagePalette = Vec::new();
    let mut any_colour = false;
    // Keyed by the colour, the picture's address and the chart; the palette order is the
    // first-seen order of the faces, so the map's own order never reaches an output.
    let mut id_of: HashMap<FaceLook, u64> = HashMap::new();
    let face_ids = (0..mesh.faces.len())
        .map(|face| {
            let colour = colour_of(face);
            any_colour |= colour.is_some();
            let image = face_image(mesh, face);
            let address = address_of(face);
            *id_of
                .entry((colour, address, charts[face]))
                .or_insert_with(|| {
                    colours.push(colour.unwrap_or(Mesh::UNKNOWN_FACE_COLOR));
                    images.push(image.cloned());
                    images.len() as u64 - 1
                })
        })
        .collect();
    (face_ids, any_colour.then_some(colours), images)
}

fn face_image(mesh: &Mesh, face: usize) -> Option<&Arc<dyn TexturePixels>> {
    // Face indices are i32 in the texture map; a mesh with more faces cannot have textures there.
    let key = i32::try_from(face).ok()?;
    mesh.face_textures.get(&key)?.image.as_ref()
}

/// Rewrites a position-only `MeshGL64` of `mesh` (three properties per vertex, triangles
/// indexing positions) into one carrying each corner's UV.
///
/// Position vertex `i` stays property vertex `i`, holding the UV of the first corner that uses
/// it; every corner whose UV differs from that gets its own property vertex (one per distinct
/// UV at that position, in first-seen corner order) appended after them, with a merge pair back
/// to `i`. An untextured face's corners have UV (0, 0).
pub(crate) fn add_uv_channel(mesh_gl: &mut MeshGL64, mesh: &Mesh) {
    let vertex_count = mesh_gl.vert_properties.len() / 3;
    let mut properties = Vec::with_capacity(vertex_count * TEXTURED_NUM_PROP);
    for position in mesh_gl.vert_properties.as_chunks::<3>().0 {
        properties.extend_from_slice(position);
        properties.extend_from_slice(&[0.0, 0.0]);
    }
    let mut claimed = vec![false; vertex_count];
    let mut split: HashMap<(u64, u64, u64), u64> = HashMap::new();
    let mut merge_from = Vec::new();
    let mut merge_to = Vec::new();

    for (face, corners) in mesh_gl
        .tri_verts
        .as_chunks_mut::<3>()
        .0
        .iter_mut()
        .enumerate()
    {
        let uvs = corner_uvs(mesh, face);
        for (corner, uv) in corners.iter_mut().zip(uvs) {
            let position = *corner;
            // An index past the positions is left as it is so the import refuses it, as it
            // would without the channel.
            let Some(slot) = usize::try_from(position)
                .ok()
                .filter(|&slot| slot < vertex_count)
            else {
                continue;
            };
            if !claimed[slot] {
                claimed[slot] = true;
                properties[slot * TEXTURED_NUM_PROP + 3] = uv[0];
                properties[slot * TEXTURED_NUM_PROP + 4] = uv[1];
                continue;
            }
            let own = &properties[slot * TEXTURED_NUM_PROP + 3..slot * TEXTURED_NUM_PROP + 5];
            if own[0].to_bits() == uv[0].to_bits() && own[1].to_bits() == uv[1].to_bits() {
                continue;
            }
            *corner = *split
                .entry((position, uv[0].to_bits(), uv[1].to_bits()))
                .or_insert_with(|| {
                    let property_vertex = (properties.len() / TEXTURED_NUM_PROP) as u64;
                    let start = slot * TEXTURED_NUM_PROP;
                    let xyz = [
                        properties[start],
                        properties[start + 1],
                        properties[start + 2],
                    ];
                    properties.extend_from_slice(&xyz);
                    properties.extend_from_slice(&uv);
                    merge_from.push(property_vertex);
                    merge_to.push(position);
                    property_vertex
                });
        }
    }

    mesh_gl.num_prop = TEXTURED_NUM_PROP as u64;
    mesh_gl.vert_properties = properties;
    mesh_gl.merge_from_vert = merge_from;
    mesh_gl.merge_to_vert = merge_to;
}

/// The UV of each corner of `face`, as the kernel takes them. A non-finite UV (which maps no
/// pixel) goes in as zero: the kernel refuses a non-finite property, and a stray value in a
/// picture's mapping must not cost the user the whole boolean.
fn corner_uvs(mesh: &Mesh, face: usize) -> [[f64; 2]; 3] {
    let Some(texture) = i32::try_from(face)
        .ok()
        .and_then(|key| mesh.face_textures.get(&key))
        .filter(|texture| texture.image.is_some())
    else {
        return [[0.0, 0.0]; 3];
    };
    let finite = |value: f32| {
        if value.is_finite() {
            f64::from(value)
        } else {
            0.0
        }
    };
    [texture.uv0, texture.uv1, texture.uv2].map(|uv| [finite(uv.x), finite(uv.y)])
}

/// A result that carries the UV channel as a mesh: property vertices welded back onto their
/// position vertices through the merge vectors, and every face from a textured operand wearing
/// its picture with its corners' UVs. Faces whose run names no picture palette, or whose face
/// id names an untextured entry, are left untextured.
pub(crate) fn build_textured_mesh(
    exported: &MeshGL64,
    original_id_to_images: &HashMap<i32, ImagePalette>,
    operation_name: &str,
) -> Result<Mesh, BooleanError> {
    let num_prop = exported.num_prop as usize;
    if num_prop < TEXTURED_NUM_PROP {
        return Err(BooleanError::Kernel {
            message: format!(
                "Manifold {operation_name} result has {num_prop} properties per vertex, expected {TEXTURED_NUM_PROP}"
            ),
        });
    }
    let property_count = exported.vert_properties.len() / num_prop;

    // Each property vertex's position vertex: itself unless a merge pair sends it elsewhere.
    let mut position_of: Vec<usize> = (0..property_count).collect();
    for (&from, &to) in exported
        .merge_from_vert
        .iter()
        .zip(exported.merge_to_vert.iter())
    {
        let (Ok(from), Ok(to)) = (usize::try_from(from), usize::try_from(to)) else {
            continue;
        };
        if from < property_count && to < property_count {
            position_of[from] = to;
        }
    }
    // Position vertices in property order, so the mesh's vertex order follows the kernel's.
    let mut vertex_of = vec![usize::MAX; property_count];
    let mut mesh = Mesh::new();
    for property in 0..property_count {
        if position_of[property] != property {
            continue;
        }
        vertex_of[property] = mesh.vertices.len();
        let at = property * num_prop;
        let xyz = &exported.vert_properties[at..at + 3];
        mesh.vertices.push(Vector3Float::from_vector3(Vector3::new(
            xyz[0], xyz[1], xyz[2],
        )));
    }

    let uv_of = |property: usize| {
        let at = property * num_prop;
        solidmesh_math::Vector2Float::new(
            exported.vert_properties[at + 3],
            exported.vert_properties[at + 4],
        )
    };
    let picture_of = face_pictures(exported, original_id_to_images);

    for (tri, corners) in exported.tri_verts.as_chunks::<3>().0.iter().enumerate() {
        let mut properties = [0usize; 3];
        let mut vertices = [0i32; 3];
        for ((property, vertex), &corner) in properties.iter_mut().zip(&mut vertices).zip(corners) {
            let index = usize::try_from(corner)
                .ok()
                .filter(|&index| index < property_count)
                .ok_or_else(|| BooleanError::Kernel {
                    message: format!(
                        "Manifold {operation_name} result names property vertex {corner} of {property_count}"
                    ),
                })?;
            *property = index;
            let welded = vertex_of[position_of[index]];
            // A merge target that is itself merged elsewhere has no position vertex; the
            // kernel's export never makes one, but a bad index must not wrap silently.
            *vertex = i32::try_from(welded).map_err(|_| BooleanError::Kernel {
                message: format!(
                    "Manifold {operation_name} result merges property vertex {index} onto one with no position"
                ),
            })?;
        }
        let face = Face::new(vertices[0], vertices[1], vertices[2], &mesh.vertices);
        mesh.faces.push(face);
        if let Some(image) = picture_of.get(tri).cloned().flatten() {
            // Result triangle counts fit the kernel's 32-bit indices, so the cast is exact.
            mesh.face_textures.insert(
                tri as i32,
                FaceTextureData::new(
                    Some(image),
                    uv_of(properties[0]),
                    uv_of(properties[1]),
                    uv_of(properties[2]),
                ),
            );
        }
    }
    Ok(mesh)
}

/// Per result triangle, the picture its operand face wore: the run's original id names the
/// operand's picture palette and the triangle's face id the entry in it.
fn face_pictures(
    exported: &MeshGL64,
    original_id_to_images: &HashMap<i32, ImagePalette>,
) -> Vec<Option<Arc<dyn TexturePixels>>> {
    let face_count = exported.tri_verts.len() / 3;
    let mut pictures = vec![None; face_count];
    let run_index = &exported.run_index;
    for (run, &original_id) in exported.run_original_id.iter().enumerate() {
        // Signed on the manifold, unsigned in the run data; the cast reinterprets the bits.
        let Some(palette) = original_id_to_images.get(&(original_id as i32)) else {
            continue;
        };
        let Some(&start) = run_index.get(run) else {
            continue;
        };
        let end = run_index
            .get(run + 1)
            .map_or(face_count, |&end| (end / 3) as usize)
            .min(face_count);
        for (tri, picture) in pictures
            .iter_mut()
            .enumerate()
            .take(end)
            .skip((start / 3) as usize)
        {
            *picture = exported
                .face_id
                .get(tri)
                .and_then(|&id| usize::try_from(id).ok())
                .and_then(|id| palette.get(id))
                .cloned()
                .flatten();
        }
    }
    pictures
}
