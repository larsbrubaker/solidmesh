//! Getting a [`Mesh`] into the kernel: the robust import, the orientation repair and the flattening
//! into the kernel's `MeshGL64`, which can tell the kernel to work to the mesh's single precision
//! so no result has a sliver f32 cannot hold (`BooleanOptions::single_precision`). Called by
//! `manifold_operand_batch` and `boolean_processing::classify_boolean_operand`.
//!
//! A mesh the kernel calls `NotClosed` gets one retry after `weld_seams` (a tolerance weld);
//! [`import_strict`] is the one-shot import with no retry, for callers that must read exactly
//! what the kernel makes of the mesh as given. `import_with_provenance` is the import for
//! face-colour and face-texture tracking: every triangle goes in tagged with its operand's
//! reserved id and the index of its colour in that operand's palette, which the kernel carries
//! through the boolean. A tagged operand with pictures on its faces also gets each corner's UV as
//! two extra vertex properties, and its face ids index (colour, picture, UV chart) triples
//! instead (`manifold_uv_channel`, `manifold_uv_charts`); an operand without pictures goes in
//! exactly as it does when no operand has textures.

use std::collections::HashMap;

use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelStatus, MeshGL64};
use solidmesh_math::Color;

use solidmesh_math::Vector3;

use super::manifold_kernel;
use super::manifold_uv_channel::{self, ImagePalette};
use crate::Mesh;

/// The kernel refused a mesh.
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
/// gets a second chance through `weld_seams` before it is refused.
///
/// The refusal is returned rather than absorbed: a boolean swallows an error operand as empty
/// geometry and still reports success, which would show up as a part silently missing.
pub fn import(mesh: &Mesh, repair_orientation: bool) -> Result<Manifold, ImportRejected> {
    import_at_precision(mesh, repair_orientation, false)
}

/// [`import`], telling the kernel to work to the mesh's own f32 precision when `single_precision`
/// is set ([`single_precision_tolerance`]) and to its own exact f64 precision when it is not.
pub(crate) fn import_at_precision(
    mesh: &Mesh,
    repair_orientation: bool,
    single_precision: bool,
) -> Result<Manifold, ImportRejected> {
    import_with_retry(mesh, repair_orientation, single_precision, None)
        .map(|(manifold, _)| manifold)
}

/// Which operand a mesh is, for an import whose result triangles must be traceable to it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct OperandTag {
    /// An id reserved from the kernel (`Manifold::reserve_ids`) for this operand alone; the
    /// result's runs name it for every triangle that came from this operand.
    pub original_id: u32,
    /// The colour the whole operand wears when the mesh has no per-face colours of its own.
    pub operand_color: Option<Color>,
    /// Whether the caller asked for colour tracking; without it the operand comes back with no
    /// colour palette and its face colours do not split its face ids.
    pub tracks_colours: bool,
    /// Whether the boolean carries face textures (some operand has a picture on a face). The UV
    /// channel goes only on an operand that itself has one.
    pub carries_textures: bool,
}

/// What a tagged operand's face ids index: its colour palette (`None` when it has no colour, or
/// colours are not tracked) and, when it went in with the UV channel, its picture palette.
#[derive(Default)]
pub(crate) struct OperandLooks {
    pub colours: Option<Vec<Color>>,
    pub pictures: Option<ImagePalette>,
}

/// [`import_at_precision`] with every triangle tagged for colour tracking: the run carries
/// `tag.original_id`, and each triangle's face id is the index of its colour in the palette this
/// returns (the mesh's per-face colours in first-seen order, or the one operand colour). The
/// kernel keeps both through booleans and simplification, so the result can be painted exactly.
/// A textured operand's face ids instead index its (colour, picture, UV chart) triples, and the
/// palettes returned are aligned with them (`manifold_uv_channel::picture_face_ids`).
///
/// The face id is a colour index rather than a triangle index so coplanar neighbours of one
/// colour still merge in simplification; a triangle index would keep every input edge and the
/// mesh would grow at each level of a nested boolean. Edges between non-coplanar faces are kept
/// either way: the kernel only merges triangles that share both face id and plane.
///
/// The palette is `None` when the operand has no colour at all (no per-face colours and no
/// `operand_color`); its triangles are then left unattributed rather than given a colour from
/// nowhere. When the tolerance weld retry runs, the palette is built from the welded mesh,
/// whose per-face colours follow its faces through the weld, so no triangle's colour is ever
/// re-derived from its position.
pub(crate) fn import_with_provenance(
    mesh: &Mesh,
    repair_orientation: bool,
    single_precision: bool,
    tag: OperandTag,
) -> Result<(Manifold, OperandLooks), ImportRejected> {
    import_with_retry(mesh, repair_orientation, single_precision, Some(tag))
}

fn import_with_retry(
    mesh: &Mesh,
    repair_orientation: bool,
    single_precision: bool,
    tag: Option<OperandTag>,
) -> Result<(Manifold, OperandLooks), ImportRejected> {
    manifold_kernel::ensure_configured();

    let rejected = match try_import(mesh, repair_orientation, single_precision, tag) {
        Ok(imported) => return Ok(imported),
        Err(rejected) => rejected,
    };

    if rejected.status == KernelStatus::NotClosed {
        if let Some(welded) = weld_seams(mesh) {
            if let Ok(retried) = try_import(&welded, repair_orientation, single_precision, tag) {
                return Ok(retried);
            }
        }
    }

    Err(rejected)
}

/// Uploads a mesh exactly as given: the kernel's robust import over the same marshalling, with no
/// weld-and-retry and no orientation repair.
///
/// For callers that must see the kernel's own reading of the mesh - the bevel tests that check
/// a built fillet is a solid the kernel takes, where a tolerance weld would close a seam the
/// builder left open and hide it.
pub fn import_strict(mesh: &Mesh) -> Result<Manifold, ImportRejected> {
    manifold_kernel::ensure_configured();
    try_import(mesh, false, false, None).map(|(manifold, _)| manifold)
}

/// Uploads a mesh, handing back the status that explains a failure so [`import`] can decide whether
/// it is worth a retry, and the palettes its face ids index when it was tagged.
fn try_import(
    mesh: &Mesh,
    repair_orientation: bool,
    single_precision: bool,
    tag: Option<OperandTag>,
) -> Result<(Manifold, OperandLooks), ImportRejected> {
    let mut mesh_gl = to_rust_mesh_data(mesh, single_precision);
    let mut palette = OperandLooks::default();
    if let Some(tag) = tag {
        if tag.carries_textures && manifold_uv_channel::has_textures(mesh) {
            let colour_of = |face: usize| {
                if !tag.tracks_colours {
                    return None;
                }
                match &mesh.face_colors {
                    Some(colors) => Some(
                        colors
                            .get(face)
                            .copied()
                            .unwrap_or(Mesh::UNKNOWN_FACE_COLOR),
                    ),
                    None => tag.operand_color,
                }
            };
            let (face_ids, colours, pictures) =
                manifold_uv_channel::picture_face_ids(mesh, colour_of);
            manifold_uv_channel::add_uv_channel(&mut mesh_gl, mesh);
            mesh_gl.face_id = face_ids;
            mesh_gl.run_original_id = vec![tag.original_id];
            palette = OperandLooks {
                colours,
                pictures: Some(pictures),
            };
        } else if tag.tracks_colours {
            if let Some((colors, face_ids)) = face_palette(mesh, tag.operand_color) {
                mesh_gl.face_id = face_ids;
                mesh_gl.run_original_id = vec![tag.original_id];
                palette.colours = Some(colors);
            }
        }
    }
    let imported = Manifold::from_mesh_gl64_robust(&mesh_gl);
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
        return Ok((imported, palette));
    }

    // On the imported manifold rather than the mesh: the repair is a kernel operation over the
    // imported shells. A mesh that needs no repair comes back as a plain copy, so this is safe
    // unconditionally. It only rewinds triangles in place, so their face ids stay with them.
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
    Ok((repaired, palette))
}

/// A mesh's colour palette and, per face, the index of its colour in it: the per-face colours in
/// first-seen order (a face past the end of the list counts as [`Mesh::UNKNOWN_FACE_COLOR`]), or
/// `operand_color` for every face when the mesh has none. `None` when there is no colour at all.
fn face_palette(mesh: &Mesh, operand_color: Option<Color>) -> Option<(Vec<Color>, Vec<u64>)> {
    let Some(face_colors) = &mesh.face_colors else {
        return operand_color.map(|color| (vec![color], vec![0; mesh.faces.len()]));
    };
    let mut palette = Vec::new();
    let mut index_of: HashMap<Color, u64> = HashMap::new();
    let face_ids = (0..mesh.faces.len())
        .map(|face| {
            let color = face_colors
                .get(face)
                .copied()
                .unwrap_or(Mesh::UNKNOWN_FACE_COLOR);
            *index_of.entry(color).or_insert_with(|| {
                palette.push(color);
                palette.len() as u64 - 1
            })
        })
        .collect();
    Some((palette, face_ids))
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
/// properties per vertex and, when `single_precision` is set, the single-precision tolerance
/// ([`single_precision_tolerance`]); nothing else set (no merge vectors, runs or tangents), so
/// without it the kernel works to its own exact f64 precision. A textured operand's UVs are added
/// on top of this by `manifold_uv_channel::add_uv_channel`, after the tolerance is taken from the
/// positions alone.
pub(crate) fn to_rust_mesh_data(mesh: &Mesh, single_precision: bool) -> MeshGL64 {
    let mut vert_properties = Vec::with_capacity(mesh.vertices.len() * 3);
    for vertex in &mesh.vertices {
        vert_properties.push(f64::from(vertex.x));
        vert_properties.push(f64::from(vertex.y));
        vert_properties.push(f64::from(vertex.z));
    }
    let tolerance = if single_precision {
        single_precision_tolerance(&vert_properties)
    } else {
        0.0
    };

    // Face indices are i32; a negative one is out of bounds for the kernel either way, so it is
    // passed as the u64 it wraps to and the import reports VertexOutOfBounds.
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
        tolerance,
        ..MeshGL64::default()
    }
}

/// How finely a mesh's own coordinates can place a point: one f32 step at its largest
/// coordinate (`f32::EPSILON` times the largest absolute coordinate), the precision the kernel
/// is told to work to when a boolean asks for single precision.
///
/// A [`Mesh`] stores f32 positions and every boolean result is narrowed back to f32. Left at the
/// kernel's own f64 precision, a boolean whose operands nearly touch (a thread's root and the core
/// it is unioned onto, both rounded to f32 on their own) cuts slivers whose corners lie a
/// hundred-millionth of a millimetre apart: valid in f64, but narrowed to f32 they become
/// triangles with no area. Told the coordinates are only good to an f32 step, the kernel merges
/// those edges instead of cutting them, and the result has nothing f32 cannot hold. Nothing a
/// user could see moves: the shape changes by under one f32 step.
///
/// It is opt-in because the merging reaches every edge of the result, not only the slivers: a
/// faceted edge whose facets bend by less than the tolerance can lose facets, and an operand
/// built exactly in f64 (a bevel's cutting tool) no longer cuts exactly where it was built.
///
/// Zero for a mesh with no vertices or with a non-finite one, which leaves the kernel at its own
/// precision and its own verdict on the non-finite vertex.
fn single_precision_tolerance(vert_properties: &[f64]) -> f64 {
    let mut scale = 0.0f64;
    for &coordinate in vert_properties {
        if !coordinate.is_finite() {
            return 0.0;
        }
        scale = scale.max(coordinate.abs());
    }
    f64::from(f32::EPSILON) * scale
}
