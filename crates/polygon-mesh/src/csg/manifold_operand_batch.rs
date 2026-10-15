//! Collecting a boolean's operands: the empty-operand rules, the transformed copy, the import,
//! the per-operand refusals a union degrades around, and the colour bookkeeping the result is
//! painted from, including the re-tag that marks an import as an original. Driven by
//! `manifold_kernel`, read back by `manifold_result_reader`.

use std::collections::HashMap;

use agg_path::Color;
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{Error as KernelStatus, OpType};
use vector_math::Matrix4X4;

use super::{manifold_cancellable_boolean, manifold_import, manifold_kernel_call_counts};
use super::{BooleanError, CsgModes, SkippedBooleanOperand};
use crate::{CancelToken, CentroidColor, Face, Mesh};

/// Whether the batch took an operand, or the operation's answer is already known to be empty.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AddOutcome {
    Added,
    /// An empty operand that makes the whole result empty (any intersect operand, or the
    /// first operand of a subtract).
    ResultIsEmpty,
}

pub(crate) struct ManifoldOperandBatch {
    operation: CsgModes,
    /// The per-operand colours the caller supplied, or `None` when it asked for no colour
    /// tracking at all.
    pub(crate) mesh_colors: Option<Vec<Color>>,
    repair_orientation: bool,
    // A union is the one operation where leaving an operand out still has an answer: the other
    // operands' union. Subtract and Intersect are defined by every operand, so dropping one of
    // those would silently change what the operation means.
    skip_refused_operands: bool,
    pub(crate) manifolds: Vec<Manifold>,
    pub(crate) skipped: Vec<SkippedBooleanOperand>,
    pub(crate) original_id_to_color: HashMap<i32, Color>,
    pub(crate) original_id_to_spatial_colors: HashMap<i32, Vec<CentroidColor>>,
    operand_count: usize,
}

impl ManifoldOperandBatch {
    pub(crate) fn new(
        operation: CsgModes,
        mesh_colors: Option<Vec<Color>>,
        repair_orientation: bool,
    ) -> Self {
        Self {
            operation,
            mesh_colors,
            repair_orientation,
            skip_refused_operands: operation == CsgModes::Union,
            manifolds: Vec::new(),
            skipped: Vec::new(),
            original_id_to_color: HashMap::new(),
            original_id_to_spatial_colors: HashMap::new(),
            operand_count: 0,
        }
    }

    /// Adds one operand.
    pub(crate) fn try_add(
        &mut self,
        mesh: &Mesh,
        matrix: Matrix4X4,
        cancel: &CancelToken,
    ) -> Result<AddOutcome, BooleanError> {
        // Before the copy and the upload, not just before the boolean: importing a large set is
        // itself seconds of work, and an already-cancelled caller should not pay for it.
        if cancel.is_cancelled() {
            return Err(BooleanError::Cancelled);
        }

        let mesh_index = self.operand_count;

        if mesh.vertices.is_empty() || mesh.faces.is_empty() {
            if self.operation == CsgModes::Intersect {
                return Ok(AddOutcome::ResultIsEmpty);
            }
            if mesh_index == 0 && self.operation == CsgModes::Subtract {
                return Ok(AddOutcome::ResultIsEmpty);
            }
            self.operand_count += 1;
            return Ok(AddOutcome::Added);
        }

        let mut mesh_copy = mesh.copy();
        mesh_copy.transform(matrix);

        match self.import_operand(&mesh_copy, mesh_index, cancel) {
            Ok(manifold) => self.manifolds.push(manifold),
            Err(OperandError::Cancelled) => return Err(BooleanError::Cancelled),
            Err(OperandError::Refused(refused)) if self.skip_refused_operands => {
                // Not swallowed: throw_if_nothing_imported or throw_if_any_skipped names every
                // operand that landed here.
                self.skipped.push(SkippedBooleanOperand {
                    index: mesh_index,
                    reason: refused.message,
                });
            }
            Err(OperandError::Refused(refused)) => {
                return Err(BooleanError::ImportRejected {
                    message: refused.message,
                    status: refused.status,
                })
            }
        }

        self.operand_count += 1;
        Ok(AddOutcome::Added)
    }

    /// Uploads one already-transformed operand, recording whatever the colour machinery needs to
    /// paint that operand's triangles in the result.
    fn import_operand(
        &mut self,
        mesh_copy: &Mesh,
        mesh_index: usize,
        cancel: &CancelToken,
    ) -> Result<Manifold, OperandError> {
        let Some(mesh_colors) = &self.mesh_colors else {
            return manifold_import::import(mesh_copy, self.repair_orientation)
                .map_err(OperandError::Refused);
        };

        if mesh_copy.face_colors.is_some() {
            if let Some(split) = try_split_by_face_colors(
                mesh_copy,
                &mut self.original_id_to_color,
                cancel,
                self.repair_orientation,
            )? {
                return Ok(split);
            }
        }

        // as_original is what gives the input an original id, and the run data that carries
        // colours back is keyed on that.
        let manifold = import_as_original(mesh_copy, self.repair_orientation)
            .map_err(OperandError::Refused)?;

        // -1 is what original_id reports when the re-tag did not take (a soup manifold). It is
        // not an id: every such operand would register under the same key. Registering nothing
        // leaves this operand's triangles unattributed, which is what they honestly are.
        let original_id = manifold.original_id();
        if original_id >= 0 {
            if let Some(saved) = mesh_copy.save_face_centroid_colors() {
                self.original_id_to_spatial_colors
                    .insert(original_id, saved);
            } else if mesh_index < mesh_colors.len() {
                // Nothing registered past the end of mesh_colors: the caller supplied no colour
                // for this operand, and inventing one would make its triangles count as
                // attributed and come back wearing a colour from nowhere.
                self.original_id_to_color
                    .insert(original_id, mesh_colors[mesh_index]);
            }
        }

        Ok(manifold)
    }

    /// Fails a union every one of whose operands was refused.
    pub(crate) fn throw_if_nothing_imported(&self) -> Result<(), BooleanError> {
        if !self.skipped.is_empty() && self.manifolds.is_empty() {
            return Err(self.partial(Mesh::new()));
        }
        Ok(())
    }

    /// Passes the result through, or fails with it as the partial result of a union that left
    /// operands out.
    pub(crate) fn throw_if_any_skipped(&self, result: Mesh) -> Result<Mesh, BooleanError> {
        if !self.skipped.is_empty() {
            return Err(self.partial(result));
        }
        Ok(result)
    }

    fn partial(&self, partial_result: Mesh) -> BooleanError {
        BooleanError::Partial {
            message: describe_skipped(&self.skipped, self.operand_count),
            partial_result: Box::new(partial_result),
            skipped_operands: self.skipped.clone(),
        }
    }
}

fn describe_skipped(skipped: &[SkippedBooleanOperand], operand_count: usize) -> String {
    // One-based: these numbers are read by a human against a list of parts.
    let described: Vec<String> = skipped
        .iter()
        .map(|operand| format!("operand {} - {}", operand.index + 1, operand.reason))
        .collect();
    format!(
        "Manifold could not use {} of {} operands: {}",
        skipped.len(),
        operand_count,
        described.join("; ")
    )
}

/// Why one operand did not import: the kernel refused it, or the caller cancelled.
enum OperandError {
    Refused(manifold_import::ImportRejected),
    Cancelled,
}

/// The import re-tagged as an original, so its triangles carry an id of their own through the
/// boolean. A soup manifold (closed but non-manifold input) cannot be re-tagged - `as_original`
/// hands back an errored empty manifold - so the plain import is kept; its triangles then lose
/// their face colours rather than the boolean.
fn import_as_original(
    mesh: &Mesh,
    repair_orientation: bool,
) -> Result<Manifold, manifold_import::ImportRejected> {
    let imported = manifold_import::import(mesh, repair_orientation)?;
    let as_original = imported.as_original();
    if as_original.status() != KernelStatus::NoError {
        return Ok(imported);
    }
    Ok(as_original)
}

/// Splits a mesh with face colors into one sub-manifold per color group and unions them, so each
/// group's original id carries its colour through the boolean. `Ok(None)` when any color group is
/// not a valid manifold on its own (e.g. a boolean result whose color groups share boundaries) or
/// anything else fails; cancellation is the one failure passed on.
fn try_split_by_face_colors(
    mesh_copy: &Mesh,
    original_id_to_color: &mut HashMap<i32, Color>,
    cancel: &CancelToken,
    repair_orientation: bool,
) -> Result<Option<Manifold>, OperandError> {
    let Some(face_colors) = &mesh_copy.face_colors else {
        return Ok(None);
    };

    // Groups in first-seen order, so the result's colour runs are deterministic.
    let mut group_of_color: HashMap<Color, usize> = HashMap::new();
    let mut color_groups: Vec<(Color, Vec<usize>)> = Vec::new();
    for face_idx in 0..mesh_copy.faces.len() {
        let face_color = face_colors
            .get(face_idx)
            .copied()
            .unwrap_or(Mesh::UNKNOWN_FACE_COLOR);
        let group = *group_of_color.entry(face_color).or_insert_with(|| {
            color_groups.push((face_color, Vec::new()));
            color_groups.len() - 1
        });
        color_groups[group].1.push(face_idx);
    }

    let mut sub_manifolds: Vec<Manifold> = Vec::new();
    for (color, face_indices) in &color_groups {
        // One sub-mesh build and one import per colour group, so this loop is worth
        // interrupting on its own rather than only at the union below.
        if cancel.is_cancelled() {
            return Err(OperandError::Cancelled);
        }

        let mut sub_mesh = Mesh::new();
        let mut vertex_map: HashMap<i32, i32> = HashMap::new();
        for &face_idx in face_indices {
            let face = mesh_copy.faces[face_idx];
            let mut get_or_add_vertex = |orig_idx: i32| -> i32 {
                *vertex_map.entry(orig_idx).or_insert_with(|| {
                    sub_mesh
                        .vertices
                        .push(mesh_copy.vertices[orig_idx as usize]);
                    sub_mesh.vertices.len() as i32 - 1
                })
            };
            let (a, b, c) = (
                get_or_add_vertex(face.v0),
                get_or_add_vertex(face.v1),
                get_or_add_vertex(face.v2),
            );
            let new_face = Face::new(a, b, c, &sub_mesh.vertices);
            sub_mesh.faces.push(new_face);
        }

        // Check if sub-mesh is manifold before trying to create a Manifold
        if !sub_mesh.is_manifold() {
            return Ok(None);
        }

        let Ok(sub_manifold) = import_as_original(&sub_mesh, repair_orientation) else {
            return Ok(None);
        };
        // Same -1 guard as import_operand.
        if sub_manifold.original_id() >= 0 {
            original_id_to_color.insert(sub_manifold.original_id(), *color);
        }
        sub_manifolds.push(sub_manifold);
    }

    match sub_manifolds.len() {
        0 => Ok(None),
        1 => Ok(sub_manifolds.pop()),
        // Not as_original: the union has to keep the sub-manifolds' boolean history, because
        // that history is what carries each colour group's original id into the run data of
        // whatever this is later combined with.
        _ => {
            manifold_kernel_call_counts::count_color_group_union();
            match manifold_cancellable_boolean::batch_boolean(
                &sub_manifolds,
                OpType::Add,
                None,
                cancel,
            ) {
                Ok(union) => Ok(Some(union)),
                Err(BooleanError::Cancelled) => Err(OperandError::Cancelled),
                Err(_) => Ok(None),
            }
        }
    }
}
