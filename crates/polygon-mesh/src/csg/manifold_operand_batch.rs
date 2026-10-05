//! Collecting a boolean's operands: the empty-operand rules, the transformed copy, the import,
//! and the per-operand refusals a union degrades around. Port of agg-sharp
//! `Csg/ManifoldOperandBatch.cs` without its face-colour bookkeeping (Phase 2f); driven by
//! `manifold_kernel`.

use manifold_rust::manifold::Manifold;
use vector_math::Matrix4X4;

use super::manifold_import;
use super::{BooleanError, CsgModes, SkippedBooleanOperand};
use crate::{CancelToken, Mesh};

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
    repair_orientation: bool,
    // A union is the one operation where leaving an operand out still has an answer: the other
    // operands' union. Subtract and Intersect are defined by every operand, so dropping one of
    // those would silently change what the operation means.
    skip_refused_operands: bool,
    pub(crate) manifolds: Vec<Manifold>,
    pub(crate) skipped: Vec<SkippedBooleanOperand>,
    operand_count: usize,
}

impl ManifoldOperandBatch {
    pub(crate) fn new(operation: CsgModes, repair_orientation: bool) -> Self {
        Self {
            operation,
            repair_orientation,
            skip_refused_operands: operation == CsgModes::Union,
            manifolds: Vec::new(),
            skipped: Vec::new(),
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

        match manifold_import::import(&mesh_copy, self.repair_orientation) {
            Ok(manifold) => self.manifolds.push(manifold),
            Err(refused) if self.skip_refused_operands => {
                // Not swallowed: throw_if_nothing_imported or throw_if_any_skipped names every
                // operand that landed here.
                self.skipped.push(SkippedBooleanOperand {
                    index: mesh_index,
                    reason: refused.message,
                });
            }
            Err(refused) => {
                return Err(BooleanError::ImportRejected {
                    message: refused.message,
                    status: refused.status,
                })
            }
        }

        self.operand_count += 1;
        Ok(AddOutcome::Added)
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
