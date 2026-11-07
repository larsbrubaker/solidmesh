//! Runs the shadow clips the rebuild planned: each cutter less the columns some other feature
//! standing on its bevelled face has a claim on ([`super::bevel_shadow_clip`]), with the retry that
//! gives the obstruction's BODY up when the kernel refuses it ([`super::bevel_shadow_body`]).
//!
//! The apply half of the rebuild's three cutter corrections, alongside the pair-corner trim and
//! [`super::bevel_cap_clip::apply`]. It runs FIRST of the three because it replaces cutters in
//! place and never withdraws one, so it leaves every index written down while the tools were built
//! valid for the two that follow - the cap clip, which does renumber, has to be last for that
//! reason. Called from the cut stage (`BevelCutStage`).

use clipper_library::ClipperError;
use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Matrix4X4;

use super::bevel_bead_clip::CombineMeshes;
use super::bevel_shadow_clip::{self, Obstruction};
use super::{bevel_shadow_body, bevel_working_mesh, BevelRebuildAdvisories};

/// One planned clip: which cutter a group's columns belong to.
#[derive(Debug, Clone, Default)]
pub struct PlannedShadowClip {
    /// The cutter in the batch the columns come off.
    pub cutter_index: i32,
    /// The group whose obstructions the columns were built from.
    pub group_index: i32,
    /// The columns (and bodies) to subtract.
    pub columns: Vec<Mesh>,
}

/// What stops a shadow clip run: the combine's error (a cancel among them), or the column rebuild's
/// Clipper error.
#[derive(Debug)]
pub enum ShadowClipApplyError {
    /// The rebuild's combine failed or was cancelled.
    Boolean(BooleanError),
    /// Rebuilding the columns for the retry failed in Clipper.
    Clipper(ClipperError),
}

impl From<BooleanError> for ShadowClipApplyError {
    fn from(error: BooleanError) -> Self {
        ShadowClipApplyError::Boolean(error)
    }
}

impl From<ClipperError> for ShadowClipApplyError {
    fn from(error: ClipperError) -> Self {
        ShadowClipApplyError::Clipper(error)
    }
}

/// The cutter followed by each column, none of them carrying a matrix.
fn operands(cutter: &Mesh, columns: &[Mesh]) -> Vec<(Mesh, Matrix4X4)> {
    std::iter::once((cutter.clone(), Matrix4X4::IDENTITY))
        .chain(
            columns
                .iter()
                .map(|column| (column.clone(), Matrix4X4::IDENTITY)),
        )
        .collect()
}

/// Cuts each planned clip's columns out of its cutter, replacing the cutter in place where the
/// result is usable and saying so on the advisories where it is not.
///
/// `cutters` is the cutter batch, in the source item's own space; clipped entries are replaced.
/// `working_mesh` and `working_mesh_graph` are what the columns were built on, for the body-less
/// retry; `shadow_body_count_by_group` says how many of each group's columns are BODIES rather
/// than prisms, and `standing_on_a_face_by_group` what they were built from (`None` where there is
/// nothing). `combine` is the rebuild's batched boolean, so these steps land on the same progress
/// bar.
#[allow(clippy::too_many_arguments)]
pub fn apply(
    cutters: &mut [Mesh],
    clips: &[PlannedShadowClip],
    working_mesh: &Mesh,
    working_mesh_graph: &MeshEdgeGraph,
    shadow_body_count_by_group: &[i32],
    standing_on_a_face_by_group: &[Option<Vec<Obstruction>>],
    advisories: &mut BevelRebuildAdvisories,
    combine: &CombineMeshes<'_>,
) -> Result<(), ShadowClipApplyError> {
    for clip in clips {
        // Both operands carry no matrix, which is what keeps the answer in the space the batch is
        // in: the cutter and the columns are both built on the working mesh, in the source item's
        // own local space, and a combine hands its result back in the space its operands were
        // given in. Passing itemMatrix here would put a WORLD space mesh back into a list every
        // later stage still multiplies by itemMatrix, and the cut lands somewhere else entirely.
        let cutter_index = clip.cutter_index as usize;
        let group_index = clip.group_index as usize;
        let cutter = cutters[cutter_index].clone();

        let mut clipped = Some(combine(
            &operands(&cutter, &clip.columns),
            CsgModes::Subtract,
        )?);

        // A slightly wider clearance can resolve a facet-scale coincidence without losing the
        // feature's shoulders. Keep the smaller clearance whenever it already works.
        if let Some(standing_on_a_face) = &standing_on_a_face_by_group[group_index] {
            if shadow_body_count_by_group[group_index] > 0
                && bevel_shadow_body::the_body_should_be_given_up(&cutter, clipped.as_ref())
            {
                let (retry_columns, retry_bodies) = bevel_shadow_clip::build(
                    working_mesh,
                    working_mesh_graph,
                    standing_on_a_face,
                    true,
                    3.0,
                )?;
                if retry_bodies > 0 {
                    clipped = Some(combine(
                        &operands(&cutter, &retry_columns),
                        CsgModes::Subtract,
                    )?);
                }
            }

            // If the fuller region still fails, retain the footprint prisms and report the loss.
            if shadow_body_count_by_group[group_index] > 0
                && bevel_shadow_body::the_body_should_be_given_up(&cutter, clipped.as_ref())
            {
                let (prisms, _) = bevel_shadow_clip::build(
                    working_mesh,
                    working_mesh_graph,
                    standing_on_a_face,
                    false,
                    1.0,
                )?;

                clipped = if prisms.is_empty() {
                    None
                } else {
                    Some(combine(&operands(&cutter, &prisms), CsgModes::Subtract)?)
                };

                // Not the same thing as a refused clip, and it has its own words to say so.
                advisories.shadow_body_was_given_up();
            }
        }

        // A clip the kernel will not build leaves the cutter as it was, which is what this group
        // did before the clip existed: the obstruction is cut through. That is a worse answer but
        // not a broken one, and refusing the group instead would lose a fillet every other kernel
        // builds - so it is said out loud rather than paid for in geometry.
        match clipped {
            Some(clipped)
                if !clipped.vertices.is_empty()
                    && bevel_working_mesh::is_usable_operand(Some(&clipped), None) =>
            {
                cutters[cutter_index] = clipped;
            }
            _ => advisories.shadow_clip_was_refused(),
        }
    }

    Ok(())
}
