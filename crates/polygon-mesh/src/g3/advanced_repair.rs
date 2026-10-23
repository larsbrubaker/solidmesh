//! The geometry of the Repair operation's Advanced path, exposed as
//! [`Mesh::advanced_repaired`]: an optional vertex weld on the source mesh, then over a `DMesh3`
//! the face orientation (the kernel's shell-level repair, `csg::mesh_repair_rust`, falling back to
//! g3's `MeshRepairOrientation`), interior or occluded removal (`remove_occluded_triangles`), crack
//! repair and hole filling (the helpers `mesh_auto_repair` shares with the default path), with the
//! retry and erosion rounds and the final cleanup. Face colors are carried across by face centroid,
//! because the round trip through `DMesh3` renumbers faces.
//!
//! [`Mesh::advanced_repaired_with_g3_orientation`] is the same pipeline as the obsolete Repair
//! operation runs it, for designs saved with that object: every orientation pass is g3's
//! `MeshRepairOrientation`, and the face colors are not carried across.
//!
//! The repeat-everything and finish-early jumps are a labeled loop and its `break`. The
//! cancellation token is read between phases, and a cancel hands back the mesh as it stood before
//! the `DMesh3` work (the welded copy, or the source). No progress messages are reported: nothing
//! in the app shows a sub-step of a repair.

use super::dmesh3::{DMesh3, G3Exception};
use super::mesh_auto_repair::{
    collapse_all_degenerate_edges, fill_any_holes, fill_trivial_holes, repair_cracks,
    repair_orientation, MeshAutoRepairError,
};
use super::mesh_editor::{disconnect_all_bowties, remove_isolated_triangles, remove_triangles};
use super::mesh_face_selection::MeshFaceSelection;
use super::mesh_iterators::boundary_edges;
use super::remove_occluded_triangles::{CalculationMode, RemoveOccludedTriangles};
use super::vector3d::ZERO_TOLERANCEF;
use crate::csg::mesh_repair_rust::try_repair_orientation;
use crate::{CancelToken, Mesh};

/// g3 `MeshAutoRepair.RemoveModes`: which hidden geometry the Advanced path removes.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum RemoveModes {
    #[default]
    None,
    /// Remove triangles inside the mesh by its fast winding number.
    Interior,
    /// Remove triangles that a ray along every axis direction cannot escape from.
    Occluded,
}

/// The Advanced options of the Repair operation, with its defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdvancedRepairOptions {
    /// Align and merge any vertices that are nearly coincident.
    pub weld_vertices: bool,
    /// How distant a vertex must be to weld; 0 welds with `CleanAndMerge`.
    pub weld_tolerance: f64,
    /// Make all the faces have a consistent orientation.
    pub face_orientation: bool,
    /// Repair any small cracks or bad seams in the model.
    pub weld_edges: bool,
    /// Try to fill in any holes that are in the model.
    pub fill_holes: bool,
    /// Remove interior faces and bodies.
    pub remove_mode: RemoveModes,
}

impl Default for AdvancedRepairOptions {
    fn default() -> Self {
        Self {
            weld_vertices: true,
            weld_tolerance: 0.0,
            face_orientation: false,
            weld_edges: false,
            fill_holes: false,
            remove_mode: RemoveModes::None,
        }
    }
}

/// Why the pipeline stopped early: a cancel (caught: the mesh before the `DMesh3` work comes back)
/// or a g3 failure that is passed on.
enum Stop {
    Cancelled,
    Failed(G3Exception),
}

impl From<G3Exception> for Stop {
    fn from(e: G3Exception) -> Self {
        Stop::Failed(e)
    }
}

fn check(cancel: &CancelToken) -> Result<(), Stop> {
    if cancel.is_cancelled() {
        Err(Stop::Cancelled)
    } else {
        Ok(())
    }
}

/// How the pipeline orients faces: the Repair operation's kernel-first passes, or the obsolete
/// Repair operation's g3 flood-fill passes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Orientation {
    PreferKernel,
    Geometry3Sharp,
}

/// What the pipeline hands back: the welded mesh as is (the weld-only early return), or the
/// repaired `DMesh3`, which loses face colors and gets them restored.
enum Outcome {
    Welded(Mesh),
    Repaired(DMesh3),
}

impl Mesh {
    /// The Repair operation with `Advanced = true`. `Err` is a g3 failure (from the hole fillers,
    /// or an edge collapse over a mesh with no edges); a cancel returns the mesh as it stood before
    /// the repair work (the welded copy, or a copy of `self`).
    pub fn advanced_repaired(
        &self,
        options: &AdvancedRepairOptions,
        cancel: &CancelToken,
    ) -> Result<Mesh, MeshAutoRepairError> {
        // Save per-face colors before DMesh3 conversion (which loses them)
        let saved_face_colors = self.save_face_centroid_colors();
        let saved_textures = self.save_face_corner_textures();

        let mut in_mesh: Option<Mesh> = None;
        match run(
            self,
            options,
            Orientation::PreferKernel,
            cancel,
            &mut in_mesh,
        ) {
            Ok(Outcome::Welded(mesh)) => Ok(mesh),
            Ok(Outcome::Repaired(dmesh3)) => {
                let mut result = dmesh3.to_mesh();
                result.restore_face_colors_from_centroids(saved_face_colors.as_deref());
                result.restore_face_textures_from_corners(&saved_textures);
                Ok(result)
            }
            Err(Stop::Cancelled) => Ok(in_mesh.unwrap_or_else(|| self.copy())),
            Err(Stop::Failed(e)) => Err(MeshAutoRepairError(e.0)),
        }
    }

    /// The obsolete Repair operation: the Advanced pipeline with g3's `MeshRepairOrientation`
    /// (components, then the global sign) for the FaceOrientation step and every global pass, and
    /// the result converted straight back, without the face colors (the obsolete operation never
    /// saved them). `Err` and a cancel are as [`Self::advanced_repaired`].
    pub fn advanced_repaired_with_g3_orientation(
        &self,
        options: &AdvancedRepairOptions,
        cancel: &CancelToken,
    ) -> Result<Mesh, MeshAutoRepairError> {
        let mut in_mesh: Option<Mesh> = None;
        match run(
            self,
            options,
            Orientation::Geometry3Sharp,
            cancel,
            &mut in_mesh,
        ) {
            Ok(Outcome::Welded(mesh)) => Ok(mesh),
            Ok(Outcome::Repaired(dmesh3)) => Ok(dmesh3.to_mesh()),
            Err(Stop::Cancelled) => Ok(in_mesh.unwrap_or_else(|| self.copy())),
            Err(Stop::Failed(e)) => Err(MeshAutoRepairError(e.0)),
        }
    }
}

/// The Advanced branch of the repair. `in_mesh` is the source mesh until the weld replaces it.
fn run(
    source: &Mesh,
    options: &AdvancedRepairOptions,
    orientation: Orientation,
    cancel: &CancelToken,
    in_mesh: &mut Option<Mesh>,
) -> Result<Outcome, Stop> {
    if options.weld_vertices {
        check(cancel)?;
        let mut welded = source.copy();
        if options.weld_tolerance > 0.0 {
            welded.merge_vertices(options.weld_tolerance, 0.001, None);
        } else {
            welded.clean_and_merge();
        }

        if !options.face_orientation
            && options.remove_mode == RemoveModes::None
            && !options.weld_edges
            && !options.fill_holes
        {
            return Ok(Outcome::Welded(welded));
        }
        *in_mesh = Some(welded);
    }

    // After the weld, so the welded geometry is what the rest of the pipeline repairs and
    // returns. When welding is off, inMesh is still the source mesh.
    let mut dmesh3 = in_mesh.as_ref().unwrap_or(source).to_dmesh3();

    let mut repeat_count = 0;
    let erosion_iterations = 5;
    let repair_tolerance = ZERO_TOLERANCEF;
    let min_edge_length_tol = 0.0001;

    'repeat_all: loop {
        if options.face_orientation {
            // Shell by shell, with exact containment queries, so nested shells come out right: the
            // outside wound outward and cavities wound inward. Falls back to the g3 flood fill for
            // anything the kernel cannot take.
            dmesh3 = if orientation == Orientation::PreferKernel {
                repair_orientation_prefer_kernel(dmesh3, cancel)?
            } else {
                // make sure orientation of connected components is consistent
                check(cancel)?;
                repair_orientation(&mut dmesh3, true);
                dmesh3
            };
        }

        if options.remove_mode != RemoveModes::None {
            // Remove parts of the mesh we don't want before we bother with anything else
            remove_inside(&mut dmesh3, options.remove_mode);
            check(cancel)?;
        }

        if options.weld_edges || options.fill_holes {
            // Do safe close-cracks to handle easy cases
            repair_cracks(&mut dmesh3, true, repair_tolerance);
            if dmesh3.is_closed() {
                break 'repeat_all;
            }
            check(cancel)?;

            // Collapse tiny edges and then try easy cases again, and
            // then allow for handling of ambiguous cases
            collapse_all_degenerate_edges(&mut dmesh3, repair_tolerance * 0.5, true)?;
            check(cancel)?;
            repair_cracks(&mut dmesh3, true, 2.0 * repair_tolerance);
            check(cancel)?;
            repair_cracks(&mut dmesh3, false, 2.0 * repair_tolerance);
            check(cancel)?;
            if dmesh3.is_closed() {
                break 'repeat_all;
            }

            // Possibly we have joined regions with different orientation (is it?), fix that
            // TODO: mobius strips again
            dmesh3 =
                global_orientation_pass(dmesh3, options.face_orientation, orientation, cancel)?;
            check(cancel)?;

            // get rid of any remaining single-triangles before we start filling holes
            remove_isolated_triangles(&mut dmesh3);
        }

        if options.fill_holes {
            // Ok, fill simple holes.
            fill_trivial_holes(&mut dmesh3)?;
            check(cancel)?;
            if dmesh3.is_closed() {
                break 'repeat_all;
            }

            // Now fill harder holes. If we saw spans, that means boundary loops could
            // not be resolved in some cases, do we disconnect bowties and try again.
            let saw_spans = fill_any_holes(&mut dmesh3)?;
            check(cancel)?;
            if saw_spans {
                disconnect_all_bowties(&mut dmesh3, 10);
                fill_any_holes(&mut dmesh3)?;
            }
            check(cancel)?;
            if dmesh3.is_closed() {
                break 'repeat_all;
            }

            // We may have a closed mesh now but it might still have bowties (eg
            // tetrahedra sharing vtx case). So disconnect those.
            disconnect_all_bowties(&mut dmesh3, 10);
            check(cancel)?;

            // If the mesh is not closed, we will do one more round to try again.
            if repeat_count == 0 && !dmesh3.is_closed() {
                repeat_count += 1;
                continue 'repeat_all;
            }

            // Ok, we didn't get anywhere on our first repeat. If we are still not
            // closed, we will try deleting boundary triangles and repeating.
            // Repeat this N times.
            if repeat_count <= erosion_iterations && !dmesh3.is_closed() {
                repeat_count += 1;
                let mut bdry_faces = MeshFaceSelection::new();
                for eid in boundary_edges(&dmesh3) {
                    bdry_faces.select_edge_tris(&dmesh3, eid);
                }
                let tris: Vec<i32> = bdry_faces.iter().collect();
                remove_triangles(&mut dmesh3, tris, true);
                continue 'repeat_all;
            }
        }
        break 'repeat_all;
    }

    // all_done: and do a final clean up of the model
    if options.fill_holes {
        // Remove tiny edges
        if min_edge_length_tol > 0.0 {
            collapse_all_degenerate_edges(&mut dmesh3, min_edge_length_tol, false)?;
        }
        check(cancel)?;

        // finally do global orientation - this is the best moment for the kernel, since hole
        // filling has just made the mesh as close to closed as it gets
        dmesh3 = global_orientation_pass(dmesh3, options.face_orientation, orientation, cancel)?;
        check(cancel)?;
    }

    Ok(Outcome::Repaired(dmesh3))
}

/// Interior by fast winding number, Occluded by the six-ray test, both per vertex.
fn remove_inside(dmesh3: &mut DMesh3, remove_mode: RemoveModes) {
    let mode = match remove_mode {
        RemoveModes::None => return,
        RemoveModes::Interior => CalculationMode::FastWindingNumber,
        RemoveModes::Occluded => CalculationMode::SimpleOcclusionTest,
    };
    let mut remove = RemoveOccludedTriangles::new(mode);
    remove.per_vertex = true;
    remove.apply(dmesh3);
}

/// The pipeline's trailing global-orientation passes, routed the same way the FaceOrientation step
/// is.
///
/// Crack repair and hole filling can join regions that disagree about winding, so they each finish
/// with a global orientation pass. Running g3's flood-fill pass there undoes the kernel repair the
/// user asked for: its flood fill picks one sign per component and turns cavity shells outward, so
/// a hollow box comes back solid. When face-orientation repair is on we therefore re-run the kernel
/// instead. With it off, or for the obsolete Repair operation (which predates the kernel pass), the
/// g3 pass runs.
fn global_orientation_pass(
    mut dmesh3: DMesh3,
    face_orientation: bool,
    orientation: Orientation,
    cancel: &CancelToken,
) -> Result<DMesh3, Stop> {
    if face_orientation && orientation == Orientation::PreferKernel {
        return repair_orientation_prefer_kernel(dmesh3, cancel);
    }
    check(cancel)?;
    repair_orientation(&mut dmesh3, true);
    Ok(dmesh3)
}

/// Rewinds inside-out shells with the kernel, falling back to g3's `MeshRepairOrientation`
/// (components, then the global sign) when the kernel will not take the mesh - most often because
/// it is not closed yet. Returns a new `DMesh3` when the kernel repaired it, because the round trip
/// rebuilds the mesh.
fn repair_orientation_prefer_kernel(
    mut dmesh3: DMesh3,
    cancel: &CancelToken,
) -> Result<DMesh3, Stop> {
    check(cancel)?;
    if let Some(rewound) = try_repair_orientation(&dmesh3.to_mesh()) {
        return Ok(rewound.to_dmesh3());
    }
    repair_orientation(&mut dmesh3, true);
    Ok(dmesh3)
}
