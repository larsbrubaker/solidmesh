//! geometry3Sharp's `MeshAutoRepair`: the Repair tool's default pass. Makes component orientation
//! consistent (`mesh_repair_orientation`), welds cracks (`merge_coincident_edges`), collapses
//! degenerate edges, fills holes (`simple_hole_filler`, `minimal_hole_fill`), splits bowties, and
//! if the mesh is still open retries and then erodes boundary triangles, before a final
//! degenerate-edge cleanup and global orientation. Also [`modulo_iteration`].
//!
//! It never removes interior triangles here (the Repair tool's default never asks; the Advanced
//! path's interior/occluded removal runs `remove_occluded_triangles` itself, from
//! `advanced_repair`, which also reuses the hole, crack, collapse and orientation helpers below),
//! and there is no progress reporter or cancellation: nothing sets them on this path. There is no
//! final compacted copy or normal computation: callers read the mesh they passed in, which is the
//! one edited here.

use super::dmesh3::{DMesh3, G3Exception, MeshResult};
use super::merge_coincident_edges::MergeCoincidentEdges;
use super::mesh_boundary_loops::MeshBoundaryLoops;
use super::mesh_editor::{disconnect_all_bowties, remove_isolated_triangles, remove_triangles};
use super::mesh_face_selection::MeshFaceSelection;
use super::mesh_iterators::boundary_edges;
use super::mesh_repair_orientation::MeshRepairOrientation;
use super::minimal_hole_fill::MinimalHoleFill;
use super::simple_hole_filler::SimpleHoleFiller;
use super::vector3d::{add, distance, scale, ZERO_TOLERANCEF};

/// `0, p, 2p, ...` modulo `n` until it comes back to 0. `n == 0` is `Err` rather than a division by
/// zero (a mesh with no edges has nothing to iterate, so nothing is lost).
pub(crate) fn modulo_iteration(
    n_max_exclusive: i32,
    n_prime: i32,
) -> Result<Vec<i32>, G3Exception> {
    if n_max_exclusive == 0 {
        return Err(G3Exception(
            "MathUtil.ModuloIteration: attempted to divide by zero.",
        ));
    }
    let mut out = Vec::new();
    let mut i: i32 = 0;
    loop {
        out.push(i);
        // i32 arithmetic: i + nPrime cannot overflow for an id below i32::MAX - 31337.
        i = (i + n_prime) % n_max_exclusive;
        if i == 0 {
            break;
        }
    }
    Ok(out)
}

/// g3 `MeshAutoRepair` with `RemoveMode = None`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct MeshAutoRepair {
    pub(crate) repair_tolerance: f64,
    /// assume edges shorter than this are degenerate and should be collapsed
    pub(crate) min_edge_length_tol: f64,
    /// number of times we will delete border triangles and try again
    pub(crate) erosion_iterations: i32,
}

impl Default for MeshAutoRepair {
    fn default() -> Self {
        Self {
            repair_tolerance: ZERO_TOLERANCEF,
            min_edge_length_tol: 0.0001,
            erosion_iterations: 5,
        }
    }
}

impl MeshAutoRepair {
    /// Always true without cancellation. `Err` is a g3 failure (from the hole fillers, or
    /// [`modulo_iteration`] on a mesh with no edges).
    pub(crate) fn apply(&self, mesh: &mut DMesh3) -> Result<bool, G3Exception> {
        // Remove parts of the mesh we don't want before we bother with anything else:
        // RemoveMode None removes nothing.

        let mut repeat_count = 0;
        'repeat_all: loop {
            // make sure orientation of connected components is consistent
            // TODO: what about mobius strip problems?
            repair_orientation(mesh, false);

            // Do safe close-cracks to handle easy cases
            repair_cracks(mesh, true, self.repair_tolerance);
            if mesh.is_closed() {
                break 'repeat_all;
            }

            // Collapse tiny edges and then try easy cases again, and
            // then allow for handling of ambiguous cases
            collapse_all_degenerate_edges(mesh, self.repair_tolerance * 0.5, true)?;
            repair_cracks(mesh, true, 2.0 * self.repair_tolerance);
            repair_cracks(mesh, false, 2.0 * self.repair_tolerance);
            if mesh.is_closed() {
                break 'repeat_all;
            }

            // Possibly we have joined regions with different orientation (is it?), fix that
            // TODO: mobius strips again
            repair_orientation(mesh, false);

            // get rid of any remaining single-triangles before we start filling holes
            remove_isolated_triangles(mesh);

            // Ok, fill simple holes.
            fill_trivial_holes(mesh)?;
            if mesh.is_closed() {
                break 'repeat_all;
            }

            // Now fill harder holes. If we saw spans, that means boundary loops could
            // not be resolved in some cases, do we disconnect bowties and try again.
            let saw_spans = fill_any_holes(mesh)?;
            if saw_spans {
                disconnect_all_bowties(mesh, 10);
                fill_any_holes(mesh)?;
            }
            if mesh.is_closed() {
                break 'repeat_all;
            }

            // We may have a closed mesh now but it might still have bowties (eg
            // tetrahedra sharing vtx case). So disconnect those.
            disconnect_all_bowties(mesh, 10);

            // If the mesh is not closed, we will do one more round to try again.
            if repeat_count == 0 && !mesh.is_closed() {
                repeat_count += 1;
                continue 'repeat_all;
            }

            // Ok, we didn't get anywhere on our first repeat. If we are still not
            // closed, we will try deleting boundary triangles and repeating.
            // Repeat this N times.
            if repeat_count <= self.erosion_iterations && !mesh.is_closed() {
                repeat_count += 1;
                let mut bdry_faces = MeshFaceSelection::new();
                for eid in boundary_edges(mesh) {
                    bdry_faces.select_edge_tris(mesh, eid);
                }
                let tris: Vec<i32> = bdry_faces.iter().collect();
                remove_triangles(mesh, tris, true);
                continue 'repeat_all;
            }
            break 'repeat_all;
        }

        // all_done: Remove tiny edges
        if self.min_edge_length_tol > 0.0 {
            collapse_all_degenerate_edges(mesh, self.min_edge_length_tol, false)?;
        }

        // finally do global orientation
        repair_orientation(mesh, true);
        Ok(true)
    }
}

/// Three-vertex loops get one triangle, four-vertex loops a minimal fill (or a fan if that fails).
pub(crate) fn fill_trivial_holes(mesh: &mut DMesh3) -> Result<(), G3Exception> {
    let loops = MeshBoundaryLoops::new(mesh);
    for lp in &loops.loops {
        if lp.vertex_count() == 3 {
            SimpleHoleFiller::new(lp).fill(mesh, -1)?;
        } else if lp.vertex_count() == 4 {
            let filled = MinimalHoleFill::default().apply(mesh, lp)?;
            if !filled {
                SimpleHoleFiller::new(lp).fill(mesh, -1)?;
            }
        }
    }
    Ok(())
}

/// A minimal fill of every loop, a fan where that fails. Returns `SawOpenSpans` of the loops it
/// filled.
pub(crate) fn fill_any_holes(mesh: &mut DMesh3) -> Result<bool, G3Exception> {
    let loops = MeshBoundaryLoops::new(mesh);
    for lp in &loops.loops {
        let filled = MinimalHoleFill::default().apply(mesh, lp)?;
        if !filled {
            SimpleHoleFiller::new(lp).fill(mesh, -1)?;
        }
    }
    Ok(loops.saw_open_spans)
}

/// A `MergeCoincidentEdges` pass; anything it throws is caught (and the mesh keeps whatever it had
/// already changed).
pub(crate) fn repair_cracks(mesh: &mut DMesh3, unique_only: bool, merge_dist: f64) -> bool {
    let merge = MergeCoincidentEdges {
        merge_distance: merge_dist,
        only_unique_pairs: unique_only,
    };
    merge.apply(mesh).unwrap_or(false)
}

/// Visits edge ids in [`modulo_iteration`] order ("don't iterate sequentially because there may be
/// pathological cases") and collapses each edge shorter than `min_length`, keeping a boundary
/// vertex where there is one.
fn collapse_degenerate_edges(
    mesh: &mut DMesh3,
    min_length: f64,
    boundary_only: bool,
) -> Result<i32, G3Exception> {
    let mut collapse_count = 0;
    for eid in modulo_iteration(mesh.max_edge_id(), 31337)? {
        if !mesh.is_edge(eid) {
            continue;
        }
        let is_boundary_edge = mesh.is_boundary_edge(eid);
        if boundary_only && !is_boundary_edge {
            continue;
        }

        let ev = mesh.get_edge_v(eid);
        let a = mesh.vertex(ev[0]);
        let b = mesh.vertex(ev[1]);
        if distance(a, b) < min_length {
            let keep = if mesh.is_boundary_vertex(ev[0]) {
                ev[0]
            } else {
                ev[1]
            };
            let discard = if keep == ev[0] { ev[1] } else { ev[0] };
            let (result, _) = mesh.collapse_edge(keep, discard);
            if result == MeshResult::Ok {
                collapse_count += 1;
                if !mesh.is_boundary_vertex(keep) || is_boundary_edge {
                    mesh.set_vertex(keep, scale(add(a, b), 0.5));
                }
            }
        }
    }
    Ok(collapse_count)
}

/// Repeats until a pass collapses nothing.
pub(crate) fn collapse_all_degenerate_edges(
    mesh: &mut DMesh3,
    min_length: f64,
    boundary_only: bool,
) -> Result<(), G3Exception> {
    while collapse_degenerate_edges(mesh, min_length, boundary_only)? != 0 {}
    Ok(())
}

pub(crate) fn repair_orientation(mesh: &mut DMesh3, global: bool) {
    let mut orient = MeshRepairOrientation::new();
    orient.orient_components(mesh);
    if global {
        orient.solve_global_orientation(mesh);
    }
}

/// An exception geometry3Sharp's repair let escape, with its message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeshAutoRepairError(pub &'static str);

impl std::fmt::Display for MeshAutoRepairError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for MeshAutoRepairError {}

impl crate::Mesh {
    /// The geometry half of the Repair operation's default (non-Advanced) path: the mesh into a
    /// `DMesh3`, `MeshAutoRepair::apply`, then back to a [`crate::Mesh`] from that same `DMesh3`.
    /// Face colors are not carried (the caller saves and restores them by face centroid, since the
    /// round trip renumbers faces); face textures are, onto the faces whose corners did not move.
    pub fn auto_repaired(&self) -> Result<crate::Mesh, MeshAutoRepairError> {
        let saved_textures = self.save_face_corner_textures();
        let mut dmesh3 = self.to_dmesh3();
        MeshAutoRepair::default()
            .apply(&mut dmesh3)
            .map_err(|e| MeshAutoRepairError(e.0))?;
        let mut result = dmesh3.to_mesh();
        result.restore_face_textures_from_corners(&saved_textures);
        Ok(result)
    }
}
