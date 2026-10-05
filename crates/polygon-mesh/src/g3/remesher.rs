//! geometry3Sharp's `Remesher` (`mesh/Remesher.cs`): edge-length-driven isotropic remeshing by
//! per-edge collapse / flip / split (`ProcessEdge`), constraint-aware, with buffered smoothing
//! and optional projection onto a target. This file holds the settings, `ProcessEdge` and its
//! split/collapse helpers, the prime-modulo edge order, and the smoothing buffer;
//! `remesher_pro.rs` holds the queue-driven passes (`RemesherPro`) that `MinimalHoleFill`
//! runs. The constraint checks are in `mesh_refiner_base.rs`.
//!
//! Ported subset: what `RemesherPro.FastestRemesh` reaches. `BasicRemeshPass` with its
//! `FullSmoothPass_*` / `FullProjectionPass`, the `MeanValue`/`Cotan`/custom smoothers
//! (`SmoothType`, `CustomSmoothF`, which need `MeshWeights`), the profiling stopwatches, the
//! debug checks (`ENABLE_DEBUG_CHECKS`, `DebugEdges`) and cancellation are not ported. Passes
//! run on one thread; see divergence 795.

// Consumed by `MinimalHoleFill` (mesh repair), which lands in a later slice; until then only the
// tests reach some of it.
#![allow(dead_code)]

use super::dmesh3::{DMesh3, G3Exception, MeshResult, INVALID_ID};
use super::dmesh3_edge_ops::EdgeSplitInfo;
use super::mesh_constraints::{same_target, EdgeConstraint, TargetRef, VertexConstraint};
use super::mesh_refiner_base::MeshRefinerBase;
use super::mesh_util::uniform_smooth;
use super::vector3d::{distance_squared, is_finite};

/// C# `Remesher.VertexControl` (a `[Flags]` enum): per-vertex overrides.
pub(crate) mod vertex_control {
    pub(crate) const ALLOW_ALL: u32 = 0;
    pub(crate) const NO_SMOOTH: u32 = 1;
    pub(crate) const NO_PROJECT: u32 = 2;
    pub(crate) const NO_MOVEMENT: u32 = NO_SMOOTH | NO_PROJECT;
}
use vertex_control as vc_flags;

/// C# `Remesher.TargetProjectionMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TargetProjectionMode {
    /// Disable projection.
    NoProjection,
    /// Do all projection after the refine/smooth pass.
    AfterRefinement,
    /// Project after each vertex update: better results but more expensive.
    Inline,
}

/// C# `Remesher.ProcessResult`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ProcessResult {
    OkCollapsed,
    OkFlipped,
    OkSplit,
    IgnoredEdgeIsFine,
    IgnoredEdgeIsFullyConstrained,
    FailedOpNotSuccessful,
    FailedNotAnEdge,
}

/// C#'s `nPrime`: edges are visited in prime-modulo order to break symmetry, so that eg the
/// tiny cap edges of a minimal cylinder are not collapsed one after another until the mesh
/// erodes away.
const N_PRIME: i32 = 31337;

/// C# `Remesher`.
pub(crate) struct Remesher<'a> {
    pub(crate) base: MeshRefinerBase<'a>,
    target: TargetRef,

    pub(crate) enable_flips: bool,
    pub(crate) enable_collapses: bool,
    pub(crate) enable_splits: bool,
    pub(crate) enable_smoothing: bool,
    pub(crate) prevent_normal_flips: bool,

    pub(crate) min_edge_length: f64,
    pub(crate) max_edge_length: f64,
    pub(crate) smooth_speed_t: f64,

    /// C# `VertexControlF`: granular per-vertex control, somewhat redundant with the vertex
    /// constraints.
    pub(crate) vertex_control_f: Option<Box<dyn Fn(i32) -> u32 + 'a>>,
    pub(crate) projection_mode: TargetProjectionMode,

    /// If the mesh is known closed, the is-boundary checks are skipped (`Precompute`).
    mesh_is_closed: bool,
    /// Edges modified in the previous pass; small relative to the edge count means converged.
    pub(crate) modified_edges_last_pass: i32,
    n_max_edge_id: i32,

    pub(crate) v_buffer_v: Vec<[f64; 3]>,
    pub(crate) v_modified_v: Vec<bool>,
}

impl<'a> Remesher<'a> {
    /// C# `new Remesher(m)`, with C#'s defaults (the `float` literals widened as C# does).
    pub(crate) fn new(mesh: &'a mut DMesh3) -> Self {
        Self {
            base: MeshRefinerBase::new(mesh),
            target: None,
            enable_flips: true,
            enable_collapses: true,
            enable_splits: true,
            enable_smoothing: true,
            prevent_normal_flips: false,
            min_edge_length: 0.001_f32 as f64,
            max_edge_length: 0.1_f32 as f64,
            smooth_speed_t: 0.1_f32 as f64,
            vertex_control_f: None,
            projection_mode: TargetProjectionMode::AfterRefinement,
            mesh_is_closed: false,
            modified_edges_last_pass: 0,
            n_max_edge_id: 0,
            v_buffer_v: Vec::new(),
            v_modified_v: vec![false; 4096],
        }
    }

    /// C# `Mesh`.
    pub(crate) fn mesh(&self) -> &DMesh3 {
        self.base.mesh
    }

    /// C# `ProjectionTarget`.
    pub(crate) fn projection_target(&self) -> &TargetRef {
        &self.target
    }

    /// C# `SetProjectionTarget`.
    pub(crate) fn set_projection_target(&mut self, target: TargetRef) {
        self.target = target;
    }

    /// C# `EnableInlineProjection`.
    fn enable_inline_projection(&self) -> bool {
        self.projection_mode == TargetProjectionMode::Inline
    }

    /// C# `SetTargetEdgeLength`: min/max at 0.66 / 1.33 of the target, so that both halves of
    /// a split edge stay above the minimum.
    pub(crate) fn set_target_edge_length(&mut self, f_length: f64) {
        self.min_edge_length = f_length * 0.66;
        self.max_edge_length = f_length * 1.33;
    }

    /// C# `Precompute`: records whether the mesh is closed so the flip valence tests can skip
    /// the boundary checks. Re-run after editing the mesh externally.
    pub(crate) fn precompute(&mut self) {
        let mesh = &*self.base.mesh;
        self.mesh_is_closed = !mesh.edge_indices().any(|eid| mesh.is_boundary_edge(eid));
    }

    fn vertex_control(&self, vid: i32) -> u32 {
        match &self.vertex_control_f {
            Some(f) => f(vid),
            None => vc_flags::ALLOW_ALL,
        }
    }

    /// C# `start_edges()` / `next_edge()`, collected: every id below the pass's starting
    /// `MaxEdgeID`, stepping by [`N_PRIME`] modulo it. The bound is fixed when the pass
    /// starts, so collecting up front visits the same ids as C#'s lazy iterator.
    pub(crate) fn edge_order(&mut self) -> Vec<i32> {
        self.n_max_edge_id = self.base.mesh.max_edge_id();
        let mut out = Vec::new();
        let mut cur_eid = 0;
        loop {
            out.push(cur_eid);
            let new_eid = (cur_eid + N_PRIME) % self.n_max_edge_id;
            if new_eid == 0 {
                break;
            }
            cur_eid = new_eid;
        }
        out
    }

    /// C# `smooth_vertices()`.
    pub(crate) fn smooth_vertices(&self) -> Vec<i32> {
        self.base.mesh.vertex_indices().collect()
    }

    /// C# `ProcessEdge`: collapse the edge if it is too short, else flip it if that improves
    /// the valences, else split it if it is too long. `on_split` is C#'s `OnEdgeSplit` hook.
    /// A `G3Exception` from `FlipEdge` unwinds out, as in C#.
    pub(crate) fn process_edge(
        &mut self,
        edge_id: i32,
        on_split: &mut dyn FnMut(&DMesh3, &EdgeSplitInfo),
    ) -> Result<ProcessResult, G3Exception> {
        let constraint = match &self.base.constraints {
            None => EdgeConstraint::unconstrained(),
            Some(c) => c.get_edge_constraint(edge_id),
        };
        if constraint.no_modifications() {
            return Ok(ProcessResult::IgnoredEdgeIsFullyConstrained);
        }

        // look up verts and tris for this edge
        if !self.base.mesh.is_edge(edge_id) {
            return Ok(ProcessResult::FailedNotAnEdge);
        }
        let [a, b, t0, t1] = self.base.mesh.get_edge(edge_id);
        let b_is_boundary_edge = t1 == INVALID_ID;

        // look up 'other' verts c (from t0) and d (from t1, if it exists)
        let [c, d] = self.base.mesh.get_edge_opposing_v(edge_id);

        let v_a = self.base.mesh.vertex(a);
        let v_b = self.base.mesh.vertex(b);
        let edge_len_sqr = distance_squared(v_a, v_b);

        // check if we should collapse, and also find which vertex we should collapse to, in
        // cases where we have constraints/etc
        let mut collapse_to = -1;
        let b_can_collapse = self.enable_collapses
            && constraint.can_collapse()
            && edge_len_sqr < self.min_edge_length * self.min_edge_length
            && {
                let (ok, to) = self
                    .base
                    .can_collapse_constraints(edge_id, a, b, c, d, t0, t1);
                collapse_to = to;
                ok
            };

        // if edge length is too short, we want to collapse it
        let mut b_tried_collapse = false;
        if b_can_collapse {
            if let Some(result) = self.try_collapse(edge_id, a, b, t0, t1, v_a, v_b, collapse_to) {
                if result == MeshResult::Ok {
                    return Ok(ProcessResult::OkCollapsed);
                }
                b_tried_collapse = true;
            }
        }

        // if this is not a boundary edge, maybe we want to flip
        let mut b_tried_flip = false;
        if self.enable_flips && constraint.can_flip() && !b_is_boundary_edge {
            let mesh = &*self.base.mesh;
            let closed = self.mesh_is_closed;
            // can we do this more efficiently somehow?
            let a_is_boundary_vtx = !closed && (b_is_boundary_edge || mesh.is_boundary_vertex(a));
            let b_is_boundary_vtx = !closed && (b_is_boundary_edge || mesh.is_boundary_vertex(b));
            let c_is_boundary_vtx = !closed && mesh.is_boundary_vertex(c);
            let d_is_boundary_vtx = !closed && mesh.is_boundary_vertex(d);
            let valence_a = mesh.get_vtx_edge_count(a);
            let valence_b = mesh.get_vtx_edge_count(b);
            let valence_c = mesh.get_vtx_edge_count(c);
            let valence_d = mesh.get_vtx_edge_count(d);
            let valence_a_target = if a_is_boundary_vtx { valence_a } else { 6 };
            let valence_b_target = if b_is_boundary_vtx { valence_b } else { 6 };
            let valence_c_target = if c_is_boundary_vtx { valence_c } else { 6 };
            let valence_d_target = if d_is_boundary_vtx { valence_d } else { 6 };

            // if total valence error improves by flip, we want to do it
            let curr_err = (valence_a - valence_a_target).abs()
                + (valence_b - valence_b_target).abs()
                + (valence_c - valence_c_target).abs()
                + (valence_d - valence_d_target).abs();
            let flip_err = ((valence_a - 1) - valence_a_target).abs()
                + ((valence_b - 1) - valence_b_target).abs()
                + ((valence_c + 1) - valence_c_target).abs()
                + ((valence_d + 1) - valence_d_target).abs();

            let mut b_try_flip = flip_err < curr_err;
            if b_try_flip
                && self.prevent_normal_flips
                && self.base.flip_inverts_normals(a, b, c, d, t0)
            {
                b_try_flip = false;
            }

            if b_try_flip {
                let (result, _flip_info) = self.base.mesh.flip_edge(edge_id)?;
                if result == MeshResult::Ok {
                    return Ok(ProcessResult::OkFlipped);
                }
                b_tried_flip = true;
            }
        }

        // if edge length is too long, we want to split it
        let mut b_tried_split = false;
        if self.enable_splits
            && constraint.can_split()
            && edge_len_sqr > self.max_edge_length * self.max_edge_length
        {
            let (result, split_info) = self.base.mesh.split_edge(edge_id, 0.5);
            if result == MeshResult::Ok {
                self.update_after_split(edge_id, a, b, &split_info);
                on_split(self.base.mesh, &split_info);
                return Ok(ProcessResult::OkSplit);
            }
            b_tried_split = true;
        }

        if b_tried_flip || b_tried_split || b_tried_collapse {
            Ok(ProcessResult::FailedOpNotSuccessful)
        } else {
            Ok(ProcessResult::IgnoredEdgeIsFine)
        }
    }

    /// The collapse branch of C# `ProcessEdge`. `None` is C#'s `goto abort_collapse` (the
    /// normal-flip guard refused, so the collapse was not tried).
    #[allow(clippy::too_many_arguments)]
    fn try_collapse(
        &mut self,
        edge_id: i32,
        a: i32,
        b: i32,
        t0: i32,
        t1: i32,
        v_a: [f64; 3],
        v_b: [f64; 3],
        collapse_to: i32,
    ) -> Option<MeshResult> {
        let mut i_keep = b;
        let mut i_collapse = a;
        let mut v_new_pos = [
            (v_a[0] + v_b[0]) * 0.5,
            (v_a[1] + v_b[1]) * 0.5,
            (v_a[2] + v_b[2]) * 0.5,
        ];

        // if either vtx is fixed, collapse to that position
        if collapse_to == b {
            v_new_pos = v_b;
        } else if collapse_to == a {
            i_keep = a;
            i_collapse = b;
            v_new_pos = v_a;
        } else {
            v_new_pos = self.get_projected_collapse_position(i_keep, v_new_pos);
        }

        // if new position would flip normal of one of the existing triangles either one-ring,
        // don't allow it
        if self.prevent_normal_flips
            && (self
                .base
                .collapse_creates_flip_or_invalid(a, b, v_new_pos, t0, t1)
                || self
                    .base
                    .collapse_creates_flip_or_invalid(b, a, v_new_pos, t0, t1))
        {
            return None;
        }

        let (result, collapse_info) = self.base.mesh.collapse_edge(i_keep, i_collapse);
        if result == MeshResult::Ok {
            self.base.mesh.set_vertex(i_keep, v_new_pos);
            if let Some(constraints) = &mut self.base.constraints {
                constraints.clear_edge_constraint(edge_id);
                constraints.clear_edge_constraint(collapse_info.e_removed[0]);
                if collapse_info.e_removed[1] != INVALID_ID {
                    constraints.clear_edge_constraint(collapse_info.e_removed[1]);
                }
                constraints.clear_vertex_constraint(i_collapse);
            }
        }
        Some(result)
    }

    /// C# `update_after_split`: the new edge inherits the split edge's constraint, and the new
    /// vertex may inherit Fixed (both ends fixed) or a Target (shared with the edge).
    fn update_after_split(&mut self, edge_id: i32, va: i32, vb: i32, split_info: &EdgeSplitInfo) {
        let mut b_position_fixed = false;
        let mut project_to: TargetRef = None;
        if let Some(constraints) = &mut self.base.constraints {
            if constraints.has_edge_constraint(edge_id) {
                // inherit edge constraint
                let edge_constraint = constraints.get_edge_constraint(edge_id);
                constraints.set_or_update_edge_constraint(split_info.e_new_bn, edge_constraint);

                // vert inherits Fixed if both orig edge verts Fixed, and both tagged with same
                // SetID. (Both ends constrained does not by itself mean the edge is on a
                // constraint, which is why this only applies to a constrained edge.)
                let ca = constraints.get_vertex_constraint(va);
                let cb = constraints.get_vertex_constraint(vb);
                if ca.fixed && cb.fixed {
                    let n_set_id = if ca.fixed_set_id > 0 && ca.fixed_set_id == cb.fixed_set_id {
                        ca.fixed_set_id
                    } else {
                        VertexConstraint::INVALID_SET_ID
                    };
                    constraints.set_or_update_vertex_constraint(
                        split_info.v_new,
                        VertexConstraint::new_fixed(true, n_set_id),
                    );
                    b_position_fixed = true;
                }

                // vert inherits Target if:
                //  1) both source verts and edge have same Target, and is same as edge target
                //  2) either vert has same target as edge, and other vert is fixed
                if ca.target.is_some() || cb.target.is_some() {
                    let edge_target = constraints.get_edge_constraint(edge_id).target;
                    let inherits = (same_target(&ca.target, &cb.target)
                        && same_target(&ca.target, &edge_target))
                        || (same_target(&ca.target, &edge_target) && cb.fixed)
                        || (same_target(&cb.target, &edge_target) && ca.fixed);
                    let set_target = if inherits { edge_target } else { None };

                    if set_target.is_some() {
                        constraints.set_or_update_vertex_constraint(
                            split_info.v_new,
                            VertexConstraint::new_target(set_target.clone()),
                        );
                        project_to = set_target;
                        b_position_fixed = true;
                    }
                }
            }
        }
        if let Some(target) = project_to {
            self.project_vertex(split_info.v_new, &*target);
        }

        if self.enable_inline_projection() && !b_position_fixed {
            if let Some(target) = self.target.clone() {
                self.project_vertex(split_info.v_new, &*target);
            }
        }
    }

    /// C# `project_vertex`.
    fn project_vertex(&mut self, vid: i32, target: &dyn super::mesh_constraints::ProjectionTarget) {
        let curpos = self.base.mesh.vertex(vid);
        let projected = target.project(curpos, vid);
        self.base.mesh.set_vertex(vid, projected);
    }

    /// C# `get_projected_collapse_position`: the collapse point projected onto the kept
    /// vertex's constraint target, or the surface target when projecting inline.
    fn get_projected_collapse_position(&self, vid: i32, v_new_pos: [f64; 3]) -> [f64; 3] {
        if let Some(constraints) = &self.base.constraints {
            let vc = constraints.get_vertex_constraint(vid);
            if let Some(target) = &vc.target {
                return target.project(v_new_pos, vid);
            }
            if vc.fixed {
                return v_new_pos;
            }
        }
        // no constraint applied, so if we have a target surface, project to that
        if self.enable_inline_projection() {
            if let Some(target) = &self.target {
                if (self.vertex_control(vid) & vc_flags::NO_PROJECT) == 0 {
                    return target.project(v_new_pos, vid);
                }
            }
        }
        v_new_pos
    }

    /// C# `InitializeVertexBufferForPass`: buffer room for every vertex id, all flags clear.
    pub(crate) fn initialize_vertex_buffer_for_pass(&mut self) {
        let max_vid = self.base.mesh.max_vertex_id() as usize;
        if self.v_buffer_v.len() < max_vid {
            self.v_buffer_v.resize(max_vid + max_vid / 5, [0.0; 3]);
        }
        if self.v_modified_v.len() < max_vid {
            self.v_modified_v = vec![false; 2 * max_vid];
        } else {
            self.v_modified_v.fill(false);
        }
    }

    /// C# `ApplyVertexBuffer`: writes every flagged buffered position back to the mesh.
    pub(crate) fn apply_vertex_buffer(&mut self) {
        let max_vid = self.base.mesh.max_vertex_id();
        for vid in 0..max_vid {
            if self.v_modified_v[vid as usize] {
                self.base
                    .mesh
                    .set_vertex(vid, self.v_buffer_v[vid as usize]);
            }
        }
    }

    /// C# `ComputeSmoothedVertexPos` with the uniform smoother: the smoothed position under
    /// the vertex's constraint and control flags, and whether it may move. Does not modify the
    /// mesh.
    pub(crate) fn compute_smoothed_vertex_pos(&self, vid: i32) -> ([f64; 3], bool) {
        // C#'s `get_vertex_constraint(vID, ref vc)` leaves a default struct when none is set:
        // not fixed, no target.
        let v_constraint = self
            .base
            .constraints
            .as_ref()
            .and_then(|c| c.try_get_vertex_constraint(vid))
            .unwrap_or_else(VertexConstraint::unconstrained);
        if v_constraint.fixed {
            return (self.base.mesh.vertex(vid), false);
        }

        let v_control = self.vertex_control(vid);
        if (v_control & vc_flags::NO_SMOOTH) != 0 {
            return (self.base.mesh.vertex(vid), false);
        }

        let mut v_smoothed = uniform_smooth(self.base.mesh, vid, self.smooth_speed_t);
        debug_assert!(is_finite(v_smoothed)); // this will really catch a lot of bugs...

        // project onto either vtx constraint target, or surface target
        if let Some(target) = &v_constraint.target {
            v_smoothed = target.project(v_smoothed, vid);
        } else if self.enable_inline_projection() {
            if let Some(target) = &self.target {
                if (v_control & vc_flags::NO_PROJECT) == 0 {
                    v_smoothed = target.project(v_smoothed, vid);
                }
            }
        }

        (v_smoothed, true)
    }

    /// The `VertexControlF` check of C#'s projection passes.
    pub(crate) fn vertex_blocks_projection(&self, vid: i32) -> bool {
        (self.vertex_control(vid) & vc_flags::NO_PROJECT) != 0
    }
}
