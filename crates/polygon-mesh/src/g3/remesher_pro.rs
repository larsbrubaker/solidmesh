//! The queued remesher (derived from geometry3Sharp): a `Remesher` (`remesher.rs`) that
//! queues the edges each pass touched and only revisits those - a fast split iteration splits
//! to reach the target edge length quickly, the remesh iteration runs the collapse/flip/split
//! pass over the queue and then the tracked smoothing and projection passes, and
//! `fastest_remesh` drives both. `MinimalHoleFill` remeshes its fill this way.
//!
//! Projection moves each vertex to its nearest surface point (there is no face-aligned
//! projection). Passes run on one thread, so the queue order, and with it the remeshed result,
//! is the same on every run.

use std::collections::HashSet;

use super::dmesh3::{DMesh3, G3Exception};
use super::dmesh3_edge_ops::EdgeSplitInfo;
use super::mesh_constraints::{fix_all_boundary_edges, MeshConstraints};
use super::remesher::{ProcessResult, Remesher, TargetProjectionMode};
use super::vector3d::{distance, distance_squared, epsilon_equal, ZERO_TOLERANCEF};

/// The modified-edge queue. The set is only ever added to and cleared, and its order sets the
/// next iteration's edge order, so it enumerates in first-insertion order.
#[derive(Default)]
struct EdgeQueue {
    order: Vec<i32>,
    seen: HashSet<i32>,
}

impl EdgeQueue {
    fn add(&mut self, eid: i32) {
        if self.seen.insert(eid) {
            self.order.push(eid);
        }
    }

    fn clear(&mut self) {
        self.order.clear();
        self.seen.clear();
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    fn len(&self) -> usize {
        self.order.len()
    }
}

/// The remesher settings a pass changes, saved by `push_state` and restored by `pop_state`.
#[derive(Clone, Copy)]
struct SettingState {
    enable_flips: bool,
    enable_collapses: bool,
    enable_splits: bool,
    enable_smoothing: bool,
    min_edge_length: f64,
    max_edge_length: f64,
    smooth_speed_t: f64,
    projection_mode: TargetProjectionMode,
}

/// The queued remesher over one mesh.
pub(crate) struct RemesherPro<'a> {
    pub(crate) remesher: Remesher<'a>,
    /// `None` until a pass starts, and again after the queue is reset.
    modified_edges: Option<EdgeQueue>,
    state_stack: Vec<SettingState>,
}

/// Queues every edge of a live vertex.
fn queue_one_ring(mesh: &DMesh3, queue: &mut EdgeQueue, vid: i32) {
    if mesh.is_vertex(vid) {
        for eid in mesh.vtx_edges(vid) {
            queue.add(eid);
        }
    }
}

impl<'a> RemesherPro<'a> {
    /// A queued remesher over `mesh` with the default settings.
    pub(crate) fn new(mesh: &'a mut DMesh3) -> Self {
        Self {
            remesher: Remesher::new(mesh),
            modified_edges: None,
            state_stack: Vec::new(),
        }
    }

    /// Creates the constraint set if
    /// there is none, then pins every boundary edge and vertex.
    pub(crate) fn fix_all_boundary_edges(&mut self) {
        let base = &mut self.remesher.base;
        let cons = base
            .constraints
            .get_or_insert_with(MeshConstraints::default);
        fix_all_boundary_edges(cons, base.mesh);
    }

    /// Fast splits to reach
    /// the edge-length target, then queued remesh iterations with projection on every other
    /// one, then a final iteration with full projection.
    pub(crate) fn fastest_remesh(
        &mut self,
        n_max_iterations: i32,
        b_do_fast_splits: bool,
    ) -> Result<(), G3Exception> {
        self.reset_queue();

        // first we do fast splits to hit edge length target
        let mut fastsplit_i = 0;
        let max_fastsplits = n_max_iterations;
        if b_do_fast_splits {
            let mut b_continue = true;
            while b_continue {
                let n_splits = self.fast_split_iteration()?;
                let done_iterating = fastsplit_i > max_fastsplits;
                fastsplit_i += 1;
                if done_iterating {
                    b_continue = false;
                }

                if (n_splits as f64) / (self.remesher.mesh().edge_count() as f64) < 0.01 {
                    b_continue = false;
                }
            }
            self.reset_queue();
        }

        // now do queued remesh iterations. disable projection every other iteration to
        // improve speed
        let save_mode = self.remesher.projection_mode;
        for k in 0..n_max_iterations - 1 {
            self.remesher.projection_mode = if k % 2 == 0 {
                TargetProjectionMode::NoProjection
            } else {
                save_mode
            };
            self.remesh_iteration()?;
        }

        // final pass w/ full projection
        self.remesher.projection_mode = save_mode;
        self.remesh_iteration()
    }

    /// Call when the mesh was edited externally between passes, and between
    /// different kinds of pass.
    pub(crate) fn reset_queue(&mut self) {
        self.modified_edges = None;
    }

    /// The edges a pass visits: the whole mesh in prime-modulo order on the first pass, the
    /// queued edges after that (the queue is emptied for the pass to refill).
    fn pass_edges(&mut self) -> Vec<i32> {
        match &mut self.modified_edges {
            None => {
                let edges = self.remesher.edge_order();
                self.modified_edges = Some(EdgeQueue::default());
                edges
            }
            Some(queue) => {
                let edges = queue.order.clone();
                queue.clear();
                edges
            }
        }
    }

    /// One pass of splits only. Returns the number of splits.
    pub(crate) fn fast_split_iteration(&mut self) -> Result<i32, G3Exception> {
        if self.remesher.mesh().triangle_count() == 0 {
            // badness if we don't catch this...
            return Ok(0);
        }

        self.push_state();
        let r = &mut self.remesher;
        r.enable_flips = false;
        r.enable_collapses = false;
        r.enable_smoothing = false;
        r.projection_mode = TargetProjectionMode::NoProjection;

        // Iterate over all edges in the mesh at start of pass. Some may be removed, so we skip
        // those. However, some old eid's may also be re-used, so we will touch some new edges.
        let edges = self.pass_edges();
        let mut split_edges = 0;

        // When we split an edge, we need to check it and the adjacent ones we added. Because of
        // overhead in process_edge, it is worth it to do a distance-check here (split_f).
        let max_edge_len_sqr = self.remesher.max_edge_length * self.remesher.max_edge_length;
        let queue = self.modified_edges.get_or_insert_with(EdgeQueue::default);
        let mut split_f = |mesh: &DMesh3, info: &EdgeSplitInfo| {
            let v_new = info.v_new;
            let v = mesh.vertex(v_new);
            for eid in mesh.vtx_edges(v_new) {
                let ev = mesh.get_edge_v(eid);
                let othervid = if ev[0] == v_new { ev[1] } else { ev[0] };
                if distance_squared(mesh.vertex(othervid), v) > max_edge_len_sqr {
                    queue.add(eid);
                }
            }
        };

        self.remesher.modified_edges_last_pass = 0;
        let mut outcome = Ok(());
        for cur_eid in edges {
            if self.remesher.mesh().is_edge(cur_eid) {
                // Flips are off for this pass, so the flip (the only edit that can fail) is not
                // reached; an error still stops the pass only after the settings are restored.
                match self.remesher.process_edge(cur_eid, &mut split_f) {
                    Ok(ProcessResult::OkSplit) => {
                        // new edges queued by split_f
                        self.remesher.modified_edges_last_pass += 1;
                        split_edges += 1;
                    }
                    Ok(_) => {}
                    Err(error) => {
                        outcome = Err(error);
                        break;
                    }
                }
            }
        }

        self.pop_state();
        outcome?;
        Ok(split_edges)
    }

    /// The collapse/flip/split pass over the queued edges (queueing the
    /// one-rings around every edit), then tracked smoothing, then tracked projection.
    pub(crate) fn remesh_iteration(&mut self) -> Result<(), G3Exception> {
        if self.remesher.mesh().triangle_count() == 0 {
            // badness if we don't catch this...
            return Ok(());
        }

        let edges = self.pass_edges();
        let queue = self.modified_edges.get_or_insert_with(EdgeQueue::default);

        self.remesher.modified_edges_last_pass = 0;
        for cur_eid in edges {
            if !self.remesher.mesh().is_edge(cur_eid) {
                continue;
            }
            let ev = self.remesher.mesh().get_edge_v(cur_eid);
            let ov = self.remesher.mesh().get_edge_opposing_v(cur_eid);

            let result = self.remesher.process_edge(cur_eid, &mut |_, _| {})?;
            if matches!(
                result,
                ProcessResult::OkCollapsed | ProcessResult::OkSplit | ProcessResult::OkFlipped
            ) {
                let mesh = self.remesher.mesh();
                queue_one_ring(mesh, queue, ev[0]);
                queue_one_ring(mesh, queue, ev[1]);
                queue_one_ring(mesh, queue, ov[0]);
                queue_one_ring(mesh, queue, ov[1]);
                self.remesher.modified_edges_last_pass += 1;
            }
        }

        if self.remesher.enable_smoothing && self.remesher.smooth_speed_t > 0.0 {
            self.tracked_smooth_pass();
        }

        if self.remesher.projection_target().is_some()
            && self.remesher.projection_mode == TargetProjectionMode::AfterRefinement
        {
            self.tracked_projection_pass();
        }
        Ok(())
    }

    /// Smooths every vertex into the buffer, queueing each edge whose
    /// new length leaves [min, max], then applies the buffer.
    fn tracked_smooth_pass(&mut self) {
        self.remesher.initialize_vertex_buffer_for_pass();
        let queue = self.modified_edges.get_or_insert_with(EdgeQueue::default);
        let r = &mut self.remesher;

        for vid in r.smooth_vertices() {
            let (v_smoothed, b_modified) = r.compute_smoothed_vertex_pos(vid);
            if b_modified {
                r.v_modified_v[vid as usize] = true;
                r.v_buffer_v[vid as usize] = v_smoothed;
                queue_edges_leaving_range(r, queue, vid, v_smoothed);
            }
        }

        r.apply_vertex_buffer();
    }

    /// Projects every unconstrained vertex onto the target into
    /// the buffer, queueing edges whose new length leaves [min, max], then applies it.
    fn tracked_projection_pass(&mut self) {
        self.remesher.initialize_vertex_buffer_for_pass();
        let queue = self.modified_edges.get_or_insert_with(EdgeQueue::default);
        let r = &mut self.remesher;

        for vid in r.smooth_vertices() {
            let v_cur = r.mesh().vertex(vid);
            let (v_projected, mut b_modified) = compute_projected_vertex_pos(r, vid);
            if epsilon_equal(v_cur, v_projected, ZERO_TOLERANCEF) {
                b_modified = false;
            }

            if b_modified {
                r.v_modified_v[vid as usize] = true;
                r.v_buffer_v[vid as usize] = v_projected;
                queue_edges_leaving_range(r, queue, vid, v_projected);
            }
        }

        r.apply_vertex_buffer();
    }

    /// Saves the settings a specialized pass overrides.
    pub(crate) fn push_state(&mut self) {
        let r = &self.remesher;
        self.state_stack.push(SettingState {
            enable_flips: r.enable_flips,
            enable_collapses: r.enable_collapses,
            enable_splits: r.enable_splits,
            enable_smoothing: r.enable_smoothing,
            min_edge_length: r.min_edge_length,
            max_edge_length: r.max_edge_length,
            smooth_speed_t: r.smooth_speed_t,
            projection_mode: r.projection_mode,
        });
    }

    /// Restores the settings `push_state` saved.
    pub(crate) fn pop_state(&mut self) {
        let Some(s) = self.state_stack.pop() else {
            return;
        };
        let r = &mut self.remesher;
        r.enable_flips = s.enable_flips;
        r.enable_collapses = s.enable_collapses;
        r.enable_splits = s.enable_splits;
        r.enable_smoothing = s.enable_smoothing;
        r.min_edge_length = s.min_edge_length;
        r.max_edge_length = s.max_edge_length;
        r.smooth_speed_t = s.smooth_speed_t;
        r.projection_mode = s.projection_mode;
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// The number of queued edges (for tests and convergence checks).
    pub(crate) fn queued_edge_count(&self) -> usize {
        self.modified_edges.as_ref().map_or(0, EdgeQueue::len)
    }
}

/// The edge check shared by the tracked passes: queue each edge of `vid` whose length with
/// `vid` at `new_pos` falls outside [min, max].
fn queue_edges_leaving_range(r: &Remesher<'_>, queue: &mut EdgeQueue, vid: i32, new_pos: [f64; 3]) {
    let mesh = r.mesh();
    for eid in mesh.vtx_edges(vid) {
        let ev = mesh.get_edge_v(eid);
        let othervid = if ev[0] == vid { ev[1] } else { ev[0] };
        let otherv = mesh.vertex(othervid);
        let new_len = distance(new_pos, otherv);
        if new_len < r.min_edge_length || new_len > r.max_edge_length {
            queue.add(eid);
        }
    }
}

/// The vertex projected onto the target unless it is
/// constrained or its control flags forbid it. Does not modify the mesh.
fn compute_projected_vertex_pos(r: &Remesher<'_>, vid: i32) -> ([f64; 3], bool) {
    if r.base.vertex_is_constrained(vid) {
        return (r.mesh().vertex(vid), false);
    }
    if r.vertex_blocks_projection(vid) {
        return (r.mesh().vertex(vid), false);
    }
    let curpos = r.mesh().vertex(vid);
    match r.projection_target() {
        Some(target) => (target.project(curpos, vid), true),
        // Only called with a target set; without one nothing moves.
        None => (curpos, false),
    }
}
