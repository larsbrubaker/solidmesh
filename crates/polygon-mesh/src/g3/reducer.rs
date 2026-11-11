//! The reducer (derived from geometry3Sharp): mesh simplification by Garland & Heckbert's
//! Quadric Error Metric. Each vertex carries the area-weighted quadric of its triangles' planes
//! (`quadric_error.rs`); every edge is queued (`index_priority_queue.rs`) by the error of
//! collapsing it to the point that minimizes the sum of its two vertices' quadrics, and the
//! cheapest edges are collapsed - through `MeshRefinerBase`'s constraint and flip checks
//! (`mesh_refiner_base.rs`) and `DMesh3::collapse_edge` - until the triangle (or vertex) count
//! reaches its target. With a projection target (`mesh_projection_target.rs`) the collapsed
//! points are projected back onto the original surface, inline or after the pass. MatterCAD's
//! Reduce operation (`DecimateObject3D`) runs it through [`crate::Mesh::reduced_to_triangle_count`].
//!
//! The arithmetic order is fixed, and the initial queue is filled from an introsort of the edge
//! errors (`solidmesh_math::fixed_tie_sort`), whose order for tied errors is fixed too: the order ties
//! are queued in decides which edges collapse, so Reduce gives the same mesh on every run. The
//! per-vertex and per-edge setup loops write disjoint slots and run sequentially. There is no
//! cancel check: nothing that runs the reducer interrupts it.
//!
//! A mesh with face textures or face colours carries them through every collapse
//! (`reducer_face_data.rs`): the seams between its regions become constraints, and the child
//! module `reducer_seams.rs` places collapses by the seam rules and scores edges inside a textured
//! region with the texture-aware attribute quadric (`attribute_quadric.rs`), whose optimum also
//! gives the kept vertex its UV. A mesh with neither runs with no constraints at all, exactly as
//! before. Some textured shapes cannot reduce everywhere: at a lat-long sphere's poles every
//! spoke is a UV seam (each face has its own u at the pole), so the pole fans stay as they are.

use std::cmp::Ordering;

use super::dmesh3::{DMesh3, MeshResult, INVALID_ID};
use super::dmesh3_queries::find_tri_other_vtx;
use super::index_priority_queue::IndexPriorityQueue;
use super::mesh_constraints::{EdgeConstraint, ProjectionTarget};
use super::mesh_refiner_base::MeshRefinerBase;
use super::quadric_error::QuadricError;
use super::reducer_face_data::ReducerFaceData;
use seams::parameter_along;

#[path = "reducer_seams.rs"]
mod seams;
use super::vector3d::{add, cross, distance_squared, scale, sub, EPSILON};

/// How a projection target is used.
#[allow(dead_code)] // no caller selects `NoProjection` yet
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TargetProjectionMode {
    /// Disable projection.
    NoProjection,
    /// Do all projection after reducing.
    AfterRefinement,
    /// Projection is computed before evaluating quadrics, so the QEM error of the result is
    /// evaluated properly.
    Inline,
}

/// What the reduce counts toward its target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TargetModes {
    TriangleCount,
    VertexCount,
    MinEdgeLength,
}

/// The outcome of one edge collapse attempt.
#[allow(dead_code)] // the full set of outcomes, though no path returns some of them yet
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ProcessResult {
    OkCollapsed,
    IgnoredCannotCollapse,
    IgnoredEdgeIsFullyConstrained,
    IgnoredEdgeTooLong,
    IgnoredConstrained,
    IgnoredCreatesFlip,
    FailedOpNotSuccessful,
    FailedNotAnEdge,
}

/// An edge's summed quadric and its collapse point.
#[derive(Debug, Clone, Copy, Default)]
struct QEdge {
    q: QuadricError,
    collapse_pt: [f64; 3],
}

/// The quadric-error reducer over one mesh.
pub(crate) struct Reducer<'a, 't> {
    pub(crate) base: MeshRefinerBase<'a>,
    target: Option<&'t dyn ProjectionTarget>,
    /// If true, collapsed vertices go where the quadric error is minimal; if false, to edge
    /// midpoints. Note: midpoints are *significantly* slower, because many more collapses would
    /// flip a triangle and are rejected.
    pub(crate) minimize_quadric_position_error: bool,
    /// If true, boundary vertices are kept on the boundary.
    pub(crate) preserve_boundary_shape: bool,
    /// How the projection target (if set) is used.
    pub(crate) projection_mode: TargetProjectionMode,
    min_edge_length: f64,
    target_count: i32,
    reduce_mode: TargetModes,
    vert_quadrics: Vec<QuadricError>,
    edge_quadrics: Vec<QEdge>,
    edge_queue: IndexPriorityQueue,
    have_boundary: bool,
    is_boundary_vtx_cache: Vec<bool>,
    /// The face textures and colours carried through the collapses, when the mesh has any.
    face_data: Option<ReducerFaceData>,
    /// Per edge id, the collapse point's UV when the attribute quadric placed it
    /// (`reducer_seams.rs`); empty on a mesh without face data.
    edge_uvs: Vec<Option<[f64; 2]>>,
}

/// The unit normal and the area of triangle `v1 v2 v3`.
fn fast_normal_area(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> ([f64; 3], f64) {
    let edge1 = sub(v2, v1);
    let edge2 = sub(v3, v1);
    let v_cross = cross(edge1, edge2);
    // Normalizes in place and returns the length (zero, with a zero
    // vector, at or below the double epsilon).
    let length =
        (v_cross[0] * v_cross[0] + v_cross[1] * v_cross[1] + v_cross[2] * v_cross[2]).sqrt();
    if length > EPSILON {
        let inv_length = 1.0 / length;
        let n = [
            v_cross[0] * inv_length,
            v_cross[1] * inv_length,
            v_cross[2] * inv_length,
        ];
        (n, 0.5 * length)
    } else {
        ([0.0; 3], 0.0)
    }
}

/// The smaller of two doubles, NaN if either is NaN (so a NaN error is never mistaken for
/// the cheapest).
fn min_or_nan(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if b < a {
        b
    } else {
        a
    }
}

/// Sorts `items` by `keys`: NaN keys move to the front in order, then an introsort orders
/// the rest, with its fixed placement of tied keys.
fn sort_keys_and_items_nan_first(keys: &mut [f32], items: &mut [i32]) {
    let mut pairs: Vec<(f32, i32)> = keys.iter().copied().zip(items.iter().copied()).collect();
    let mut left = 0;
    for i in 0..pairs.len() {
        if pairs[i].0.is_nan() {
            pairs.swap(left, i);
            left += 1;
        }
    }
    solidmesh_math::fixed_tie_sort::introspective_sort(&mut pairs[left..], &mut |a, b| {
        a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal)
    });
    for (i, (k, v)) in pairs.into_iter().enumerate() {
        keys[i] = k;
        items[i] = v;
    }
}

impl<'a, 't> Reducer<'a, 't> {
    /// A reducer over `mesh`, with no target and no projection.
    pub(crate) fn new(mesh: &'a mut DMesh3) -> Self {
        Self {
            base: MeshRefinerBase::new(mesh),
            target: None,
            minimize_quadric_position_error: true,
            preserve_boundary_shape: true,
            projection_mode: TargetProjectionMode::AfterRefinement,
            min_edge_length: f64::MAX,
            target_count: i32::MAX,
            reduce_mode: TargetModes::TriangleCount,
            vert_quadrics: Vec::new(),
            edge_quadrics: Vec::new(),
            edge_queue: IndexPriorityQueue::new(0),
            have_boundary: false,
            is_boundary_vtx_cache: Vec::new(),
            face_data: None,
            edge_uvs: Vec::new(),
        }
    }

    /// Carries `data` (the mesh's face textures and colours) through the reduce: its seams
    /// become the refiner's constraints.
    pub(crate) fn attach_face_data(&mut self, mut data: ReducerFaceData) {
        let constraints = data.seam_constraints(self.base.mesh);
        data.init_attributes(self.base.mesh);
        self.base.constraints = Some(constraints);
        self.face_data = Some(data);
    }

    /// Edge `[a, b]`'s collapse point, its error and (when the attribute quadric placed it) its
    /// UV. A mesh without face data scores by position alone.
    fn score_edge(
        &self,
        eid: i32,
        q: &QuadricError,
        a: i32,
        b: i32,
    ) -> ([f64; 3], f64, Option<[f64; 2]>) {
        if self.face_data.is_some() {
            return self.score_seamed_edge(eid, q, a, b);
        }
        let opt = self.optimal_point(eid, q, a, b);
        (opt, q.evaluate(opt), None)
    }

    /// The face data carried through the reduce, if any.
    pub(crate) fn face_data(&self) -> Option<&ReducerFaceData> {
        self.face_data.as_ref()
    }

    /// Sets the surface collapsed points are projected back onto.
    pub(crate) fn set_projection_target(&mut self, target: &'t dyn ProjectionTarget) {
        self.target = Some(target);
    }

    /// The mesh being reduced.
    pub(crate) fn mesh(&self) -> &DMesh3 {
        self.base.mesh
    }

    fn enable_inline_projection(&self) -> bool {
        self.projection_mode == TargetProjectionMode::Inline
    }

    /// Collapse the cheapest edges until the target is reached.
    pub(crate) fn do_reduce(&mut self) {
        if self.base.mesh.triangle_count() == 0 {
            // badness if we don't catch this...
            return;
        }

        self.precompute(false);
        self.initialize_vertex_quadrics();
        if self.face_data.is_some() {
            self.add_seam_planes();
        }
        self.initialize_queue();

        while self.edge_queue.count() > 0 {
            // termination criteria
            if self.reduce_mode == TargetModes::VertexCount {
                if self.base.mesh.vertex_count() as i64 <= self.target_count as i64 {
                    break;
                }
            } else if self.base.mesh.triangle_count() as i64 <= self.target_count as i64 {
                break;
            }

            let eid = self.edge_queue.dequeue();
            if !self.base.mesh.is_edge(eid) {
                continue;
            }

            let collapse_pt = self.edge_quadrics[eid as usize].collapse_pt;
            let (result, v_kept) = self.collapse_edge(eid, collapse_pt);
            if result == ProcessResult::OkCollapsed {
                self.vert_quadrics[v_kept as usize] = self.edge_quadrics[eid as usize].q;
                self.update_neighbours(v_kept);
            }
        }

        self.reproject();
    }

    /// Reduces until at most `n_count` triangles remain (at least one).
    pub(crate) fn reduce_to_triangle_count(&mut self, n_count: i32) {
        self.reduce_mode = TargetModes::TriangleCount;
        self.target_count = n_count.max(1);
        self.min_edge_length = f64::MAX;
        self.do_reduce();
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Reduces until at most `n_count` vertices remain (at least three).
    pub(crate) fn reduce_to_vertex_count(&mut self, n_count: i32) {
        self.reduce_mode = TargetModes::VertexCount;
        self.target_count = n_count.max(3);
        self.min_edge_length = f64::MAX;
        self.do_reduce();
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Collapses edges shorter than `min_edge_len`, cheapest first.
    pub(crate) fn reduce_to_edge_length(&mut self, min_edge_len: f64) {
        self.reduce_mode = TargetModes::MinEdgeLength;
        self.target_count = 1;
        self.min_edge_length = min_edge_len;
        self.do_reduce();
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Collapse every interior edge shorter than `f_min_edge_length` to
    /// its midpoint, up to `n_rounds` times or until a round collapses nothing.
    pub(crate) fn fast_collapse_pass(
        &mut self,
        f_min_edge_length: f64,
        n_rounds: i32,
        mesh_is_closed_hint: bool,
    ) {
        if self.base.mesh.triangle_count() == 0 {
            // badness if we don't catch this...
            return;
        }

        self.min_edge_length = f_min_edge_length;
        let min_sqr = self.min_edge_length * self.min_edge_length;

        // we don't collapse on the boundary
        self.have_boundary = false;

        self.precompute(mesh_is_closed_hint);

        let n = self.base.mesh.max_edge_id();
        for _ in 0..n_rounds {
            let mut num_last_pass = 0;
            for eid in 0..n {
                if !self.base.mesh.is_edge(eid) || self.base.mesh.is_boundary_edge(eid) {
                    continue;
                }

                let [a, b] = self.base.mesh.get_edge_v(eid);
                let va = self.base.mesh.vertex(a);
                let vb = self.base.mesh.vertex(b);
                if distance_squared(va, vb) > min_sqr {
                    continue;
                }

                let midpoint = scale(add(va, vb), 0.5);
                let (result, _) = self.collapse_edge(eid, midpoint);
                if result == ProcessResult::OkCollapsed {
                    num_last_pass += 1;
                }
            }

            if num_last_pass == 0 {
                // converged
                break;
            }
        }

        self.reproject();
    }

    /// Each vertex's quadric is the area-weighted sum of its
    /// triangles' plane quadrics.
    fn initialize_vertex_quadrics(&mut self) {
        let mesh = &*self.base.mesh;
        let nt = mesh.max_triangle_id() as usize;
        let mut tri_quadrics = vec![QuadricError::ZERO; nt];
        let mut tri_areas = vec![0.0f64; nt];
        for tid in 0..nt {
            if mesh.is_triangle(tid as i32) {
                // The centroid, then the unit normal and area.
                let [v0, v1, v2] = mesh.get_tri_vertices(tid);
                let c = scale(add(add(v0, v1), v2), 1.0 / 3.0);
                let (n, area) = fast_normal_area(v0, v1, v2);
                tri_areas[tid] = area;
                tri_quadrics[tid] = QuadricError::from_plane(n, c);
            }
        }

        let nv = mesh.max_vertex_id() as usize;
        let mut vert_quadrics = vec![QuadricError::ZERO; nv];
        for (vid, quadric) in vert_quadrics.iter_mut().enumerate() {
            if mesh.is_vertex(vid as i32) {
                for tid in mesh.vtx_triangles(vid as i32) {
                    quadric.add(tri_areas[tid as usize], &tri_quadrics[tid as usize]);
                }
            }
        }
        self.vert_quadrics = vert_quadrics;
    }

    /// Score every edge, then queue them in error order (a sorted insert
    /// is faster).
    fn initialize_queue(&mut self) {
        let max_eid = self.base.mesh.max_edge_id();
        let n = max_eid as usize;

        self.edge_quadrics = vec![QEdge::default(); n];
        if self.face_data.is_some() {
            self.edge_uvs = vec![None; n];
        }
        self.edge_queue = IndexPriorityQueue::new(max_eid);
        let mut edge_errors = vec![0.0f32; n];

        for eid in 0..max_eid {
            if self.base.mesh.is_edge(eid) {
                let [a, b] = self.base.mesh.get_edge_v(eid);
                let q = QuadricError::sum(
                    &self.vert_quadrics[a as usize],
                    &self.vert_quadrics[b as usize],
                );
                let (opt, err, uv) = self.score_edge(eid, &q, a, b);
                edge_errors[eid as usize] = err as f32;
                self.edge_quadrics[eid as usize] = QEdge {
                    q,
                    collapse_pt: opt,
                };
                if let Some(slot) = self.edge_uvs.get_mut(eid as usize) {
                    *slot = uv;
                }
            }
        }

        // sorted pq insert is faster, so sort edge errors array and index map
        let mut indices: Vec<i32> = (0..max_eid).collect();
        sort_keys_and_items_nan_first(&mut edge_errors, &mut indices);

        // now do inserts
        for (i, &eid) in indices.iter().enumerate() {
            if self.base.mesh.is_edge(eid) {
                self.edge_queue.insert(eid, edge_errors[i]);
            }
        }
    }

    /// The point that minimizes quadric error `q` for edge `[ea, eb]`.
    fn optimal_point(&self, eid: i32, q: &QuadricError, ea: i32, eb: i32) -> [f64; 3] {
        let mesh = &*self.base.mesh;
        // if we would like to preserve boundary, we need to know that here so that we properly
        // score these edges
        if self.have_boundary && self.preserve_boundary_shape {
            if mesh.is_boundary_edge(eid) {
                return scale(add(mesh.vertex(ea), mesh.vertex(eb)), 0.5);
            } else if self.is_boundary_v(ea) {
                return mesh.vertex(ea);
            } else if self.is_boundary_v(eb) {
                return mesh.vertex(eb);
            }
        }

        if !self.minimize_quadric_position_error {
            return self.project(scale(add(mesh.vertex(ea), mesh.vertex(eb)), 0.5));
        }

        if let Some(result) = q.optimal_point() {
            return self.project(result);
        }

        // degenerate matrix, evaluate quadric at edge end and midpoints
        // (could do line search here...)
        let va = mesh.vertex(ea);
        let vb = mesh.vertex(eb);
        let c = self.project(scale(add(va, vb), 0.5));
        let fa = q.evaluate(va);
        let fb = q.evaluate(vb);
        let fc = q.evaluate(c);
        // The minimum of the three, NaN-propagating
        let m = min_or_nan(fa, min_or_nan(fb, fc));
        if m == fa {
            va
        } else if m == fb {
            vb
        } else {
            c
        }
    }

    fn project(&self, pos: [f64; 3]) -> [f64; 3] {
        match self.target {
            Some(target) if self.enable_inline_projection() => target.project(pos, -1),
            _ => pos,
        }
    }

    /// Re-score every edge in `vid`'s one-ring.
    fn update_neighbours(&mut self, vid: i32) {
        for eid in self.base.mesh.vtx_edges(vid) {
            let [a, b] = self.base.mesh.get_edge_v(eid);
            let q = QuadricError::sum(
                &self.vert_quadrics[a as usize],
                &self.vert_quadrics[b as usize],
            );
            let (opt, err, uv) = self.score_edge(eid, &q, a, b);
            self.edge_quadrics[eid as usize] = QEdge {
                q,
                collapse_pt: opt,
            };
            if let Some(slot) = self.edge_uvs.get_mut(eid as usize) {
                *slot = uv;
            }
            if self.edge_queue.contains(eid) {
                self.edge_queue.update(eid, err as f32);
            } else {
                self.edge_queue.insert(eid, err as f32);
            }
        }
    }

    /// Projects every vertex back onto the target when projection runs after the pass.
    fn reproject(&mut self) {
        if self.target.is_some() && self.projection_mode == TargetProjectionMode::AfterRefinement {
            self.full_projection_pass();
        }
    }

    /// Which vertices are on the boundary (none if the mesh is known closed).
    fn precompute(&mut self, b_mesh_is_closed: bool) {
        self.have_boundary = false;
        self.is_boundary_vtx_cache = vec![false; self.base.mesh.max_vertex_id() as usize];
        if !b_mesh_is_closed {
            let boundary: Vec<i32> = self.base.mesh.boundary_edge_indices().collect();
            for eid in boundary {
                let [a, b] = self.base.mesh.get_edge_v(eid);
                self.is_boundary_vtx_cache[a as usize] = true;
                self.is_boundary_vtx_cache[b as usize] = true;
                self.have_boundary = true;
            }
        }
    }

    fn is_boundary_v(&self, vid: i32) -> bool {
        self.is_boundary_vtx_cache[vid as usize]
    }

    /// Collapse `edge_id` with its new vertex at `v_new_pos` (or at a fixed or
    /// boundary end), unless a constraint, the edge length or a normal flip forbids it. The
    /// second value is the kept vertex (`INVALID_ID` unless collapsed).
    fn collapse_edge(&mut self, edge_id: i32, mut v_new_pos: [f64; 3]) -> (ProcessResult, i32) {
        let mut collapse_to_v = INVALID_ID;

        let constraint = match &self.base.constraints {
            None => EdgeConstraint::unconstrained(),
            Some(c) => c.get_edge_constraint(edge_id),
        };
        if constraint.no_modifications() || !constraint.can_collapse() {
            return (ProcessResult::IgnoredEdgeIsFullyConstrained, collapse_to_v);
        }

        // look up verts and tris for this edge
        if !self.base.mesh.is_edge(edge_id) {
            return (ProcessResult::FailedNotAnEdge, collapse_to_v);
        }
        let [a, b, t0, t1] = self.base.mesh.get_edge(edge_id);
        let b_is_boundary_edge = t1 == INVALID_ID;

        // look up 'other' verts c (from t0) and d (from t1, if it exists)
        let t0tv = self.base.mesh.get_triangle(t0);
        let c = find_tri_other_vtx(a, b, &t0tv);
        let d = if b_is_boundary_edge {
            INVALID_ID
        } else {
            find_tri_other_vtx(a, b, &self.base.mesh.get_triangle(t1))
        };

        let v_a = self.base.mesh.vertex(a);
        let v_b = self.base.mesh.vertex(b);
        let d_ab = sub(v_a, v_b);
        let edge_len_sqr = d_ab[0] * d_ab[0] + d_ab[1] * d_ab[1] + d_ab[2] * d_ab[2];
        if edge_len_sqr > self.min_edge_length * self.min_edge_length {
            return (ProcessResult::IgnoredEdgeTooLong, collapse_to_v);
        }

        // check if we should collapse, and also find which vertex we should collapse to, in
        // cases where we have constraints/etc
        let (b_can_collapse, mut collapse_to) = self
            .base
            .can_collapse_constraints(edge_id, a, b, c, d, t0, t1);
        if !b_can_collapse || (self.face_data.is_some() && self.seam_refuses(edge_id, a, b)) {
            return (ProcessResult::IgnoredConstrained, collapse_to_v);
        }

        // if we have a boundary, we want to collapse to boundary. A boundary edge whose
        // constraint keeps one end (a pinned seam end on the boundary) collapses along the
        // boundary onto that end, which keeps the boundary as a midpoint collapse does.
        let along_boundary_onto_kept_end =
            collapse_to != -1 && b_is_boundary_edge && self.is_boundary_v(collapse_to);
        if self.preserve_boundary_shape && self.have_boundary && !along_boundary_onto_kept_end {
            if collapse_to != -1
                && ((self.is_boundary_v(b) && collapse_to != b)
                    || (self.is_boundary_v(a) && collapse_to != a))
            {
                return (ProcessResult::IgnoredConstrained, collapse_to_v);
            }
            if self.is_boundary_v(b) {
                collapse_to = b;
            } else if self.is_boundary_v(a) {
                collapse_to = a;
            }
        }

        let mut i_keep = b;
        let mut i_collapse = a;

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

        // check if this collapse will create a normal flip. Also checks for invalid collapse
        // nbrhood, since we are doing one-ring iter anyway.
        if self
            .base
            .collapse_creates_flip_or_invalid(a, b, v_new_pos, t0, t1)
            || self
                .base
                .collapse_creates_flip_or_invalid(b, a, v_new_pos, t0, t1)
        {
            return (ProcessResult::IgnoredCreatesFlip, collapse_to_v);
        }

        // The corner UVs the collapse leaves at the kept vertex, read before the triangles
        // around the removed vertex change.
        let uv_plan = match &self.face_data {
            Some(data) => {
                let (v_keep, v_gone) = if i_keep == b { (v_b, v_a) } else { (v_a, v_b) };
                let onto_kept = collapse_to == i_keep;
                let s = if onto_kept {
                    1.0
                } else {
                    parameter_along(v_gone, v_keep, v_new_pos)
                };
                // The attribute quadric's UV belongs to the point it scored, so it is used only
                // when the collapse goes there.
                let scored_here = self
                    .edge_quadrics
                    .get(edge_id as usize)
                    .is_some_and(|e| e.collapse_pt == v_new_pos);
                let uv = if scored_here {
                    self.edge_uvs.get(edge_id as usize).copied().flatten()
                } else {
                    None
                };
                Some(data.plan_collapse(
                    &*self.base.mesh,
                    edge_id,
                    (i_collapse, i_keep),
                    (t0, t1),
                    s,
                    v_new_pos,
                    uv,
                    onto_kept,
                ))
            }
            None => None,
        };

        // lots of cases where we cannot collapse, but we should just let mesh sort that out,
        // right?
        let mut ret_val = ProcessResult::FailedOpNotSuccessful;
        let (result, collapse_info) = self.base.mesh.collapse_edge(i_keep, i_collapse);
        if result == MeshResult::Ok {
            collapse_to_v = i_keep;
            self.base.mesh.set_vertex(i_keep, v_new_pos);
            if let (Some(data), Some(plan)) = (&mut self.face_data, uv_plan) {
                data.apply_collapse(&*self.base.mesh, i_keep, plan);
                data.merge_attributes(i_keep, i_collapse);
            }
            if let Some(constraints) = &mut self.base.constraints {
                constraints.clear_edge_constraint(edge_id);
                constraints.clear_edge_constraint(collapse_info.e_removed[0]);
                if collapse_info.e_removed[1] != INVALID_ID {
                    constraints.clear_edge_constraint(collapse_info.e_removed[1]);
                }
                constraints.clear_vertex_constraint(i_collapse);
            }
            ret_val = ProcessResult::OkCollapsed;
        }

        (ret_val, collapse_to_v)
    }

    /// The new vertex's position under `vid`'s vertex
    /// constraint. (Inline projection onto the target already happened in `optimal_point`.)
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
        v_new_pos
    }

    /// Project every unconstrained vertex onto the target.
    fn full_projection_pass(&mut self) {
        let Some(target) = self.target else {
            return;
        };
        let vertices: Vec<i32> = self.base.mesh.vertex_indices().collect();
        for vid in vertices {
            if self.base.vertex_is_constrained(vid) {
                continue;
            }
            let curpos = self.base.mesh.vertex(vid);
            let projected = target.project(curpos, vid);
            self.base.mesh.set_vertex(vid, projected);
        }
    }
}

impl crate::Mesh {
    /// The geometry of the Reduce operation (`DecimateObject3D`): the mesh as a `DMesh3`, a
    /// `Reducer` reducing to `target_count` triangles - with "Maintain Surface"
    /// (`maintain_surface`), projecting every collapse inline onto a copy of the original mesh
    /// through its `DMeshAABBTree3` - then back to a `Mesh`. Face textures and face colours come
    /// through: no reduced face straddles a seam between pictures, UV islands or colours, and
    /// each takes its region's picture and colour, with UVs that follow the collapses. The caller
    /// applies Reduce's floor of 4.
    pub fn reduced_to_triangle_count(&self, target_count: i32, maintain_surface: bool) -> Self {
        let (mut mesh, face_data) = self.to_dmesh3_with_face_data();
        let original = if maintain_surface {
            Some(mesh.clone())
        } else {
            None
        };
        let tree = original.as_ref().map(super::DMeshAABBTree3::new);
        let target = tree
            .as_ref()
            .map(super::mesh_projection_target::MeshProjectionTarget::new);
        let mut reducer = Reducer::new(&mut mesh);
        if let Some(target) = &target {
            reducer.set_projection_target(target);
            reducer.projection_mode = TargetProjectionMode::Inline;
        }
        if let Some(data) = face_data {
            reducer.attach_face_data(data);
        }
        reducer.reduce_to_triangle_count(target_count);
        let mut out = reducer.mesh().to_mesh();
        if let Some(data) = reducer.face_data() {
            data.write_faces(reducer.mesh(), &mut out);
        }
        out
    }
}
