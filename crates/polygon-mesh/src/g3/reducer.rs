//! geometry3Sharp `Reducer` (`mesh/Reducer.cs`): mesh simplification by Garland & Heckbert's
//! Quadric Error Metric. Each vertex carries the area-weighted quadric of its triangles' planes
//! (`quadric_error.rs`); every edge is queued (`index_priority_queue.rs`) by the error of
//! collapsing it to the point that minimizes the sum of its two vertices' quadrics, and the
//! cheapest edges are collapsed - through `MeshRefinerBase`'s constraint and flip checks
//! (`mesh_refiner_base.rs`) and `DMesh3::collapse_edge` - until the triangle (or vertex) count
//! reaches its target. With a projection target (`mesh_projection_target.rs`) the collapsed
//! points are projected back onto the original surface, inline or after the pass. MatterCAD's
//! Reduce operation (`DecimateObject3D`) runs it through [`crate::Mesh::reduced_to_triangle_count`].
//!
//! C#'s operation order is kept, and the initial queue is filled in the order .NET's
//! `Array.Sort(edgeErrors, indices)` leaves tied errors in (`vector_math::cs_sort`), since
//! the order ties are queued in decides which edges collapse. The parallel loops of C#
//! (`gParallel`) write disjoint slots and run sequentially here. Not ported: the profiling
//! stopwatches (`ENABLE_PROFILING`), the debugger hooks (`DebugEdges`, `ENABLE_DEBUG_CHECKS`), and
//! the subclass hooks no ported type overrides (`OnEdgeCollapse`, `start_edges`/`next_edge`, which
//! the Reducer itself never calls). g3's `Cancelled()` is always false on this path (see
//! `mesh_refiner_base.rs`), so its checks are left out.

use std::cmp::Ordering;

use super::dmesh3::{DMesh3, MeshResult, INVALID_ID};
use super::dmesh3_queries::find_tri_other_vtx;
use super::index_priority_queue::IndexPriorityQueue;
use super::mesh_constraints::{EdgeConstraint, ProjectionTarget};
use super::mesh_refiner_base::MeshRefinerBase;
use super::quadric_error::QuadricError;
use super::vector3d::{add, cross, distance_squared, scale, sub, EPSILON};

/// C# `Reducer.TargetProjectionMode`: how a projection target is used.
#[allow(dead_code)] // `NoProjection` is geometry3Sharp API no ported caller selects
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

/// C# `Reducer.TargetModes`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TargetModes {
    TriangleCount,
    VertexCount,
    MinEdgeLength,
}

/// C# `Reducer.ProcessResult`.
#[allow(dead_code)] // the variants C# names but no path here returns are kept for parity
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

/// C# `Reducer.QEdge`: an edge's summed quadric and its collapse point.
#[derive(Debug, Clone, Copy, Default)]
struct QEdge {
    q: QuadricError,
    collapse_pt: [f64; 3],
}

/// C# `Reducer`.
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
}

/// C# `MathUtil.FastNormalArea`: the unit normal and the area of triangle `v1 v2 v3`.
fn fast_normal_area(v1: [f64; 3], v2: [f64; 3], v3: [f64; 3]) -> ([f64; 3], f64) {
    let edge1 = sub(v2, v1);
    let edge2 = sub(v3, v1);
    let v_cross = cross(edge1, edge2);
    // C# `Vector3d.Normalize()`: normalizes in place and returns the length (zero, with a zero
    // vector, at or below `MathUtil.Epsilon`).
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

/// .NET `Math.Min(double, double)`: NaN if either is NaN.
fn cs_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if b < a {
        b
    } else {
        a
    }
}

/// .NET `Array.Sort(float[] keys, int[] items)`: NaN keys moved to the front in order
/// (`SortUtils.MoveNansToFront`), then .NET's introsort over the rest.
fn cs_sort_keys_and_items(keys: &mut [f32], items: &mut [i32]) {
    let mut pairs: Vec<(f32, i32)> = keys.iter().copied().zip(items.iter().copied()).collect();
    let mut left = 0;
    for i in 0..pairs.len() {
        if pairs[i].0.is_nan() {
            pairs.swap(left, i);
            left += 1;
        }
    }
    vector_math::cs_sort::introspective_sort(&mut pairs[left..], &mut |a, b| {
        a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal)
    });
    for (i, (k, v)) in pairs.into_iter().enumerate() {
        keys[i] = k;
        items[i] = v;
    }
}

impl<'a, 't> Reducer<'a, 't> {
    /// C# `new Reducer(mesh)`.
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
        }
    }

    /// C# `SetProjectionTarget`.
    pub(crate) fn set_projection_target(&mut self, target: &'t dyn ProjectionTarget) {
        self.target = Some(target);
    }

    /// C# `Mesh`.
    pub(crate) fn mesh(&self) -> &DMesh3 {
        self.base.mesh
    }

    fn enable_inline_projection(&self) -> bool {
        self.projection_mode == TargetProjectionMode::Inline
    }

    /// C# `DoReduce`: collapse the cheapest edges until the target is reached.
    pub(crate) fn do_reduce(&mut self) {
        if self.base.mesh.triangle_count() == 0 {
            // badness if we don't catch this...
            return;
        }

        self.precompute(false);
        self.initialize_vertex_quadrics();
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

    /// C# `ReduceToTriangleCount`.
    pub(crate) fn reduce_to_triangle_count(&mut self, n_count: i32) {
        self.reduce_mode = TargetModes::TriangleCount;
        self.target_count = n_count.max(1);
        self.min_edge_length = f64::MAX;
        self.do_reduce();
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `ReduceToVertexCount`.
    pub(crate) fn reduce_to_vertex_count(&mut self, n_count: i32) {
        self.reduce_mode = TargetModes::VertexCount;
        self.target_count = n_count.max(3);
        self.min_edge_length = f64::MAX;
        self.do_reduce();
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `ReduceToEdgeLength`.
    pub(crate) fn reduce_to_edge_length(&mut self, min_edge_len: f64) {
        self.reduce_mode = TargetModes::MinEdgeLength;
        self.target_count = 1;
        self.min_edge_length = min_edge_len;
        self.do_reduce();
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `FastCollapsePass`: collapse every interior edge shorter than `f_min_edge_length` to
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

    /// C# `InitializeVertexQuadrics`: each vertex's quadric is the area-weighted sum of its
    /// triangles' plane quadrics.
    fn initialize_vertex_quadrics(&mut self) {
        let mesh = &*self.base.mesh;
        let nt = mesh.max_triangle_id() as usize;
        let mut tri_quadrics = vec![QuadricError::ZERO; nt];
        let mut tri_areas = vec![0.0f64; nt];
        for tid in 0..nt {
            if mesh.is_triangle(tid as i32) {
                // C# `GetTriInfo`: the centroid, then `FastNormalArea`.
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

    /// C# `InitializeQueue`: score every edge, then queue them in error order (a sorted insert
    /// is faster).
    fn initialize_queue(&mut self) {
        let max_eid = self.base.mesh.max_edge_id();
        let n = max_eid as usize;

        self.edge_quadrics = vec![QEdge::default(); n];
        self.edge_queue = IndexPriorityQueue::new(max_eid);
        let mut edge_errors = vec![0.0f32; n];

        for eid in 0..max_eid {
            if self.base.mesh.is_edge(eid) {
                let [a, b] = self.base.mesh.get_edge_v(eid);
                let q = QuadricError::sum(
                    &self.vert_quadrics[a as usize],
                    &self.vert_quadrics[b as usize],
                );
                let opt = self.optimal_point(eid, &q, a, b);
                edge_errors[eid as usize] = q.evaluate(opt) as f32;
                self.edge_quadrics[eid as usize] = QEdge {
                    q,
                    collapse_pt: opt,
                };
            }
        }

        // sorted pq insert is faster, so sort edge errors array and index map
        let mut indices: Vec<i32> = (0..max_eid).collect();
        cs_sort_keys_and_items(&mut edge_errors, &mut indices);

        // now do inserts
        for (i, &eid) in indices.iter().enumerate() {
            if self.base.mesh.is_edge(eid) {
                self.edge_queue.insert(eid, edge_errors[i]);
            }
        }
    }

    /// C# `OptimalPoint`: the point that minimizes quadric error `q` for edge `[ea, eb]`.
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

        // [TODO] if we have constraints, we should apply them here, for same reason as bdry
        // above...

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
        // C# `MathUtil.Min(fa, fb, fc)` = `Math.Min(fa, Math.Min(fb, fc))`
        let m = cs_min(fa, cs_min(fb, fc));
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

    /// C# `UpdateNeighbours`: re-score every edge in `vid`'s one-ring.
    fn update_neighbours(&mut self, vid: i32) {
        for eid in self.base.mesh.vtx_edges(vid) {
            let [a, b] = self.base.mesh.get_edge_v(eid);
            let q = QuadricError::sum(
                &self.vert_quadrics[a as usize],
                &self.vert_quadrics[b as usize],
            );
            let opt = self.optimal_point(eid, &q, a, b);
            let err = q.evaluate(opt);
            self.edge_quadrics[eid as usize] = QEdge {
                q,
                collapse_pt: opt,
            };
            if self.edge_queue.contains(eid) {
                self.edge_queue.update(eid, err as f32);
            } else {
                self.edge_queue.insert(eid, err as f32);
            }
        }
    }

    /// C# `Reproject`.
    fn reproject(&mut self) {
        if self.target.is_some() && self.projection_mode == TargetProjectionMode::AfterRefinement {
            self.full_projection_pass();
        }
    }

    /// C# `Precompute`: which vertices are on the boundary (none if the mesh is known closed).
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

    /// C# `CollapseEdge`: collapse `edge_id` with its new vertex at `v_new_pos` (or at a fixed or
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
        if !b_can_collapse {
            return (ProcessResult::IgnoredConstrained, collapse_to_v);
        }

        // if we have a boundary, we want to collapse to boundary
        if self.preserve_boundary_shape && self.have_boundary {
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

        // lots of cases where we cannot collapse, but we should just let mesh sort that out,
        // right?
        let mut ret_val = ProcessResult::FailedOpNotSuccessful;
        let (result, collapse_info) = self.base.mesh.collapse_edge(i_keep, i_collapse);
        if result == MeshResult::Ok {
            collapse_to_v = i_keep;
            self.base.mesh.set_vertex(i_keep, v_new_pos);
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

    /// C# `get_projected_collapse_position`: the new vertex's position under `vid`'s vertex
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

    /// C# `FullProjectionPass`: project every unconstrained vertex onto the target.
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
    /// The geometry of MatterCAD's `DecimateObject3D.Reduce(inMesh, targetCount)`: `ToDMesh3`,
    /// a `Reducer` reducing to `target_count` triangles - with "Maintain Surface"
    /// (`maintain_surface`), projecting every collapse inline onto a copy of the original mesh
    /// through its `DMeshAABBTree3` - then `ToMesh`. The caller applies Reduce's floor of 4.
    pub fn reduced_to_triangle_count(&self, target_count: i32, maintain_surface: bool) -> Self {
        let mut mesh = self.to_dmesh3();
        if maintain_surface {
            let original = mesh.clone();
            let tree = super::DMeshAABBTree3::new(&original);
            let target = super::mesh_projection_target::MeshProjectionTarget::new(&tree);
            let mut reducer = Reducer::new(&mut mesh);
            reducer.set_projection_target(&target);
            reducer.projection_mode = TargetProjectionMode::Inline;
            reducer.reduce_to_triangle_count(target_count);
            reducer.mesh().to_mesh()
        } else {
            let mut reducer = Reducer::new(&mut mesh);
            reducer.reduce_to_triangle_count(target_count);
            reducer.mesh().to_mesh()
        }
    }
}
