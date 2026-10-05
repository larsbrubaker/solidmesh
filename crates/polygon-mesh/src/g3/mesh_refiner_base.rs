//! geometry3Sharp's `MeshRefinerBase` (`mesh/MeshRefinerBase.cs`): the mesh and constraint
//! set a refiner edits, and the constraint-aware collapse and flip checks the `Remesher`
//! (`remesher.rs`) runs per edge. g3's `Progress`/`Cancelled()` is not ported: nothing that
//! reaches the remesher sets a `ProgressCancel`, so `Cancelled()` is always false.

use super::dmesh3::{DMesh3, INVALID_ID};
use super::dmesh3_queries::orient_tri_edge;
use super::mesh_constraints::{same_target, MeshConstraints, VertexConstraint};
use super::vector3d::{clamp, cross, dot, fast_normal_direction, normalized, sub};

/// C# `MeshRefinerBase`.
pub(crate) struct MeshRefinerBase<'a> {
    pub(crate) mesh: &'a mut DMesh3,
    pub(crate) constraints: Option<MeshConstraints>,
    /// If true, two Fixed vertices with the same valid set id are treated as not fixed, so
    /// their edge may collapse.
    pub(crate) allow_collapse_fixed_verts_with_same_set_id: bool,
    edge_flip_tol: f64,
}

impl<'a> MeshRefinerBase<'a> {
    /// C# `new MeshRefinerBase(mesh)`.
    pub(crate) fn new(mesh: &'a mut DMesh3) -> Self {
        Self {
            mesh,
            constraints: None,
            allow_collapse_fixed_verts_with_same_set_id: true,
            edge_flip_tol: 0.0,
        }
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `EdgeFlipTolerance` getter: a normal-pair dot below this counts as a flip.
    pub(crate) fn edge_flip_tolerance(&self) -> f64 {
        self.edge_flip_tol
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `EdgeFlipTolerance` setter, clamped to [-1, 1].
    pub(crate) fn set_edge_flip_tolerance(&mut self, value: f64) {
        self.edge_flip_tol = clamp(value, -1.0, 1.0);
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `SetExternalConstraints`.
    pub(crate) fn set_external_constraints(&mut self, cons: MeshConstraints) {
        self.constraints = Some(cons);
    }

    /// C# `edge_flip_metric`.
    fn edge_flip_metric(&self, n0: [f64; 3], n1: [f64; 3]) -> f64 {
        if self.edge_flip_tol == 0.0 {
            dot(n0, n1)
        } else {
            dot(normalized(n0), normalized(n1))
        }
    }

    /// C# `collapse_creates_flip_or_invalid`: would moving `vid` to `newv` (collapsing it
    /// with `vother`) flip a one-ring normal of `vid` or break the link condition? Only
    /// `vid`'s one-ring is checked; call it twice with the roles swapped.
    pub(crate) fn collapse_creates_flip_or_invalid(
        &self,
        vid: i32,
        vother: i32,
        newv: [f64; 3],
        tc: i32,
        td: i32,
    ) -> bool {
        for tid in self.mesh.vtx_triangles(vid) {
            if tid == tc || tid == td {
                continue;
            }

            let curt = self.mesh.get_triangle(tid);
            if curt[0] == vother || curt[1] == vother || curt[2] == vother {
                return true; // invalid nbrhood for collapse
            }

            let va = self.mesh.vertex(curt[0]);
            let vb = self.mesh.vertex(curt[1]);
            let vc = self.mesh.vertex(curt[2]);
            let ncur = cross(sub(vb, va), sub(vc, va));
            let sign = if curt[0] == vid {
                let nnew = cross(sub(vb, newv), sub(vc, newv));
                self.edge_flip_metric(ncur, nnew)
            } else if curt[1] == vid {
                let nnew = cross(sub(newv, va), sub(vc, va));
                self.edge_flip_metric(ncur, nnew)
            } else if curt[2] == vid {
                let nnew = cross(sub(vb, va), sub(newv, va));
                self.edge_flip_metric(ncur, nnew)
            } else {
                // C# throws "should never be here!": vtx_triangles only yields triangles
                // that contain vid.
                unreachable!("vtx_triangles yielded a triangle without its vertex")
            };

            if sign <= self.edge_flip_tol {
                return true;
            }
        }
        false
    }

    /// C# `flip_inverts_normals`: does flipping `[a,b]` (between `t0` and its neighbour, with
    /// opposite vertices `c`, `d`) reverse any old/new normal pair?
    pub(crate) fn flip_inverts_normals(&self, a: i32, b: i32, c: i32, d: i32, t0: i32) -> bool {
        let v_c = self.mesh.vertex(c);
        let v_d = self.mesh.vertex(d);
        let tri_v = self.mesh.get_triangle(t0);
        let mut oa = a;
        let mut ob = b;
        orient_tri_edge(&mut oa, &mut ob, &tri_v);
        let v_oa = self.mesh.vertex(oa);
        let v_ob = self.mesh.vertex(ob);
        let n0 = fast_normal_direction(v_oa, v_ob, v_c);
        let n1 = fast_normal_direction(v_ob, v_oa, v_d);
        let f0 = fast_normal_direction(v_c, v_d, v_ob);
        if self.edge_flip_metric(n0, f0) <= self.edge_flip_tol
            || self.edge_flip_metric(n1, f0) <= self.edge_flip_tol
        {
            return true;
        }

        let f1 = fast_normal_direction(v_d, v_c, v_oa);
        if self.edge_flip_metric(n0, f1) <= self.edge_flip_tol
            || self.edge_flip_metric(n1, f1) <= self.edge_flip_tol
        {
            return true;
        }

        false
    }

    /// C# `can_collapse_constraints`: may edge `[a,b]` collapse under the constraints? The
    /// second value is C#'s `collapse_to` (the vertex to keep, or -1).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn can_collapse_constraints(
        &self,
        eid: i32,
        a: i32,
        b: i32,
        c: i32,
        d: i32,
        tc: i32,
        td: i32,
    ) -> (bool, i32) {
        let Some(constraints) = &self.constraints else {
            return (true, -1);
        };

        let (b_vtx, collapse_to) = self.can_collapse_vtx(eid, a, b);
        if !b_vtx {
            return (false, collapse_to);
        }

        // when we lose a vtx in a collapse, we also lose two edges [iCollapse,c] and
        // [iCollapse,d]. If either of those edges is constrained, we would lose that
        // constraint. This would be bad.
        let i_collapse = if collapse_to == a { b } else { a };
        if c != INVALID_ID {
            let ec = self.mesh.find_edge_from_tri(i_collapse, c, tc);
            if !constraints.get_edge_constraint(ec).is_unconstrained() {
                return (false, collapse_to);
            }
        }
        if d != INVALID_ID {
            let ed = self.mesh.find_edge_from_tri(i_collapse, d, td);
            if !constraints.get_edge_constraint(ed).is_unconstrained() {
                return (false, collapse_to);
            }
        }

        (true, collapse_to)
    }

    /// C# `can_collapse_vtx`: resolves the vertex constraints of edge `[a,b]`. If `a` or `b`
    /// is constrained the collapse keeps that vertex, returned as `collapse_to` (else -1).
    pub(crate) fn can_collapse_vtx(&self, eid: i32, a: i32, b: i32) -> (bool, i32) {
        let Some(constraints) = &self.constraints else {
            return (true, -1);
        };

        let ca = constraints.get_vertex_constraint(a);
        let cb = constraints.get_vertex_constraint(b);

        // no constraint at all
        if !ca.fixed && !cb.fixed && ca.target.is_none() && cb.target.is_none() {
            return (true, -1);
        }

        // handle a or b fixed
        if ca.fixed && !cb.fixed {
            // if b is fixed to a target, and it is different than a's target, we can't collapse
            if cb.target.is_some() && !same_target(&cb.target, &ca.target) {
                return (false, -1);
            }
            return (true, a);
        }
        if cb.fixed && !ca.fixed {
            if ca.target.is_some() && !same_target(&ca.target, &cb.target) {
                return (false, -1);
            }
            return (true, b);
        }
        // if both fixed, and options allow, treat this edge as unconstrained (eg collapse to
        // midpoint)
        if self.allow_collapse_fixed_verts_with_same_set_id
            && ca.fixed_set_id >= 0
            && ca.fixed_set_id == cb.fixed_set_id
        {
            return (true, -1);
        }

        // handle a or b w/ target
        if ca.target.is_some() && cb.target.is_none() {
            return (true, a);
        }
        if cb.target.is_some() && ca.target.is_none() {
            return (true, b);
        }
        // if both vertices are on the same target, and the edge is on that target, then we can
        // collapse to either and use the midpoint (which will be projected to the target).
        // *However*, if the edge is not on the same target, then we cannot collapse because we
        // would be changing the constraint topology!
        if cb.target.is_some() && ca.target.is_some() && same_target(&ca.target, &cb.target) {
            let edge_target = constraints.get_edge_constraint(eid).target;
            if same_target(&edge_target, &ca.target) {
                return (true, -1);
            }
        }

        (false, -1)
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `vertex_is_fixed`.
    pub(crate) fn vertex_is_fixed(&self, vid: i32) -> bool {
        self.constraints
            .as_ref()
            .is_some_and(|c| c.get_vertex_constraint(vid).fixed)
    }

    /// C# `vertex_is_constrained`: fixed or on a target.
    pub(crate) fn vertex_is_constrained(&self, vid: i32) -> bool {
        if let Some(constraints) = &self.constraints {
            let vc = constraints.get_vertex_constraint(vid);
            if vc.fixed || vc.target.is_some() {
                return true;
            }
        }
        false
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `get_vertex_constraint(vid)`.
    pub(crate) fn get_vertex_constraint(&self, vid: i32) -> VertexConstraint {
        match &self.constraints {
            Some(c) => c.get_vertex_constraint(vid),
            None => VertexConstraint::unconstrained(),
        }
    }
}
