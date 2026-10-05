//! geometry3Sharp's `MeshConstraints` (`mesh/MeshConstraints.cs`: `EdgeRefineFlags`,
//! `EdgeConstraint`, `VertexConstraint`, `MeshConstraints`), the `IProjectionTarget` interface
//! (`spatial/SpatialInterfaces.cs`) the constraints and the `Remesher` project onto, and the
//! parts of `MeshConstraintUtil` (`mesh/MeshConstraintUtil.cs`) that `MinimalHoleFill` reaches
//! (`FixAllBoundaryEdges`). The `Remesher` (`remesher*.rs`) reads these per edge and vertex.
//!
//! The constraint maps are only looked up, set and cleared here, never iterated, so a
//! `HashMap` cannot leak an order. `FindConstrainedEdgesBySetID` and `VertexConstraintsItr`,
//! which do iterate, are not reached and are not ported.

// Consumed by `MinimalHoleFill` (mesh repair), which lands in a later slice; until then only the
// tests reach some of it.
#![allow(dead_code)]

use std::collections::HashMap;
use std::rc::Rc;

use super::dmesh3::DMesh3;

/// C# `IProjectionTarget`: something a point can be projected onto.
pub(crate) trait ProjectionTarget {
    /// C# `Project(vPoint, identifier)`.
    fn project(&self, point: [f64; 3], identifier: i32) -> [f64; 3];
}

/// A shared projection target; C# compares targets by reference, which [`same_target`] does.
pub(crate) type TargetRef = Option<Rc<dyn ProjectionTarget>>;

/// C# `a == b` on two `IProjectionTarget` references: both null, or the same object.
pub(crate) fn same_target(a: &TargetRef, b: &TargetRef) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => std::ptr::eq(Rc::as_ptr(a) as *const u8, Rc::as_ptr(b) as *const u8),
        _ => false,
    }
}

/// C# `EdgeRefineFlags` (a `[Flags]` enum).
pub(crate) mod edge_refine_flags {
    pub(crate) const NO_CONSTRAINT: u32 = 0;
    pub(crate) const NO_FLIP: u32 = 1;
    pub(crate) const NO_SPLIT: u32 = 2;
    pub(crate) const NO_COLLAPSE: u32 = 4;
    pub(crate) const FULLY_CONSTRAINED: u32 = NO_FLIP | NO_SPLIT | NO_COLLAPSE;
    /// Avoid 'losing' this edge when a neighbour collapses (the edge itself may collapse).
    pub(crate) const PRESERVE_TOPOLOGY: u32 = 8;
}
use edge_refine_flags as erf;

/// C# `EdgeConstraint`.
#[derive(Clone)]
pub(crate) struct EdgeConstraint {
    refine_flags: u32,
    /// The edge is associated with this target. Only information: edges are not projected
    /// (a `VertexConstraint` must also be set).
    pub(crate) target: TargetRef,
    /// Not a constraint: lets descendants of a constrained input edge be found.
    pub(crate) tracking_set_id: i32,
}

impl EdgeConstraint {
    /// C# `new EdgeConstraint(rflags)`.
    pub(crate) fn new(refine_flags: u32) -> Self {
        Self {
            refine_flags,
            target: None,
            tracking_set_id: -1,
        }
    }

    /// C# `new EdgeConstraint(rflags, target)`.
    pub(crate) fn with_target(refine_flags: u32, target: TargetRef) -> Self {
        Self {
            refine_flags,
            target,
            tracking_set_id: -1,
        }
    }

    /// C# `EdgeConstraint.Unconstrained`.
    pub(crate) fn unconstrained() -> Self {
        Self::new(erf::NO_CONSTRAINT)
    }

    /// C# `EdgeConstraint.NoFlips`.
    pub(crate) fn no_flips() -> Self {
        Self::new(erf::NO_FLIP)
    }

    /// C# `EdgeConstraint.FullyConstrained`.
    pub(crate) fn fully_constrained() -> Self {
        Self::new(erf::FULLY_CONSTRAINED)
    }

    /// C# `CanFlip`.
    pub(crate) fn can_flip(&self) -> bool {
        (self.refine_flags & erf::NO_FLIP) == 0
    }

    /// C# `CanSplit`.
    pub(crate) fn can_split(&self) -> bool {
        (self.refine_flags & erf::NO_SPLIT) == 0
    }

    /// C# `CanCollapse`.
    pub(crate) fn can_collapse(&self) -> bool {
        (self.refine_flags & erf::NO_COLLAPSE) == 0
    }

    /// C# `NoModifications`.
    pub(crate) fn no_modifications(&self) -> bool {
        (self.refine_flags & erf::FULLY_CONSTRAINED) == erf::FULLY_CONSTRAINED
    }

    /// C# `IsUnconstrained`.
    pub(crate) fn is_unconstrained(&self) -> bool {
        self.refine_flags == erf::NO_CONSTRAINT && self.target.is_none()
    }
}

/// C# `VertexConstraint`.
#[derive(Clone)]
pub(crate) struct VertexConstraint {
    pub(crate) fixed: bool,
    /// The `Remesher` may collapse two fixed vertices with the same set id together.
    pub(crate) fixed_set_id: i32,
    /// The vertex is constrained to lie on this target. Fixed and Target are exclusive.
    pub(crate) target: TargetRef,
}

impl VertexConstraint {
    /// C# `VertexConstraint.InvalidSetID`; clients treat any negative id as invalid.
    pub(crate) const INVALID_SET_ID: i32 = -1;

    /// C# `new VertexConstraint(isFixed, setID)`.
    pub(crate) fn new_fixed(is_fixed: bool, set_id: i32) -> Self {
        Self {
            fixed: is_fixed,
            fixed_set_id: set_id,
            target: None,
        }
    }

    /// C# `new VertexConstraint(target)`.
    pub(crate) fn new_target(target: TargetRef) -> Self {
        Self {
            fixed: false,
            fixed_set_id: Self::INVALID_SET_ID,
            target,
        }
    }

    /// C# `VertexConstraint.Unconstrained`.
    pub(crate) fn unconstrained() -> Self {
        Self::new_fixed(false, Self::INVALID_SET_ID)
    }

    /// C# `VertexConstraint.Pinned`.
    pub(crate) fn pinned() -> Self {
        Self::new_fixed(true, Self::INVALID_SET_ID)
    }
}

/// C# `MeshConstraints`.
#[derive(Clone, Default)]
pub(crate) struct MeshConstraints {
    edges: HashMap<i32, EdgeConstraint>,
    vertices: HashMap<i32, VertexConstraint>,
    set_id_counter: i32,
}

impl MeshConstraints {
    /// C# `AllocateSetID`.
    pub(crate) fn allocate_set_id(&mut self) -> i32 {
        let id = self.set_id_counter;
        self.set_id_counter += 1;
        id
    }

    /// C# `HasEdgeConstraint`.
    pub(crate) fn has_edge_constraint(&self, eid: i32) -> bool {
        self.edges.contains_key(&eid)
    }

    /// C# `GetEdgeConstraint`: `Unconstrained` when none is set.
    pub(crate) fn get_edge_constraint(&self, eid: i32) -> EdgeConstraint {
        self.edges
            .get(&eid)
            .cloned()
            .unwrap_or_else(EdgeConstraint::unconstrained)
    }

    /// C# `SetOrUpdateEdgeConstraint`.
    pub(crate) fn set_or_update_edge_constraint(&mut self, eid: i32, ec: EdgeConstraint) {
        self.edges.insert(eid, ec);
    }

    /// C# `ClearEdgeConstraint`.
    pub(crate) fn clear_edge_constraint(&mut self, eid: i32) {
        self.edges.remove(&eid);
    }

    /// C# `HasVertexConstraint`.
    pub(crate) fn has_vertex_constraint(&self, vid: i32) -> bool {
        self.vertices.contains_key(&vid)
    }

    /// C# `GetVertexConstraint(vid)`: `Unconstrained` when none is set.
    pub(crate) fn get_vertex_constraint(&self, vid: i32) -> VertexConstraint {
        self.vertices
            .get(&vid)
            .cloned()
            .unwrap_or_else(VertexConstraint::unconstrained)
    }

    /// C# `GetVertexConstraint(vid, ref vc)`: `None` when none is set (C#'s `TryGetValue`
    /// then leaves `vc` as the default struct: not fixed, set id 0, no target).
    pub(crate) fn try_get_vertex_constraint(&self, vid: i32) -> Option<VertexConstraint> {
        self.vertices.get(&vid).cloned()
    }

    /// C# `SetOrUpdateVertexConstraint`.
    pub(crate) fn set_or_update_vertex_constraint(&mut self, vid: i32, vc: VertexConstraint) {
        self.vertices.insert(vid, vc);
    }

    /// C# `ClearVertexConstraint`.
    pub(crate) fn clear_vertex_constraint(&mut self, vid: i32) {
        self.vertices.remove(&vid);
    }

    /// C# `HasConstraints`.
    pub(crate) fn has_constraints(&self) -> bool {
        !self.edges.is_empty() || !self.vertices.is_empty()
    }
}

/// C# `MeshConstraintUtil.FixAllBoundaryEdges(cons, mesh)`: every boundary edge gets no
/// flip/split/collapse, and its vertices are pinned in place.
pub(crate) fn fix_all_boundary_edges(cons: &mut MeshConstraints, mesh: &DMesh3) {
    let ne = mesh.max_edge_id();
    for ei in 0..ne {
        if mesh.is_edge(ei) && mesh.is_boundary_edge(ei) {
            cons.set_or_update_edge_constraint(ei, EdgeConstraint::fully_constrained());

            let ev = mesh.get_edge_v(ei);
            cons.set_or_update_vertex_constraint(ev[0], VertexConstraint::pinned());
            cons.set_or_update_vertex_constraint(ev[1], VertexConstraint::pinned());
        }
    }
}
