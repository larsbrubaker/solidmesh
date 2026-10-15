//! Remesh constraints (derived from geometry3Sharp): the edge refine flags, the per-edge and
//! per-vertex constraints, the constraint set, the projection target the constraints and the
//! `Remesher` project onto, and [`fix_all_boundary_edges`], which `MinimalHoleFill` uses. The
//! `Remesher` (`remesher*.rs`) reads these per edge and vertex.
//!
//! The constraint maps are only looked up, set and cleared, never iterated, so a `HashMap`
//! cannot leak an order into a result.

use std::collections::HashMap;
use std::rc::Rc;

use super::dmesh3::DMesh3;

/// Something a point can be projected onto.
pub(crate) trait ProjectionTarget {
    /// The projection of `point` onto the target; `identifier` names the caller's element.
    fn project(&self, point: [f64; 3], identifier: i32) -> [f64; 3];
}

/// A shared projection target. Targets are compared by identity ([`same_target`]): two
/// constraints agree only when they project onto the very same object.
pub(crate) type TargetRef = Option<Rc<dyn ProjectionTarget>>;

/// Whether two targets are the same: both absent, or the same object.
pub(crate) fn same_target(a: &TargetRef, b: &TargetRef) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => std::ptr::eq(Rc::as_ptr(a) as *const u8, Rc::as_ptr(b) as *const u8),
        _ => false,
    }
}

/// Edge refine flags, combined bitwise: which edits the remesher may not make to an edge.
pub(crate) mod edge_refine_flags {
    pub(crate) const NO_CONSTRAINT: u32 = 0;
    pub(crate) const NO_FLIP: u32 = 1;
    pub(crate) const NO_SPLIT: u32 = 2;
    pub(crate) const NO_COLLAPSE: u32 = 4;
    pub(crate) const FULLY_CONSTRAINED: u32 = NO_FLIP | NO_SPLIT | NO_COLLAPSE;
    #[allow(dead_code)] // mesh API only the tests call so far
    /// Avoid 'losing' this edge when a neighbour collapses (the edge itself may collapse).
    pub(crate) const PRESERVE_TOPOLOGY: u32 = 8;
}
use edge_refine_flags as erf;

/// What the remesher may do to one edge.
#[derive(Clone)]
pub(crate) struct EdgeConstraint {
    refine_flags: u32,
    /// The edge is associated with this target. Only information: edges are not projected
    /// (a `VertexConstraint` must also be set).
    pub(crate) target: TargetRef,
    #[allow(dead_code)] // mesh API only the tests call so far
    /// Not a constraint: lets descendants of a constrained input edge be found.
    pub(crate) tracking_set_id: i32,
}

impl EdgeConstraint {
    /// A constraint with these refine flags and no target.
    pub(crate) fn new(refine_flags: u32) -> Self {
        Self {
            refine_flags,
            target: None,
            tracking_set_id: -1,
        }
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// A constraint with these refine flags, associated with `target`.
    pub(crate) fn with_target(refine_flags: u32, target: TargetRef) -> Self {
        Self {
            refine_flags,
            target,
            tracking_set_id: -1,
        }
    }

    /// No restriction.
    pub(crate) fn unconstrained() -> Self {
        Self::new(erf::NO_CONSTRAINT)
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// The edge may be split or collapsed but not flipped.
    pub(crate) fn no_flips() -> Self {
        Self::new(erf::NO_FLIP)
    }

    /// The edge may not be flipped, split or collapsed.
    pub(crate) fn fully_constrained() -> Self {
        Self::new(erf::FULLY_CONSTRAINED)
    }

    /// Whether the edge may be flipped.
    pub(crate) fn can_flip(&self) -> bool {
        (self.refine_flags & erf::NO_FLIP) == 0
    }

    /// Whether the edge may be split.
    pub(crate) fn can_split(&self) -> bool {
        (self.refine_flags & erf::NO_SPLIT) == 0
    }

    /// Whether the edge may be collapsed.
    pub(crate) fn can_collapse(&self) -> bool {
        (self.refine_flags & erf::NO_COLLAPSE) == 0
    }

    /// Whether every edit is forbidden.
    pub(crate) fn no_modifications(&self) -> bool {
        (self.refine_flags & erf::FULLY_CONSTRAINED) == erf::FULLY_CONSTRAINED
    }

    /// Whether nothing restricts the edge: no flags and no target.
    pub(crate) fn is_unconstrained(&self) -> bool {
        self.refine_flags == erf::NO_CONSTRAINT && self.target.is_none()
    }
}

/// What the remesher may do to one vertex: keep it fixed, or keep it on a target.
#[derive(Clone)]
pub(crate) struct VertexConstraint {
    pub(crate) fixed: bool,
    /// The `Remesher` may collapse two fixed vertices with the same set id together.
    pub(crate) fixed_set_id: i32,
    /// The vertex is constrained to lie on this target. Fixed and Target are exclusive.
    pub(crate) target: TargetRef,
}

impl VertexConstraint {
    /// The set id of a vertex in no set; clients treat any negative id as invalid.
    pub(crate) const INVALID_SET_ID: i32 = -1;

    /// A fixed (or free) vertex in fixed set `set_id`.
    pub(crate) fn new_fixed(is_fixed: bool, set_id: i32) -> Self {
        Self {
            fixed: is_fixed,
            fixed_set_id: set_id,
            target: None,
        }
    }

    /// A vertex that must stay on `target`.
    pub(crate) fn new_target(target: TargetRef) -> Self {
        Self {
            fixed: false,
            fixed_set_id: Self::INVALID_SET_ID,
            target,
        }
    }

    /// A free vertex.
    pub(crate) fn unconstrained() -> Self {
        Self::new_fixed(false, Self::INVALID_SET_ID)
    }

    /// A vertex that may not move.
    pub(crate) fn pinned() -> Self {
        Self::new_fixed(true, Self::INVALID_SET_ID)
    }
}

/// The edge and vertex constraints of one mesh, keyed by id.
#[derive(Clone, Default)]
pub(crate) struct MeshConstraints {
    edges: HashMap<i32, EdgeConstraint>,
    vertices: HashMap<i32, VertexConstraint>,
    #[allow(dead_code)] // mesh API only the tests call so far
    set_id_counter: i32,
}

impl MeshConstraints {
    #[allow(dead_code)] // mesh API only the tests call so far
    /// A new fixed-set id, unique within this constraint set.
    pub(crate) fn allocate_set_id(&mut self) -> i32 {
        let id = self.set_id_counter;
        self.set_id_counter += 1;
        id
    }

    /// Whether edge `eid` has a constraint.
    pub(crate) fn has_edge_constraint(&self, eid: i32) -> bool {
        self.edges.contains_key(&eid)
    }

    /// Edge `eid`'s constraint, or unconstrained when none is set.
    pub(crate) fn get_edge_constraint(&self, eid: i32) -> EdgeConstraint {
        self.edges
            .get(&eid)
            .cloned()
            .unwrap_or_else(EdgeConstraint::unconstrained)
    }

    /// Sets (or replaces) edge `eid`'s constraint.
    pub(crate) fn set_or_update_edge_constraint(&mut self, eid: i32, ec: EdgeConstraint) {
        self.edges.insert(eid, ec);
    }

    /// Removes edge `eid`'s constraint.
    pub(crate) fn clear_edge_constraint(&mut self, eid: i32) {
        self.edges.remove(&eid);
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Whether vertex `vid` has a constraint.
    pub(crate) fn has_vertex_constraint(&self, vid: i32) -> bool {
        self.vertices.contains_key(&vid)
    }

    /// Vertex `vid`'s constraint, or unconstrained when none is set.
    pub(crate) fn get_vertex_constraint(&self, vid: i32) -> VertexConstraint {
        self.vertices
            .get(&vid)
            .cloned()
            .unwrap_or_else(VertexConstraint::unconstrained)
    }

    /// Vertex `vid`'s constraint, or `None` when none is set. Callers that need a value then
    /// use the default: not fixed, set id 0, no target.
    pub(crate) fn try_get_vertex_constraint(&self, vid: i32) -> Option<VertexConstraint> {
        self.vertices.get(&vid).cloned()
    }

    /// Sets (or replaces) vertex `vid`'s constraint.
    pub(crate) fn set_or_update_vertex_constraint(&mut self, vid: i32, vc: VertexConstraint) {
        self.vertices.insert(vid, vc);
    }

    /// Removes vertex `vid`'s constraint.
    pub(crate) fn clear_vertex_constraint(&mut self, vid: i32) {
        self.vertices.remove(&vid);
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Whether any edge or vertex is constrained.
    pub(crate) fn has_constraints(&self) -> bool {
        !self.edges.is_empty() || !self.vertices.is_empty()
    }
}

/// Every boundary edge gets no
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
