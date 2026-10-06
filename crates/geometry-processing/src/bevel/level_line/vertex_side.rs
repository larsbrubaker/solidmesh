//! Which side of the level line a mesh vertex stands on ([`VertexSide`]) and the per-vertex reading
//! cache the construction shares ([`VertexFieldReading`]). Ports the types of
//! `Bevel/LevelLine/VertexSide.cs` with the reading's constructor, its field, the side a value
//! reads ([`VertexFieldReading::side_for`]), a point's value ([`VertexFieldReading::value_at_point`])
//! the readings across the known cuts ([`VertexFieldReading::known_cut_value_at`],
//! [`VertexFieldReading::value_in_the_end_plane`]) and the known cuts' test of a chain's or pick's bounded ends
//! ([`VertexFieldReading::has_known_cuts`], [`VertexFieldReading::edge_lies_in_a_known_cut`]); the
//! reading's queries land with the refinement's method package.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::corner_cap_flat_ends::use_flat_ends;
use super::{FieldKind, LevelLineMesh, ScalarField};

/// C# `VertexSide`: which side of the level line a mesh vertex stands on, decided once and read
/// by every triangle round it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum VertexSide {
    /// Field value below the snap band: in the material the cut deletes.
    #[default]
    Inside = 0,

    /// Within the snap band of the line: the line passes through this existing vertex.
    On,

    /// Field value above the snap band: in the material the cut keeps.
    Outside,
}

/// C# `VertexFieldReading`: the per-vertex reading of the field, and the cache the rest of the
/// construction shares so that a vertex is never evaluated twice and never answers two different
/// numbers. The C# fills the cache through every holder's reference, so the caches are
/// interior-mutable here.
pub struct VertexFieldReading {
    /// The working mesh whose vertices are read.
    pub mesh: Rc<RefCell<LevelLineMesh>>,
    /// The field being read - the refinement asks a chain distance field for more than a value at
    /// a point.
    pub field: Rc<dyn ScalarField>,
    /// The field value per vertex read so far. Read ONE AT A TIME, not filled up to the vertex
    /// asked for: the band a cut touches is a few hundred triangles of a part that may have tens of
    /// thousands, and the dense fill this replaces charged every build a field query at every
    /// vertex below the highest one anybody asked about - 20,000 of the cylinder's 63,000 queries,
    /// none of which any answer depended on.
    pub values: RefCell<HashMap<i32, f64>>,
    /// The side per vertex read so far.
    pub sides: RefCell<HashMap<i32, VertexSide>>,
    /// Edge roots already found on this reading's field, by the exact ordered end points (the six
    /// coordinates' bits, -0.0 read as 0.0, as C# double equality has them), with the work units
    /// finding each one charged.
    pub remembered_roots: RefCell<HashMap<[u64; 6], (Vector3, i64)>>,
    /// The half-width of the band, as a LENGTH, inside which a vertex counts as on the line. Never
    /// negative (C#'s constructor refuses it).
    pub snap: f64,
}

impl std::fmt::Debug for VertexFieldReading {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The field has no Debug; give the snap and how much has been read.
        f.debug_struct("VertexFieldReading")
            .field("snap", &self.snap)
            .field("values_read", &self.values.borrow().len())
            .finish_non_exhaustive()
    }
}

/// C#'s `ArgumentOutOfRangeException` from the `VertexFieldReading` constructor: a snap band that
/// is not a non-negative length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NegativeSnap(pub f64);

impl std::fmt::Display for NegativeSnap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "the snap band is a length, so it cannot be negative (snap {})",
            self.0
        )
    }
}

impl std::error::Error for NegativeSnap {}

impl VertexFieldReading {
    /// C#'s constructor (with the field initializers: nothing read yet). A snap that is negative
    /// or NaN is refused, as C# throws.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn new(
        mesh: Rc<RefCell<LevelLineMesh>>,
        field: Rc<dyn ScalarField>,
        snap: f64,
    ) -> Result<Self, NegativeSnap> {
        if !(snap >= 0.0) {
            return Err(NegativeSnap(snap));
        }

        Ok(VertexFieldReading {
            mesh,
            field,
            values: RefCell::new(HashMap::new()),
            sides: RefCell::new(HashMap::new()),
            remembered_roots: RefCell::new(HashMap::new()),
            snap,
        })
    }

    /// C# `VertexFieldReading.Field`: the field being read.
    pub fn field(&self) -> &Rc<dyn ScalarField> {
        &self.field
    }

    /// C# `VertexFieldReading.HasKnownCuts`: true when flat ends are on and the field is a chain's
    /// or a pick's, whose bounded ends are known cuts - or such a pick read with its run-outs, whose
    /// ends are still the pick's.
    pub fn has_known_cuts(&self) -> bool {
        use_flat_ends()
            && matches!(
                self.pick().kind(),
                FieldKind::ChainField | FieldKind::PickField
            )
    }

    /// C# `VertexFieldReading.Pick`: the pick under a run-out wrapper - the known cuts are its
    /// chains' end planes; a run-out has none. Any other field is itself.
    pub fn pick(&self) -> Rc<dyn ScalarField> {
        match self.field.as_with_run_outs() {
            Some(wrapped) => wrapped.inner().clone(),
            None => self.field.clone(),
        }
    }

    /// C# `VertexFieldReading.EdgeLiesInAKnownCut`: whether the edge a-b lies in a bounded end's
    /// plane (a known cut); false without flat ends.
    pub fn edge_lies_in_a_known_cut(&self, a: Vector3, b: Vector3) -> bool {
        if !self.has_known_cuts() {
            return false;
        }

        let pick = self.pick();
        if let Some(chain) = pick.as_chain_field() {
            chain.lies_in_an_end_plane(a, b)
        } else if let Some(pick) = pick.as_pick_field() {
            pick.lies_in_an_end_plane(a, b)
        } else {
            false
        }
    }

    /// C# `VertexFieldReading.ValueAtPoint`: the field value at an arbitrary point - the same field
    /// the vertices are read from.
    pub fn value_at_point(&self, point: Vector3) -> Result<f64, super::WorkError> {
        self.field.value_at(point)
    }

    /// The run-outs' least at `point` where the field is a group read with its run-outs; +infinity
    /// otherwise.
    fn run_out_value_at(&self, point: Vector3) -> Result<f64, super::WorkError> {
        match self.field.as_with_run_outs() {
            Some(wrapped) => wrapped.run_out_value_at(point),
            None => Ok(f64::INFINITY),
        }
    }

    /// C# `VertexFieldReading.ValueInTheEndPlane`: the field along an edge a-b lying in a bounded
    /// end's plane ([`VertexFieldReading::edge_lies_in_a_known_cut`]): the chain's own value,
    /// whose zero is the band's boundary there - the clipped field reads 0 all along the plane
    /// inside the band. A run-out is read as [`VertexFieldReading::known_cut_value_at`] reads it.
    pub fn value_in_the_end_plane(
        &self,
        point: Vector3,
        a: Vector3,
        b: Vector3,
    ) -> Result<f64, super::WorkError> {
        let run_out = self.run_out_value_at(point)?;
        let pick = self.pick();
        if let Some(chain) = pick.as_chain_field() {
            Ok(cs_math::min_f64(
                chain.value_in_the_end_plane(point)?,
                run_out,
            ))
        } else if let Some(pick) = pick.as_pick_field() {
            Ok(cs_math::min_f64(
                pick.value_in_the_end_plane(point, a, b)?,
                run_out,
            ))
        } else {
            self.field.value_at(point)
        }
    }

    /// C# `VertexFieldReading.KnownCutValueAt`: the field read with every bounded end's plane as a
    /// KNOWN CUT (see [`super::ChainField::value_across_the_known_cut`]) when flat ends are on; the
    /// plain field otherwise. Only side decisions ask it - never the refinement. A run-out is read
    /// as it is on either side of a known cut: the least of it and the pick's value across the cut.
    pub fn known_cut_value_at(&self, point: Vector3) -> Result<f64, super::WorkError> {
        if use_flat_ends() {
            let run_out = self.run_out_value_at(point)?;
            let pick = self.pick();
            if let Some(chain) = pick.as_chain_field() {
                return Ok(cs_math::min_f64(
                    chain.value_across_the_known_cut(point)?,
                    run_out,
                ));
            }

            if let Some(pick) = pick.as_pick_field() {
                return Ok(cs_math::min_f64(
                    pick.value_across_the_known_cut(point)?,
                    run_out,
                ));
            }
        }

        self.field.value_at(point)
    }

    /// C# `VertexFieldReading.ValueAt`: the field value at a vertex, evaluated once and
    /// remembered.
    pub fn value_at(&self, vertex: i32) -> Result<f64, super::WorkError> {
        self.grow(vertex)?;
        Ok(self.values.borrow()[&vertex])
    }

    /// C# `VertexFieldReading.SideOf`: the side a vertex stands on - the one answer every
    /// triangle round it reads.
    pub fn side_of(&self, vertex: i32) -> Result<VertexSide, super::WorkError> {
        self.grow(vertex)?;
        Ok(self.sides.borrow()[&vertex])
    }

    /// C# `VertexFieldReading.PutOnTheLine`: promotes a vertex to [`VertexSide::On`] - what an
    /// edge root lying within the snap of an end does instead of cutting a sliver. The value is
    /// left as it was; only the side moves.
    pub fn put_on_the_line(&self, vertex: i32) -> Result<(), super::WorkError> {
        self.grow(vertex)?;
        self.sides.borrow_mut().insert(vertex, VertexSide::On);
        Ok(())
    }

    /// C# `VertexFieldReading.Grow`: reads a vertex once, leaving every vertex nobody asked
    /// about unread. A vertex not in the mesh is a caller's bug (C# throws
    /// `ArgumentOutOfRangeException`).
    fn grow(&self, vertex: i32) -> Result<(), super::WorkError> {
        if self.values.borrow().contains_key(&vertex) {
            return Ok(());
        }

        // The mesh is read and released before the field is asked: a field never reads it, but a
        // caller may hold it mutably between reads.
        let (position, float_input_vertex) = {
            let mesh = self.mesh.borrow();
            assert!(
                vertex >= 0 && vertex < mesh.vertex_count(),
                "no such vertex in the mesh"
            );
            (
                mesh.position(vertex),
                mesh.input_is_float_stored && mesh.input_edge_behind(vertex) == (vertex, vertex),
            )
        };
        let value = self.field.value_at(position)?;
        self.values.borrow_mut().insert(vertex, value);
        let mut side = Self::side_for(value, self.snap);

        // A float-stored INPUT vertex is where the store put it, to a float's step: a vertex the
        // shape puts on the line reads a float's width off zero, above the snap, and is on the
        // line all the same. Only an input vertex: one the build made is exact.
        if side != VertexSide::On
            && float_input_vertex
            && value.abs() <= crate::bevel::float_grid::step_at_point(position)
        {
            side = VertexSide::On;
        }

        // A vertex ON a bounded end's plane (a known cut, flat ends on) reads the clipped field's 0
        // wherever the chain is within its setback, but the plane is the band's boundary there,
        // not its level line: on the band side the chain's own value places it.
        if side == VertexSide::On
            && self.has_known_cuts()
            && self.known_cut_value_at(position)? < -self.snap
        {
            side = VertexSide::Inside;
        }

        self.sides.borrow_mut().insert(vertex, side);
        Ok(())
    }

    /// C# `VertexFieldReading.SideFor`: the side that a raw field value and a snap band say, with
    /// no mesh involved.
    pub fn side_for(value: f64, snap: f64) -> VertexSide {
        if value.abs() <= snap {
            return VertexSide::On;
        }

        if value < 0.0 {
            VertexSide::Inside
        } else {
            VertexSide::Outside
        }
    }
}
