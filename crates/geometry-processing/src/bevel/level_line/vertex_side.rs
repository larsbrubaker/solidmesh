//! Which side of the level line a mesh vertex stands on ([`VertexSide`]) and the per-vertex reading
//! cache the construction shares ([`VertexFieldReading`]), with the reading's constructor, its
//! field, the side a value
//! reads ([`VertexFieldReading::side_for`]), a point's value ([`VertexFieldReading::value_at_point`])
//! the readings across the known cuts ([`VertexFieldReading::known_cut_value_at`],
//! [`VertexFieldReading::value_in_the_end_plane`]) and the known cuts' test of a chain's or pick's bounded ends
//! ([`VertexFieldReading::has_known_cuts`], [`VertexFieldReading::edge_lies_in_a_known_cut`]),
//! whether an edge or triangle leaves the band across one chain's known cut rather than the line
//! ([`VertexFieldReading::leaves_across_a_known_cut`], read per chain and per plane) and
//! the stopped ends' tests ([`VertexFieldReading::has_stopped_ends`],
//! [`VertexFieldReading::in_a_stopped_end_plane`], [`VertexFieldReading::stopped_end_plane_at`],
//! [`VertexFieldReading::is_a_band_corner_on_a_stopped_plane`]); the
//! reading's queries land with the refinement's method package.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use vector_math::{ieee_math, Vector3};

use super::corner_cap_flat_ends::use_flat_ends;
use super::{FieldKind, LevelLineMesh, ScalarField};

/// Which side of the level line a mesh vertex stands on, decided once and read
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

/// The per-vertex reading of the field, and the cache the rest of the
/// construction shares so that a vertex is never evaluated twice and never answers two different
/// numbers. Every holder fills the cache through a shared reference, so the caches are
/// interior-mutable.
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
    /// coordinates' bits, -0.0 read as 0.0 since the two are equal), with the work units
    /// finding each one charged.
    pub remembered_roots: RefCell<HashMap<[u64; 6], (Vector3, i64)>>,
    /// The half-width of the band, as a LENGTH, inside which a vertex counts as on the line. Never
    /// negative (the constructor refuses it).
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

/// The refusal from the `VertexFieldReading` constructor: a snap band that
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
    /// A reading with nothing read yet. A snap that is negative or NaN is refused.
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

    /// The field being read.
    pub fn field(&self) -> &Rc<dyn ScalarField> {
        &self.field
    }

    /// True when flat ends are on and the field is a chain's
    /// or a pick's, whose bounded ends are known cuts - or such a pick read with its run-outs, whose
    /// ends are still the pick's.
    pub fn has_known_cuts(&self) -> bool {
        use_flat_ends()
            && matches!(
                self.pick().kind(),
                FieldKind::ChainField | FieldKind::PickField
            )
    }

    /// Whether corners `insides` (inside the band) and `outsides` (outside it) leave the band
    /// across one known-cut plane of ONE of the pick's chains, inside by that chain's own value
    /// ([`super::ChainField::leaves_across_its_known_cut`]): the edge then has no crossing, and the
    /// triangle stands outside. False where known cuts are off.
    pub fn leaves_across_a_known_cut(
        &self,
        insides: &[Vector3],
        outsides: &[Vector3],
    ) -> Result<bool, super::WorkError> {
        if !use_flat_ends() {
            return Ok(false);
        }

        let pick = self.pick();
        if let Some(chain) = pick.as_chain_field() {
            return chain.leaves_across_its_known_cut(insides, outsides);
        }

        if let Some(pick) = pick.as_pick_field() {
            for &k in &pick.members {
                if pick.chains[k as usize].leaves_across_its_known_cut(insides, outsides)? {
                    return Ok(true);
                }
            }
        }

        Ok(false)
    }

    /// The pick under a run-out wrapper - the known cuts are its
    /// chains' end planes; a run-out has none. Any other field is itself.
    pub fn pick(&self) -> Rc<dyn ScalarField> {
        match self.field.as_with_run_outs() {
            Some(wrapped) => wrapped.inner().clone(),
            None => self.field.clone(),
        }
    }

    /// Whether the edge a-b lies in a bounded end's
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

    /// Whether a point stands in a stopped end's plane of the pick (under any run-out wrapper):
    /// the line runs through a vertex there that the clipped field reads on it, while the
    /// triangles on the band side of it reach into the band. False for any other field. Reads only
    /// the clips, never the field, so it charges no work.
    pub fn in_a_stopped_end_plane(&self, point: Vector3) -> bool {
        self.stopped_end_plane_at(point).is_some()
    }

    /// The stopped end's plane of the pick (under any run-out wrapper) a point stands in, as
    /// (point on it, unit normal); `None` for any other field. Reads only the clips.
    pub fn stopped_end_plane_at(&self, point: Vector3) -> Option<(Vector3, Vector3, f64)> {
        let pick = self.pick();
        if let Some(chain) = pick.as_chain_field() {
            chain.stopped_end_plane_at(point)
        } else if let Some(pick) = pick.as_pick_field() {
            pick.stopped_end_plane_at(point)
        } else {
            None
        }
    }

    /// Whether the pick (under any run-out wrapper) has a stopped end at all; reads only the
    /// chains' bounds, so it charges no work.
    pub fn has_stopped_ends(&self) -> bool {
        let pick = self.pick();
        if let Some(chain) = pick.as_chain_field() {
            chain.is_clipped_by_a_stop()
        } else if let Some(pick) = pick.as_pick_field() {
            pick.has_a_stopped_member()
        } else {
            false
        }
    }

    /// Whether a vertex the reading puts ON the line stands in a stopped end's plane
    /// ([`VertexFieldReading::in_a_stopped_end_plane`]): a corner of the band, whose edges into the
    /// band's side can dip below the line between ends that read on or outside it.
    pub fn is_a_band_corner_on_a_stopped_plane(
        &self,
        vertex: i32,
    ) -> Result<bool, super::WorkError> {
        if self.side_of(vertex)? != VertexSide::On {
            return Ok(false);
        }

        let position = self.mesh.borrow().position(vertex);
        Ok(self.in_a_stopped_end_plane(position))
    }

    /// The field value at an arbitrary point - the same field
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

    /// The field along an edge a-b lying in a bounded
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
            Ok(ieee_math::min_f64(
                chain.value_in_the_end_plane(point)?,
                run_out,
            ))
        } else if let Some(pick) = pick.as_pick_field() {
            Ok(ieee_math::min_f64(
                pick.value_in_the_end_plane(point, a, b)?,
                run_out,
            ))
        } else {
            self.field.value_at(point)
        }
    }

    /// The field read with every bounded end's plane as a
    /// KNOWN CUT (see [`super::ChainField::value_across_the_known_cut`]) when flat ends are on; the
    /// plain field otherwise. Only side decisions ask it - never the refinement. A run-out is read
    /// as it is on either side of a known cut: the least of it and the pick's value across the cut.
    pub fn known_cut_value_at(&self, point: Vector3) -> Result<f64, super::WorkError> {
        if use_flat_ends() {
            let run_out = self.run_out_value_at(point)?;
            let pick = self.pick();
            if let Some(chain) = pick.as_chain_field() {
                return Ok(ieee_math::min_f64(
                    chain.value_across_the_known_cut(point)?,
                    run_out,
                ));
            }

            if let Some(pick) = pick.as_pick_field() {
                return Ok(ieee_math::min_f64(
                    pick.value_across_the_known_cut(point)?,
                    run_out,
                ));
            }
        }

        self.field.value_at(point)
    }

    /// The field value at a vertex, evaluated once and
    /// remembered.
    pub fn value_at(&self, vertex: i32) -> Result<f64, super::WorkError> {
        self.grow(vertex)?;
        Ok(self.values.borrow()[&vertex])
    }

    /// The side a vertex stands on - the one answer every
    /// triangle round it reads.
    pub fn side_of(&self, vertex: i32) -> Result<VertexSide, super::WorkError> {
        self.grow(vertex)?;
        Ok(self.sides.borrow()[&vertex])
    }

    /// Promotes a vertex to [`VertexSide::On`] - what an
    /// edge root lying within the snap of an end does instead of cutting a sliver. The value is
    /// left as it was; only the side moves.
    pub fn put_on_the_line(&self, vertex: i32) -> Result<(), super::WorkError> {
        self.grow(vertex)?;
        self.sides.borrow_mut().insert(vertex, VertexSide::On);
        Ok(())
    }

    /// Reads a vertex once, leaving every vertex nobody asked about unread. A vertex not in the
    /// mesh is a caller's bug, and panics.
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
        // not its level line: on the band side the chain's own value places it. Not in a STOPPED
        // end's plane: that plane crosses faces the mesh carries on with past the end, so the
        // line runs through such a vertex, and the triangles on the band side of it are found by
        // the refinement's dip from it (`band_refinement_asking`).
        if side == VertexSide::On
            && self.has_known_cuts()
            && !self.in_a_stopped_end_plane(position)
            && self.known_cut_value_at(position)? < -self.snap
        {
            side = VertexSide::Inside;
        }

        self.sides.borrow_mut().insert(vertex, side);
        Ok(())
    }

    /// The side that a raw field value and a snap band say, with
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
