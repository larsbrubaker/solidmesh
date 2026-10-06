//! The convex piece of the mesh a least-value query is asked about ([`ChainFieldMeshPiece`]), from
//! `Bevel/LevelLine/ChainField.Queries.cs`, with its constructors and closest point, the chain's
//! parameter wrap ([`ChainField::wrapped`]), the spoke test across a turn
//! ([`ChainField::spoke_gives_way_across_a_turn`]), the bounded maximum queries
//! ([`ChainField::max_along_segment`], [`ChainField::max_over_triangle`],
//! [`ChainField::island_peak`], with the segment cell bound) and the field seam ([`ScalarField`] for
//! `ChainField`, saying it is a chain's field). The other queries - the `ChainField` methods of
//! that partial - land with their method package.

use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::bounded_maximum;
use super::{
    ChainField, FieldCounters, FieldKind, FieldMaximum, FieldSample, LevelLineWork, ScalarField,
    WorkError,
};

/// An opaque closest-point function on a piece (C# `Func<Vector3, Vector3>`).
pub type ClosestOnPiece = Rc<dyn Fn(Vector3) -> Vector3>;

/// C# `ChainField.MeshPiece`: the convex piece of the mesh a least-value query is asked about - a
/// segment, a triangle, or - for the exhaustive reference alone - an opaque closest-point
/// function, which has no closed form to reach for.
#[derive(Clone, Default)]
pub struct ChainFieldMeshPiece {
    /// The piece's first corner.
    pub a: Vector3,
    /// The piece's second corner.
    pub b: Vector3,
    /// The piece's third corner; unused on a segment.
    pub c: Vector3,
    /// True for a triangle, false for a segment or a custom piece.
    pub is_triangle: bool,
    /// The custom piece's closest-point function; `None` when the piece's own geometry is known,
    /// so a constant setback can be answered outright.
    pub closest_on_piece: Option<ClosestOnPiece>,
}

impl std::fmt::Debug for ChainFieldMeshPiece {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The closest-point function has no Debug; say only whether there is one.
        f.debug_struct("ChainFieldMeshPiece")
            .field("a", &self.a)
            .field("b", &self.b)
            .field("c", &self.c)
            .field("is_triangle", &self.is_triangle)
            .field("custom", &self.closest_on_piece.is_some())
            .finish()
    }
}

impl ChainFieldMeshPiece {
    /// C#'s private constructor.
    pub fn new(
        a: Vector3,
        b: Vector3,
        c: Vector3,
        is_triangle: bool,
        closest_on_piece: Option<ClosestOnPiece>,
    ) -> Self {
        ChainFieldMeshPiece {
            a,
            b,
            c,
            is_triangle,
            closest_on_piece,
        }
    }

    /// C# `MeshPiece.HasClosedForm`: true when the piece's own geometry is known, so a constant
    /// setback can be answered outright.
    pub fn has_closed_form(&self) -> bool {
        self.closest_on_piece.is_none()
    }

    /// C# `MeshPiece.Segment`: the segment a-b.
    pub fn segment(a: Vector3, b: Vector3) -> Self {
        Self::new(a, b, Vector3::default(), false, None)
    }

    /// C# `MeshPiece.Triangle`: the triangle a, b, c.
    pub fn triangle(a: Vector3, b: Vector3, c: Vector3) -> Self {
        Self::new(a, b, c, true, None)
    }

    /// C# `MeshPiece.Custom`: a piece known only by its closest-point function.
    pub fn custom(closest_on_piece: ClosestOnPiece) -> Self {
        Self::new(
            Vector3::default(),
            Vector3::default(),
            Vector3::default(),
            false,
            Some(closest_on_piece),
        )
    }

    /// The piece's closest point to `point`: the custom function where there is one, otherwise
    /// the segment's or the triangle's own closed form.
    pub fn closest_to(&self, point: Vector3) -> Vector3 {
        if let Some(closest_on_piece) = &self.closest_on_piece {
            return closest_on_piece(point);
        }

        if self.is_triangle {
            ChainField::closest_on_triangle(point, self.a, self.b, self.c)
        } else {
            ChainField::closest_on_segment(point, self.a, self.b)
        }
    }
}

impl ChainField {
    /// C# `ChainField.Wrapped`: a parameter read onto the chain: round the loop of a closed chain,
    /// held to the ends of an open one.
    pub fn wrapped(&self, parameter: f64) -> f64 {
        if !self.is_closed {
            return parameter.clamp(0.0, self.length);
        }

        // C#'s `%` on doubles is the truncating remainder, as Rust's.
        let parameter = parameter % self.length;
        if parameter < 0.0 {
            parameter + self.length
        } else {
            parameter
        }
    }

    /// True when no point of the box `min`..`max` can carry a field value at or below `margin`:
    /// the box stands farther from the chain's own box than the greatest setback plus that margin.
    /// The cull a caller that only asks "does this piece dip below the line" uses to skip the
    /// search - it answers from two boxes, so it can never turn a dipping piece away.
    pub fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        let root = &self.nodes[0];
        super::segment_box_tree::box_gap(min, max, root.min, root.max) - self.max_setback > margin
    }
}

impl ChainField {
    /// C# `ChainField.SpokeGivesWayAcrossATurn`: whether the usual spoke between two rails' next
    /// readings (`advances_left` says which rail moves) gives way to the other because it would
    /// cut ACROSS A TURN of the chain: its chord falls short of the path along the chain by a tenth
    /// of the setback - the scale at which a row drawn about the mean chain point leaves the
    /// fillet's round - while the other spoke spans less of the chain.
    pub fn spoke_gives_way_across_a_turn(
        &self,
        left: f64,
        next_left: f64,
        right: f64,
        next_right: f64,
        advances_left: bool,
    ) -> bool {
        let (usual_from, usual_to) = if advances_left {
            (next_left, right)
        } else {
            (left, next_right)
        };
        let (other_from, other_to) = if advances_left {
            (left, next_right)
        } else {
            (next_left, right)
        };
        let along = (usual_to - usual_from).abs();
        let chord = (self.point_at(self.wrapped(usual_to))
            - self.point_at(self.wrapped(usual_from)))
        .length();
        along - chord > 0.1 * self.min_setback && (other_to - other_from).abs() < along
    }
}

impl ChainField {
    /// C# `ChainField.MaxAlongSegment(a, b, tolerance)`: the greatest field value along a segment,
    /// by bounded search (see `bounded_maximum`), from no floor.
    pub fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_along_segment_at_least(a, b, tolerance, f64::NEG_INFINITY)
    }

    /// C# `ChainField.MaxAlongSegment(a, b, tolerance, atLeast)`: the same search given a floor
    /// `at_least` to prune against from its first step - a value fixed by the caller BEFORE the
    /// search and never raised by it. Nothing beating it, the answer is `at_least` itself at a
    /// MEANINGLESS point. A caller whose test on the answer is a threshold T should pass
    /// `T - tolerance`, not T: a flush floor gives away the whole band a true maximum may sit in.
    pub fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        FieldCounters::with(|t| t.max_along += 1);
        LevelLineWork::charge(1)?;
        let reader = self.end_plane_reader();
        bounded_maximum::over_segment(
            &|p| self.evaluate(p),
            a,
            b,
            tolerance,
            at_least,
            reader.as_deref(),
        )
    }

    /// C# `ChainField.MaxOverTriangle(a, b, c, tolerance)`: the greatest field value over a
    /// triangle, interior included, by bounded search.
    pub fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_over_triangle_bounded(a, b, c, tolerance, f64::INFINITY, f64::NEG_INFINITY)
    }

    /// C# `ChainField.MaxOverTriangle(a, b, c, tolerance, stopAbove, atLeast)`: the greatest field
    /// value over a triangle, giving up as soon as it is known to stand above `stop_above` and
    /// pruning against a value `at_least` already attained elsewhere (`at_least` itself, at a
    /// meaningless point, when the triangle does not beat it).
    pub fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        FieldCounters::with(|t| t.max_tri += 1);
        LevelLineWork::charge(1)?;
        let reader = self.end_plane_reader();
        bounded_maximum::over_triangle(
            &|p| self.evaluate(p),
            a,
            b,
            c,
            tolerance,
            stop_above,
            at_least,
            None,
            reader.as_deref(),
        )
    }

    /// C# `ChainField.IslandPeak`: the island rule's search over the triangle abc, sharpened by
    /// the chain's segment cell bound (see `bounded_maximum::island_peak`).
    pub fn island_peak(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        snap: f64,
    ) -> Result<FieldMaximum, WorkError> {
        FieldCounters::with(|t| t.max_tri += 1);
        LevelLineWork::charge(1)?;
        let reader = self.end_plane_reader();
        bounded_maximum::island_peak(
            &|p| self.evaluate(p),
            a,
            b,
            c,
            tolerance,
            snap,
            Some(&|s0, s1, s2, v0, v1, v2| self.segment_cell_bound(0, s0, s1, s2, v0, v1, v2)),
            reader.as_deref(),
        )
    }

    /// C# `ChainField.SegmentCellBound`: a sound upper bound on this field over a cell, from the
    /// chain SEGMENTS its corners' samples of chain `chain` name - the island search's sharper
    /// bound. Over one segment the reading `min over u of (|q - c(u)| - s(u))` is CONVEX in q, the
    /// field is the least over every segment, and a convex function's greatest value over a cell is
    /// at a corner, so `min over j of (max over corners of g_j)` bounds the field over the cell for
    /// any choice of segments; the corners' own segments make it tight. An end clip RAISES the
    /// field, so a clipped chain gives no bound (+infinity).
    #[allow(clippy::too_many_arguments)]
    pub fn segment_cell_bound(
        &self,
        chain: i32,
        s0: &FieldSample,
        s1: &FieldSample,
        s2: &FieldSample,
        v0: Vector3,
        v1: Vector3,
        v2: Vector3,
    ) -> Result<f64, WorkError> {
        if self.end_clips.is_some() {
            return Ok(f64::INFINITY);
        }

        LevelLineWork::charge(1)?;
        let mut bound = f64::INFINITY;
        let mut done0 = usize::MAX;
        let mut done1 = usize::MAX;
        for sample in [s0, s1, s2] {
            if sample.chain != chain {
                continue;
            }

            let (segment, _) = self.locate_on_chain(sample.parameter);
            if segment == done0 || segment == done1 {
                continue;
            }

            done1 = done0;
            done0 = segment;
            let greatest = cs_math::max_f64(
                self.least_over_segment_from_point(segment, v0).0,
                cs_math::max_f64(
                    self.least_over_segment_from_point(segment, v1).0,
                    self.least_over_segment_from_point(segment, v2).0,
                ),
            );
            bound = cs_math::min_f64(bound, greatest);
        }

        Ok(bound)
    }
}

impl ScalarField for ChainField {
    fn as_chain_distance_field(&self) -> Option<&dyn super::ChainDistanceField> {
        Some(self)
    }

    /// C# `ChainField.ValueAt`: [`ChainField::evaluate`]'s value, as the bare field seam asks.
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok(self.evaluate(point)?.value)
    }

    fn kind(&self) -> FieldKind {
        FieldKind::ChainField
    }

    fn as_chain_field(&self) -> Option<&ChainField> {
        Some(self)
    }
}
