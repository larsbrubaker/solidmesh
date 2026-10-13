//! The chain field's LEAST queries: the least value along a segment of the mesh or over a
//! triangle, exact, and where it stands. Ports the least-value methods of
//! `Bevel/LevelLine/ChainField.Queries.cs` (`MinAlongSegment`, `MinOverTriangle`, `LeastOverPiece`,
//! `VisitLeast`, `LeastOverPieceExhaustive`, `LeastOverSegments` and the mesh piece's
//! `ClosestApproachTo`/`ClippedApproachTo`), with `ChainField.Mitre.cs`'s `LeastOverExtensions` and
//! `ChainField.EndClip.cs`'s `ClipLeast`, split out of `chain_field_queries.rs` for length.
//!
//! Over ONE chain segment `|P - C(u)| - setback(u)` is convex jointly in the piece's point and the
//! chain parameter, so partially minimising over the piece (its closed-form closest point) leaves
//! a convex function of u alone, found by a coarse scan and a fixed-step ternary search. Where a
//! segment's two ends ask the SAME setback no search is run: the least is the piece's closest
//! approach to that segment less the setback, in closed form (`chain_approach`).
//!
//! Here too `ChainField`'s [`ChainDistanceField`] impl, which these queries complete.

use super::chain_approach;
use super::chain_field_clip::clip_to_keep;
use super::segment_box_tree;
use super::{
    ChainDistanceField, ChainField, ChainFieldMeshPiece, FieldCounters, FieldMaximum, FieldMinimum,
    LevelLineWork, WorkError,
};
use vector_math::Vector3;

/// The held answer of a least search: (value, point on the piece, chain segment).
type Best = (f64, Vector3, i32);

impl ChainFieldMeshPiece {
    /// C# `MeshPiece.ClosestApproachTo`: the closest approach between this piece and a chain
    /// segment, and the point of the piece attaining it.
    pub fn closest_approach_to(&self, from: Vector3, to: Vector3) -> (f64, Vector3) {
        if self.is_triangle {
            chain_approach::triangle_to_segment(self.a, self.b, self.c, from, to)
        } else {
            chain_approach::segment_to_segment(self.a, self.b, from, to)
        }
    }

    /// C# `MeshPiece.ClippedApproachTo`: the closest approach between the part of this piece on
    /// the kept side of a plane through `from` (normal `keep`) and a chain segment; +infinity when
    /// no part of the piece is on that side (or a triangle only touches the plane: the other ray
    /// reads that boundary identically).
    pub fn clipped_approach_to(&self, from: Vector3, to: Vector3, keep: Vector3) -> (f64, Vector3) {
        let corners = [self.a, self.b, self.c];
        let polygon = if self.is_triangle {
            &corners[..]
        } else {
            &corners[..2]
        };
        let clipped = clip_to_keep(polygon, from, keep);
        if !self.is_triangle {
            // A segment clips to a segment (its two kept ends, or one end and the crossing).
            return if clipped.len() < 2 {
                (f64::INFINITY, from)
            } else {
                chain_approach::segment_to_segment(clipped[0], clipped[1], from, to)
            };
        }

        if clipped.len() < 3 {
            return (f64::INFINITY, from);
        }

        let mut best =
            chain_approach::triangle_to_segment(clipped[0], clipped[1], clipped[2], from, to);
        if clipped.len() == 4 {
            let other =
                chain_approach::triangle_to_segment(clipped[0], clipped[2], clipped[3], from, to);
            if other.0 < best.0 {
                best = other;
            }
        }

        best
    }
}

impl ChainField {
    /// C# `ChainField.MinAlongSegment`: the least field value along a segment of the mesh - exact,
    /// not a bound - and where it stands. Asked by the refinement of an edge whose two ends are
    /// both outside the line: a minimum below zero means the edge dips across.
    pub fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        FieldCounters::with(|t| t.min_along += 1);
        LevelLineWork::charge(1)?;
        let min = Vector3::component_min(a, b);
        let max = Vector3::component_max(a, b);
        self.least_over_piece(min, max, &ChainFieldMeshPiece::segment(a, b))
    }

    /// C# `ChainField.MinOverTriangle`: the least field value over a whole triangle, interior
    /// included - the only way to see a loop of the line that lies wholly inside one big flat
    /// triangle and touches none of its edges.
    pub fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        FieldCounters::with(|t| t.min_tri += 1);
        LevelLineWork::charge(1)?;
        let min = Vector3::component_min(Vector3::component_min(a, b), c);
        let max = Vector3::component_max(Vector3::component_max(a, b), c);
        self.least_over_piece(min, max, &ChainFieldMeshPiece::triangle(a, b, c))
    }

    /// C# `ChainField.LeastOverPiece`: the least of `|P - C(u)| - setback(u)` over a convex piece
    /// of the mesh (boxed by `piece_min`..`piece_max`) and the whole chain, its rays and its end
    /// clips.
    pub fn least_over_piece(
        &self,
        piece_min: Vector3,
        piece_max: Vector3,
        piece: &ChainFieldMeshPiece,
    ) -> Result<FieldMinimum, WorkError> {
        let mut best: Best = (f64::INFINITY, self.points[0], i32::MAX);
        self.visit_least(0, piece_min, piece_max, piece, &mut best)?;
        self.clip_least(piece, &mut best)?;
        Ok(FieldMinimum::new(best.0, best.1))
    }

    /// C# `ChainField.VisitLeast`: the box tree's walk for [`ChainField::least_over_piece`]: a
    /// subtree whose whole box stands far enough from the piece that even its greatest setback
    /// cannot bring a value below the best found so far (not even to a tie, which might come from
    /// a lower segment) is skipped whole, and the nearer child is visited FIRST.
    pub fn visit_least(
        &self,
        node: usize,
        piece_min: Vector3,
        piece_max: Vector3,
        piece: &ChainFieldMeshPiece,
        best: &mut Best,
    ) -> Result<(), WorkError> {
        let here = self.nodes[node];
        let bound =
            segment_box_tree::box_gap(piece_min, piece_max, here.min, here.max) - here.max_setback;
        if best.0 != f64::INFINITY && bound > best.0 + (Self::TIE_RELATIVE * (1.0 + best.0.abs())) {
            return Ok(());
        }

        if here.left < 0 {
            self.least_over_segments(here.start, here.start + here.count, piece, best)?;
            self.least_over_extensions(here.start, here.start + here.count, piece, best);
            return Ok(());
        }

        let left = self.nodes[here.left as usize];
        let right = self.nodes[here.right as usize];
        let left_bound =
            segment_box_tree::box_gap(piece_min, piece_max, left.min, left.max) - left.max_setback;
        let right_bound = segment_box_tree::box_gap(piece_min, piece_max, right.min, right.max)
            - right.max_setback;
        let (first, second) = if left_bound <= right_bound {
            (here.left, here.right)
        } else {
            (here.right, here.left)
        };
        self.visit_least(first as usize, piece_min, piece_max, piece, best)?;
        self.visit_least(second as usize, piece_min, piece_max, piece, best)
    }

    /// C# `ChainField.LeastOverPieceExhaustive`: the reference for
    /// [`ChainField::least_over_piece`] - every segment searched, in order, no index, no closed
    /// form - over a piece known only by its closest-point function.
    pub fn least_over_piece_exhaustive(
        &self,
        closest_on_piece: super::ClosestOnPiece,
    ) -> Result<FieldMinimum, WorkError> {
        let mut best: Best = (f64::INFINITY, self.points[0], i32::MAX);
        self.least_over_segments(
            0,
            self.segment_count(),
            &ChainFieldMeshPiece::custom(closest_on_piece),
            &mut best,
        )?;
        Ok(FieldMinimum::new(best.0, best.1))
    }

    /// C# `ChainField.LeastOverSegments`: the closed form where the setback allows it, and the
    /// convex search where it does not, over the chain segments `first..end`. A coarse scan first,
    /// then the ternary search inside the sub-interval that won it; the ends are taken outright as
    /// well, since a minimum on u = 0 or 1 (a chain corner) is the common case.
    pub fn least_over_segments(
        &self,
        first: i32,
        end: i32,
        piece: &ChainFieldMeshPiece,
        best: &mut Best,
    ) -> Result<(), WorkError> {
        let n = self.points.len();
        for i in first..end {
            FieldCounters::with(|t| t.segment_searches += 1);
            LevelLineWork::charge(1)?;
            let segment = i as usize;
            let from = self.points[segment];
            let to = self.points[(segment + 1) % n];
            let setback_from = self.setbacks[segment];
            let setback_to = self.setbacks[(segment + 1) % n];

            if setback_from == setback_to && piece.has_closed_form() {
                FieldCounters::with(|t| t.closed_forms += 1);
                let (approach, on_approach) = piece.closest_approach_to(from, to);
                Self::offer(i, approach - setback_from, on_approach, best);
                continue;
            }

            let at = |u: f64| {
                let chain_point = from + ((to - from) * u);
                let on_piece = piece.closest_to(chain_point);
                (
                    (on_piece - chain_point).length()
                        - (setback_from + ((setback_to - setback_from) * u)),
                    on_piece,
                )
            };

            let scan_steps = Self::SCAN_STEPS;
            let mut scan_best = 0;
            let mut scan_value = f64::INFINITY;
            for i2 in 0..=scan_steps {
                let value = at(i2 as f64 / scan_steps as f64).0;
                if value < scan_value {
                    scan_value = value;
                    scan_best = i2;
                }
            }

            let mut lo =
                vector_math::cs_math::max_f64(0.0, (scan_best - 1) as f64 / scan_steps as f64);
            let mut hi =
                vector_math::cs_math::min_f64(1.0, (scan_best + 1) as f64 / scan_steps as f64);
            for _ in 0..Self::TERNARY_STEPS {
                LevelLineWork::poll_cancel()?;
                let (was_lo, was_hi) = (lo, hi);
                let third = (hi - lo) / 3.0;
                let left = at(lo + third).0;
                let right = at(hi - third).0;
                if left < right {
                    hi -= third;
                } else {
                    lo += third;
                }

                // A step is a function of (lo, hi) alone, so once the bracket stops moving (a
                // third too small to change either end) every remaining step repeats it: the
                // same bits as running them out, for the cost of none of their field reads.
                if lo == was_lo && hi == was_hi {
                    break;
                }
            }

            for u in [0.0, 0.5 * (lo + hi), 1.0] {
                let (value, on_piece) = at(u);
                Self::offer(i, value, on_piece, best);
            }
        }

        Ok(())
    }

    /// C# `ChainField.LeastOverExtensions`: offers the rays of the segments `first..end` to a
    /// least-value query: the piece is clipped to each ray's side of the bisector and the
    /// closed-form approach taken. A piece with no geometry cannot be clipped, so it reads the plain
    /// polyline only.
    pub fn least_over_extensions(
        &self,
        first: i32,
        end: i32,
        piece: &ChainFieldMeshPiece,
        best: &mut Best,
    ) {
        let Some(extensions) = &self.extensions else {
            return;
        };
        if !piece.has_closed_form() {
            return;
        }

        for i in first..end {
            let Some(rays) = &extensions[i as usize] else {
                continue;
            };

            for ray in rays {
                let (approach, on_piece) =
                    piece.clipped_approach_to(ray.origin, ray.end(), ray.keep);
                if approach != f64::INFINITY {
                    Self::offer(i, approach - ray.setback, on_piece, best);
                }
            }
        }
    }

    /// C# `ChainField.ClipLeast`: the least of the CLIPPED field over a piece, given the unclipped
    /// least and where it stands. Where the end plane does not stand above the unclipped least at
    /// its minimiser, that least is attained and stands. Otherwise each term (segment, ray) whose
    /// own unclipped least could beat the plane's reading there is searched for the least of
    /// `max(term, plane)`. Never below the unclipped least, which bounds it. A piece with no
    /// geometry is returned unclipped.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn clip_least(
        &self,
        piece: &ChainFieldMeshPiece,
        best: &mut Best,
    ) -> Result<(), WorkError> {
        if self.end_clips.is_none() || !piece.has_closed_form() || best.0 == f64::INFINITY {
            return Ok(());
        }

        let plane_at_best = self.clip_at(best.1).0;
        if !(plane_at_best > best.0) {
            // The unclipped least is a lower bound on the clipped one, and here it is attained.
            return Ok(());
        }

        // The clipped field at the unclipped minimiser is the plane's reading: the value to beat.
        let unclipped = best.0;
        best.0 = plane_at_best;
        for i in 0..self.segment_count() {
            let mut term: Best = (f64::INFINITY, best.1, i32::MAX);
            self.least_over_segments(i, i + 1, piece, &mut term)?;
            if term.0 < best.0 + (Self::TIE_RELATIVE * (1.0 + best.0.abs())) {
                let segment = i as usize;
                let (value, at) = self.least_of_clipped_term(piece, None, &|p| {
                    self.least_over_segment_from_point(segment, p).0
                });
                Self::offer(i, value, at, best);
            }

            let Some(rays) = self
                .extensions
                .as_ref()
                .and_then(|e| e[i as usize].as_ref())
            else {
                continue;
            };

            for ray in rays {
                let (approach, _) = piece.clipped_approach_to(ray.origin, ray.end(), ray.keep);
                if approach == f64::INFINITY
                    || approach - ray.setback
                        >= best.0 + (Self::TIE_RELATIVE * (1.0 + best.0.abs()))
                {
                    continue;
                }

                let (value, at) =
                    self.least_of_clipped_term(piece, Some(ray), &|p| ray.value_at(p).0);
                Self::offer(i, value, at, best);
            }
        }

        // Never below the unclipped least, which bounds it: a search's rounding cannot take it
        // there.
        if best.0 < unclipped {
            best.0 = unclipped;
        }

        Ok(())
    }
}

impl ChainDistanceField for ChainField {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        ChainField::min_along_segment(self, a, b)
    }

    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        ChainField::min_over_triangle(self, a, b, c)
    }

    fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        ChainField::nothing_reaches_below(self, min, max, margin)
    }

    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        ChainField::max_along_segment(self, a, b, tolerance)
    }

    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        ChainField::max_along_segment_at_least(self, a, b, tolerance, at_least)
    }

    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        ChainField::max_over_triangle(self, a, b, c, tolerance)
    }

    fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        ChainField::max_over_triangle_bounded(self, a, b, c, tolerance, stop_above, at_least)
    }

    fn island_peak(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        snap: f64,
    ) -> Result<FieldMaximum, WorkError> {
        ChainField::island_peak(self, a, b, c, tolerance, snap)
    }
}
