//! One field over a GROUP of chains ([`PickField`]): the type with its construction
//! ([`PickField::new`], refusing bad arguments with [`PickFieldArgumentError`]), its member
//! read-throughs and its
//! end-plane reader ([`PickField::end_plane_reader`]) and whether an edge lies in a member's end
//! plane ([`PickField::lies_in_an_end_plane`]) or a point in a member's stopped end plane
//! ([`PickField::lies_in_a_stopped_end_plane`]), the point query ([`PickField::evaluate`]) and its
//! readings, the bounded maximum queries and the field seam ([`ScalarField`], saying it is a
//! pick's field), the least queries ([`PickField::min_along_segment`],
//! [`PickField::min_over_triangle`]) and the [`ChainDistanceField`] impl they complete; the swallow
//! scale and the `ChainDistanceField`
//! impl land with their method packages.

use std::cell::OnceCell;
use std::rc::Rc;

use vector_math::{ieee_math, Vector3};

use super::bounded_maximum;
use super::stopwatch;
use super::{
    ChainDistanceField, ChainField, ChainFieldEndPlaneReader, FieldCounters, FieldKind,
    FieldMaximum, FieldMinimum, FieldSample, LevelLineWork, ScalarField, WorkError,
};

/// One field over a GROUP of chains - the least value any of the group's chains
/// reads, so a band that several picked loops share is cut once, in one pass, whatever order the
/// loops were picked in.
///
/// A face's rim is an outer loop plus one per hole, and a user picks several at once. Building them
/// one after another on the evolving mesh makes a later loop find its chain edge already gone,
/// makes the pick's room not one number, and makes the result depend on loop order. The cure is
/// that every rule downstream reads ONE field - this one - whose value at a point is `min over the
/// group's chains k of value_k(point)`.
///
/// **Everything the seam promises still holds.** Each member is exactly 1-Lipschitz (it is a
/// minimum of balls; see [`ChainField`]), and a minimum of 1-Lipschitz functions is 1-Lipschitz,
/// so the bounded greatest-value searches keep their bound unchanged. **The exact minima
/// commute:** `min_q min_k v_k(q) = min_k min_q v_k(q)`, so the pick's answer is still ATTAINED at
/// the point it reports, not merely bounded.
///
/// **The tie rule.** Within a chain ties go to the lower parameter; BETWEEN chains they go to the
/// LOWER CHAIN INDEX, by the same strictly-less-with-a-few-bits-of-room comparison, walking the
/// members in ascending index order. A point on the midline of two parallel chains must read the
/// same chain for every one of its neighbours, or the loops the cut leaves are not each owned by
/// one chain.
///
/// **A pick of ONE is today's field to the bit**: every query over a single chain forwards to that
/// chain and reports chain 0. **Work budget**: a point query charges ONE unit per member chain
/// actually evaluated - the members' own charges, with nothing added here. **Why the members are
/// all read**: on the Many Holes witness every later member read won against the running best
/// (4.68M of 4.68M), because members are walked in index order, not nearest first.
pub struct PickField {
    /// Every chain the pick knows, indexed by `FieldSample::chain`.
    pub chains: Vec<Rc<ChainField>>,
    /// The chains of THIS group, in ascending index order.
    pub members: Vec<i32>,
    /// The end-plane reader over the members, made once on first ask.
    pub end_plane_reader: OnceCell<ChainFieldEndPlaneReader>,
    /// Whether any member is clipped by an end plane, found once on first ask.
    pub any_member_clipped: OnceCell<bool>,
    /// The smallest setback any point of any member chain asks for.
    pub min_setback: f64,
    /// The largest setback any point of any member chain asks for.
    pub max_setback: f64,
}

impl std::fmt::Debug for PickField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The end-plane reader has no Debug; leave it and the caches out.
        f.debug_struct("PickField")
            .field("chains", &self.chains)
            .field("members", &self.members)
            .field("min_setback", &self.min_setback)
            .field("max_setback", &self.max_setback)
            .finish_non_exhaustive()
    }
}

impl PickField {
    /// True only when NO member chain reaches below `margin` over the box `min`..`max` - one chain
    /// that might is enough to make the caller pay for the exact search.
    pub fn nothing_reaches_below(
        &self,
        min: vector_math::Vector3,
        max: vector_math::Vector3,
        margin: f64,
    ) -> bool {
        self.members
            .iter()
            .all(|&k| self.chains[k as usize].nothing_reaches_below(min, max, margin))
    }
}

/// Why [`PickField::new`] refused its arguments: a caller's bug, reported with a message rather
/// than a panic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickFieldArgumentError {
    /// No member chains.
    NoMembers,
    /// A member index outside the chains.
    MemberOutOfRange,
    /// Members repeated or out of order.
    MembersNotAscending,
}

impl std::fmt::Display for PickFieldArgumentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PickFieldArgumentError::NoMembers => "A pick field needs at least one chain.",
            PickFieldArgumentError::MemberOutOfRange => {
                "A member names a chain the pick does not hold."
            }
            PickFieldArgumentError::MembersNotAscending => {
                "A group's members must be distinct and in ascending order."
            }
        })
    }
}

impl std::error::Error for PickFieldArgumentError {}

impl PickField {
    /// A tie between two chains is two arithmetics answering the same number, so "the same" has
    /// to allow a few bits of drift - the same relative rule `ChainField` uses between its own
    /// segments.
    pub const TIE_RELATIVE: f64 = 1e-12;

    /// The field over `member_chains` (distinct, ascending indices into `all_chains`), its
    /// setbacks the least and greatest of the members'.
    pub fn new(
        all_chains: Vec<Rc<ChainField>>,
        member_chains: &[i32],
    ) -> Result<PickField, PickFieldArgumentError> {
        if member_chains.is_empty() {
            return Err(PickFieldArgumentError::NoMembers);
        }

        let mut members = Vec::with_capacity(member_chains.len());
        let mut min_setback = f64::INFINITY;
        let mut max_setback = f64::NEG_INFINITY;

        for (i, &k) in member_chains.iter().enumerate() {
            if k < 0 || k as usize >= all_chains.len() {
                return Err(PickFieldArgumentError::MemberOutOfRange);
            }

            if i > 0 && k <= members[i - 1] {
                return Err(PickFieldArgumentError::MembersNotAscending);
            }

            members.push(k);
            let chain = &all_chains[k as usize];
            min_setback = ieee_math::min_f64(min_setback, chain.min_setback);
            max_setback = ieee_math::max_f64(max_setback, chain.max_setback);
        }

        Ok(PickField {
            chains: all_chains,
            members,
            end_plane_reader: OnceCell::new(),
            any_member_clipped: OnceCell::new(),
            min_setback,
            max_setback,
        })
    }

    /// The field of ONE chain - today's field to the bit.
    pub fn of(chain: Rc<ChainField>) -> PickField {
        Self::new(vec![chain], &[0]).expect("one chain, member 0, is always a valid pick")
    }

    /// Every chain the pick knows.
    pub fn chains(&self) -> &[Rc<ChainField>] {
        &self.chains
    }

    /// The chains of this group, in ascending index order.
    pub fn members(&self) -> &[i32] {
        &self.members
    }
}

impl PickField {
    /// The members' end planes, read per chain - what the convex cell bound needs - or `None` when
    /// no member is clipped. Made once; it holds its own handles on the chains.
    pub fn end_plane_reader(&self) -> Option<ChainFieldEndPlaneReader> {
        let any_clipped = *self.any_member_clipped.get_or_init(|| {
            self.members
                .iter()
                .any(|&k| self.chains[k as usize].end_plane_reader().is_some())
        });
        if !any_clipped {
            return None;
        }

        Some(Rc::clone(self.end_plane_reader.get_or_init(|| {
            let chains = self.chains.clone();
            Rc::new(move |k: i32, point: vector_math::Vector3| {
                chains[k as usize].end_plane_at(point)
            })
        })))
    }
}

impl PickField {
    /// Whether a-b lies in an end plane of any member chain
    /// ([`ChainField::lies_in_an_end_plane`]).
    pub fn lies_in_an_end_plane(&self, a: Vector3, b: Vector3) -> bool {
        self.members
            .iter()
            .any(|&k| self.chains[k as usize].lies_in_an_end_plane(a, b))
    }

    /// Whether a stopped end clips any member chain ([`ChainField::is_clipped_by_a_stop`]).
    pub fn has_a_stopped_member(&self) -> bool {
        self.members
            .iter()
            .any(|&k| self.chains[k as usize].is_clipped_by_a_stop())
    }

    /// Whether a point stands in a stopped end's plane of any member chain
    /// ([`ChainField::lies_in_a_stopped_end_plane`]).
    pub fn lies_in_a_stopped_end_plane(&self, point: Vector3) -> bool {
        self.members
            .iter()
            .any(|&k| self.chains[k as usize].lies_in_a_stopped_end_plane(point))
    }
}

/// The least of `values`: NaN where any value is NaN, so a broken reading is never hidden (an
/// empty pick has none).
fn min_or_nan(values: impl Iterator<Item = Result<f64, WorkError>>) -> Result<f64, WorkError> {
    let mut least: Option<f64> = None;
    for value in values {
        let value = value?;
        if value.is_nan() {
            return Ok(f64::NAN);
        }

        if least.is_none_or(|l| value < l) {
            least = Some(value);
        }
    }

    // Never empty: a pick always holds a member (`PickField::new` refuses none).
    Ok(least.unwrap_or(f64::NAN))
}

impl PickField {
    /// The least member reading at a point, named by the member chain
    /// that gave it (see [`PickField::evaluate_members`]); timed under the meter.
    pub fn evaluate(&self, point: Vector3) -> Result<FieldSample, WorkError> {
        FieldCounters::with(|t| t.pick_evaluate += 1);
        if FieldCounters::meter_on() {
            // Timed around the reading rather than inside it so the meter-off path stays one bool
            // read.
            let reads_before = FieldCounters::with(|t| t.evaluate);
            let started_at = stopwatch::get_timestamp();
            let timed = self.evaluate_members(point)?;
            let ticks = stopwatch::get_timestamp() - started_at;
            FieldCounters::with(|t| {
                t.pick_evaluate_ticks += ticks;
                t.pick_member_reads += t.evaluate - reads_before;
            });
            return Ok(timed);
        }

        self.evaluate_members(point)
    }

    /// The least reading over the members, a member that cannot
    /// reach below the best so far skipped unread, and a later member taking over only by beating
    /// the best by more than a relative tie - so a tie goes to the lower member index.
    pub fn evaluate_members(&self, point: Vector3) -> Result<FieldSample, WorkError> {
        let mut best = FieldSample::default();
        let mut have = false;
        for &k in &self.members {
            let chain = &self.chains[k as usize];
            if have && chain.nothing_reaches_below(point, point, best.value) {
                continue;
            }

            let sample = chain.evaluate(point)?;
            if !have || sample.value < best.value - (Self::TIE_RELATIVE * (1.0 + best.value.abs()))
            {
                best = FieldSample { chain: k, ..sample };
                have = true;
            }
        }

        Ok(best)
    }

    /// The least of the members'
    /// [`ChainField::value_across_the_known_cut`] readings.
    pub fn value_across_the_known_cut(&self, point: Vector3) -> Result<f64, WorkError> {
        min_or_nan(
            self.members
                .iter()
                .map(|&k| self.chains[k as usize].value_across_the_known_cut(point)),
        )
    }

    /// The least member reading along an edge a-b lying in an
    /// end plane: unclipped for a member whose end plane the edge lies in, across the known cut for
    /// the rest.
    pub fn value_in_the_end_plane(
        &self,
        point: Vector3,
        a: Vector3,
        b: Vector3,
    ) -> Result<f64, WorkError> {
        min_or_nan(self.members.iter().map(|&k| {
            let chain = &self.chains[k as usize];
            if chain.lies_in_an_end_plane(a, b) {
                chain.value_in_the_end_plane(point)
            } else {
                chain.value_across_the_known_cut(point)
            }
        }))
    }

    /// The setback the sample's own chain asks at its parameter.
    pub fn setback_of(&self, sample: &FieldSample) -> f64 {
        self.chains[sample.chain as usize].setback_at(sample.parameter)
    }

    /// The greatest pick value along a segment.
    pub fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_along_segment_at_least(a, b, tolerance, f64::NEG_INFINITY)
    }

    /// As the chain's, over the pick.
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

    /// The greatest pick value over a triangle.
    pub fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_over_triangle_bounded(a, b, c, tolerance, f64::INFINITY, f64::NEG_INFINITY)
    }

    /// As the chain's, over
    /// the pick.
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

    /// The island rule's search over the pick, sharpened by the member
    /// chains' segment cell bounds ([`PickField::segment_cell_bound`]).
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
            Some(&|s0, s1, s2, v0, v1, v2| self.segment_cell_bound(s0, s1, s2, v0, v1, v2)),
            reader.as_deref(),
        )
    }

    /// The least of the segment cell bounds of the (distinct)
    /// chains the three corner samples name, each read from that chain's own samples.
    pub fn segment_cell_bound(
        &self,
        s0: &FieldSample,
        s1: &FieldSample,
        s2: &FieldSample,
        v0: Vector3,
        v1: Vector3,
        v2: Vector3,
    ) -> Result<f64, WorkError> {
        let chain = |k: i32| &self.chains[k as usize];
        let mut bound = chain(s0.chain).segment_cell_bound(s0.chain, s0, s1, s2, v0, v1, v2)?;
        if s1.chain != s0.chain {
            bound = ieee_math::min_f64(
                bound,
                chain(s1.chain).segment_cell_bound(s1.chain, s0, s1, s2, v0, v1, v2)?,
            );
        }

        if s2.chain != s0.chain && s2.chain != s1.chain {
            bound = ieee_math::min_f64(
                bound,
                chain(s2.chain).segment_cell_bound(s2.chain, s0, s1, s2, v0, v1, v2)?,
            );
        }

        Ok(bound)
    }
}

impl ScalarField for PickField {
    fn as_chain_distance_field(&self) -> Option<&dyn super::ChainDistanceField> {
        Some(self)
    }

    /// [`PickField::evaluate`]'s value.
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok(self.evaluate(point)?.value)
    }

    fn kind(&self) -> FieldKind {
        FieldKind::PickField
    }

    fn as_pick_field(&self) -> Option<&PickField> {
        Some(self)
    }
}

impl PickField {
    /// The same pick with every MEMBER chain's setbacks times
    /// `factor`; the other chains it holds are shared unchanged.
    pub fn with_setbacks_scaled(&self, factor: f64) -> Result<PickField, WorkError> {
        let mut scaled = self.chains.clone();

        for &k in &self.members {
            scaled[k as usize] = Rc::new(self.chains[k as usize].with_setbacks_scaled(factor)?);
        }

        Ok(PickField::new(scaled, &self.members).expect("a pick's own members are valid"))
    }
}

impl PickField {
    /// The least of the members' least along a segment, a later
    /// member taking over only by beating the best by more than a relative tie.
    pub fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        let mut best = FieldMinimum::default();
        let mut have = false;
        for &k in &self.members {
            let found = self.chains[k as usize].min_along_segment(a, b)?;
            if !have || found.value < best.value - (Self::TIE_RELATIVE * (1.0 + best.value.abs())) {
                best = found;
                have = true;
            }
        }

        Ok(best)
    }

    /// The least of the members' least over a triangle, ties
    /// to the lower member.
    pub fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        let mut best = FieldMinimum::default();
        let mut have = false;
        for &k in &self.members {
            let found = self.chains[k as usize].min_over_triangle(a, b, c)?;
            if !have || found.value < best.value - (Self::TIE_RELATIVE * (1.0 + best.value.abs())) {
                best = found;
                have = true;
            }
        }

        Ok(best)
    }
}

impl ChainDistanceField for PickField {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        PickField::min_along_segment(self, a, b)
    }

    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        PickField::min_over_triangle(self, a, b, c)
    }

    fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        PickField::nothing_reaches_below(self, min, max, margin)
    }

    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        PickField::max_along_segment(self, a, b, tolerance)
    }

    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        PickField::max_along_segment_at_least(self, a, b, tolerance, at_least)
    }

    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        PickField::max_over_triangle(self, a, b, c, tolerance)
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
        PickField::max_over_triangle_bounded(self, a, b, c, tolerance, stop_above, at_least)
    }

    fn island_peak(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        snap: f64,
    ) -> Result<FieldMaximum, WorkError> {
        PickField::island_peak(self, a, b, c, tolerance, snap)
    }
}
