//! The ONE field each side group of a band cut reads: the pick field of the chains that seed it -
//! and, where a `CreaseRunOut` leaves one of its corners across that group's faces, the least of
//! that and the run-out's own field (`RunOutField`). Here are the run-out gate ([`run_outs_on`],
//! [`turn_run_outs_on`] and its scope [`BandGroupFieldsRunOutsScope`], kept on the build's
//! [`WorkBudget`](super::WorkBudget) so it travels with the build) and the wrapped field
//! ([`BandGroupFieldsWithRunOuts`], with its constructor, inner field, point query, bounded maxima
//! least queries, field seam and `ChainDistanceField` impl) and the pick under it
//! ([`unwrapped`]), and each group's field ([`of`]).
//!
//! **A side's own field.** Where the rolling ball sets a pass's setbacks, each chain has a field
//! per side as well as its own (`band_gap_setback_fields.rs`). A group is tagged left or right of
//! each member chain by its `side_of_chain` (1 the left side, the triangle that runs a chain edge in
//! the chain's direction, 2 the right, as `chain_on_the_mesh::sides_of` names them), and reads
//! that side's field for that member ([`chain_fields_of`]). `side_of_chain` holds the group's own
//! tag, not a side, for a member joined through its end or carried past a step corner, so only a
//! side member proper ([`side_member_of`]) reads a side field; an end region, an end, carried,
//! grown-over or both-sided member, and every chain with no side fields read the chain's own
//! field - save a single open chain's end region whose sides differ at its end, which reads its
//! cap field, cut to meet each side's rail ([`chain_fields_of_group`]). The run-out finder is
//! [`band_group_run_outs`](super::band_group_run_outs).

use std::rc::Rc;

use indexmap::IndexMap;

use vector_math::Vector3;

use super::bounded_maximum;
use super::{BandGroup, ChainField, PickField, PickFieldArgumentError};
use super::{
    ChainDistanceField, FieldCounters, FieldKind, FieldMaximum, FieldMinimum, FieldSample,
    LevelLineWork, ScalarField, WorkError,
};

/// Whether the side groups read their crease run-outs' fields for
/// the build in force (off wherever no budget is).
///
/// The gate belongs to the BUILD rather than to a thread: no options object reaches from the
/// app's bevel build down to here, and the app builds on the thread pool, so a thread-static gate
/// could not be turned on by an app-level test. Here it travels with the build, on its
/// [`WorkBudget`](super::WorkBudget).
pub fn run_outs_on() -> bool {
    LevelLineWork::current().is_some_and(|budget| budget.run_outs_on.get())
}

thread_local! {
    /// The flow's own setting, for a scope opened before the build that will read it exists.
    /// The thread the scope was
    /// opened on stands for the flow: a build that runs inline on it - an app-level test's
    /// rebuild - captures the setting when its budget is created ([`flow_run_outs_on`]).
    static FLOW_RUN_OUTS_ON: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// What a new build's budget starts its run-out gate at: on while a [`turn_run_outs_on`] scope
/// is open on this flow.
pub fn flow_run_outs_on() -> bool {
    FLOW_RUN_OUTS_ON.with(std::cell::Cell::get)
}

/// Turns run-outs on for the build in force until the returned
/// scope is dropped, which puts back what was there before.
pub fn turn_run_outs_on() -> BandGroupFieldsRunOutsScope {
    BandGroupFieldsRunOutsScope::new()
}

/// Turns run-outs on for the build in force until it is
/// dropped, which puts back what was there before. With no budget in force the scope sets the
/// flow's setting, which the next build started on this flow captures in its budget.
#[derive(Debug)]
#[must_use = "run-outs stay on only while the scope is alive"]
pub struct BandGroupFieldsRunOutsScope {
    /// Whether run-outs were on when the scope was opened - restored when it closes.
    pub was: bool,
    /// The flow's own setting when the scope was opened - restored when it closes.
    was_in_flow: bool,
}

impl BandGroupFieldsRunOutsScope {
    /// Remembers the setting and turns run-outs on.
    pub fn new() -> Self {
        let was = LevelLineWork::current().is_some_and(|budget| budget.run_outs_on.replace(true));
        let was_in_flow = FLOW_RUN_OUTS_ON.with(|flow| flow.replace(true));
        BandGroupFieldsRunOutsScope { was, was_in_flow }
    }
}

impl Default for BandGroupFieldsRunOutsScope {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for BandGroupFieldsRunOutsScope {
    /// Puts back what was there before.
    fn drop(&mut self) {
        if let Some(budget) = LevelLineWork::current() {
            budget.run_outs_on.set(self.was);
        }
        FLOW_RUN_OUTS_ON.with(|flow| flow.set(self.was_in_flow));
    }
}

/// The pick field under a group's field: a wrapped group's inner
/// pick, any other field itself. What asks about the pick's own CHAINS - where two tie, where a
/// bounded end is a known cut - reads this; the run-out has no chain, tie or end plane.
pub fn unwrapped(field: &Rc<dyn ChainDistanceField>) -> Rc<dyn ChainDistanceField> {
    match field.as_with_run_outs() {
        Some(wrapped) => wrapped.inner().clone(),
        None => field.clone(),
    }
}

/// The least of a group's pick field and its run-outs.
///
/// A reading the run-out wins keeps the pick's chain and parameter (the loop walk needs an owner;
/// `CornerFill.TrimOffTheLine` hands the run back to the corner hole, since the chain's own field
/// reads it off its line), and is marked [`FieldSample::clipped`] so `ConvexCellBound` takes no
/// union-of-balls term from it - its `distance - value` is no setback.
pub struct BandGroupFieldsWithRunOuts {
    /// The pick field the run-outs are read beside.
    pub pick: Rc<dyn ChainDistanceField>,
    /// The pick field's sampler.
    pub pick_sample: Rc<dyn Fn(Vector3) -> Result<FieldSample, WorkError>>,
    /// The pick's end planes: the signed reading of a chain's end plane at a point; `None` where no
    /// member is clipped.
    pub end_planes: Option<Rc<dyn Fn(i32, Vector3) -> f64>>,
    /// The run-out fields read beside the pick.
    pub run_outs: Vec<Rc<dyn ChainDistanceField>>,
}

impl BandGroupFieldsWithRunOuts {
    /// A pick field wrapped with its run-outs.
    pub fn new(
        pick: Rc<dyn ChainDistanceField>,
        pick_sample: Rc<dyn Fn(Vector3) -> Result<FieldSample, WorkError>>,
        end_planes: Option<Rc<dyn Fn(i32, Vector3) -> f64>>,
        run_outs: Vec<Rc<dyn ChainDistanceField>>,
    ) -> Self {
        BandGroupFieldsWithRunOuts {
            pick,
            pick_sample,
            end_planes,
            run_outs,
        }
    }

    /// The pick field the run-outs are read beside.
    pub fn inner(&self) -> &Rc<dyn ChainDistanceField> {
        &self.pick
    }

    /// True only when neither the pick nor any run-out field reaches below `margin` over the box
    /// `min`..`max`.
    pub fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        self.pick.nothing_reaches_below(min, max, margin)
            && self
                .run_outs
                .iter()
                .all(|run_out| run_out.nothing_reaches_below(min, max, margin))
    }
}

impl BandGroupFieldsWithRunOuts {
    /// The least of the run-outs alone at `point` (NaN where any
    /// reads NaN, so a broken reading is never hidden by a good one).
    pub fn run_out_value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        let mut least: Option<f64> = None;
        for run_out in &self.run_outs {
            let value = run_out.value_at(point)?;
            if value.is_nan() {
                return Ok(f64::NAN);
            }

            if least.is_none_or(|l| value < l) {
                least = Some(value);
            }
        }

        // No run-outs reads NaN; a wrapper is made only with some (`of`).
        Ok(least.unwrap_or(f64::NAN))
    }

    /// The pick's sample, lowered to any run-out that reads below it -
    /// keeping the pick's chain and parameter and marked clipped (see the type's remarks); a
    /// run-out that cannot reach below the sample is skipped unread.
    pub fn evaluate(&self, point: Vector3) -> Result<FieldSample, WorkError> {
        let mut sample = (self.pick_sample)(point)?;
        for run_out in &self.run_outs {
            if run_out.nothing_reaches_below(point, point, sample.value) {
                continue;
            }

            let value = run_out.value_at(point)?;
            if value < sample.value {
                sample = FieldSample {
                    value,
                    clipped: true,
                    ..sample
                };
            }
        }

        Ok(sample)
    }

    /// The wrapped field's maximum along the segment from `a` to `b`, to `tolerance`.
    pub fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_along_segment_at_least(a, b, tolerance, f64::NEG_INFINITY)
    }

    /// The bounded search over the
    /// wrapped field, clipped by the pick's end planes.
    pub fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        FieldCounters::with(|t| t.max_along += 1);
        LevelLineWork::charge(1)?;
        bounded_maximum::over_segment(
            &|p| self.evaluate(p),
            a,
            b,
            tolerance,
            at_least,
            self.end_planes.as_deref(),
        )
    }

    /// The wrapped field's maximum over the triangle `a`, `b`, `c`, to `tolerance`.
    pub fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.max_over_triangle_bounded(a, b, c, tolerance, f64::INFINITY, f64::NEG_INFINITY)
    }

    /// The wrapped field's maximum over a triangle, stopping once a value above `stop_above` is
    /// found and reading nothing below `at_least`.
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
        bounded_maximum::over_triangle(
            &|p| self.evaluate(p),
            a,
            b,
            c,
            tolerance,
            stop_above,
            at_least,
            None,
            self.end_planes.as_deref(),
        )
    }
}

impl ScalarField for BandGroupFieldsWithRunOuts {
    fn as_chain_distance_field(&self) -> Option<&dyn super::ChainDistanceField> {
        Some(self)
    }

    /// [`BandGroupFieldsWithRunOuts::evaluate`]'s value.
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok(self.evaluate(point)?.value)
    }

    fn kind(&self) -> FieldKind {
        FieldKind::WithRunOuts
    }

    fn as_with_run_outs(&self) -> Option<&BandGroupFieldsWithRunOuts> {
        Some(self)
    }
}

impl BandGroupFieldsWithRunOuts {
    /// The pick's least along a segment, lowered to any run-out
    /// that could reach below it and does.
    pub fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        let mut best = self.pick.min_along_segment(a, b)?;
        for run_out in &self.run_outs {
            let (min, max) = (Vector3::component_min(a, b), Vector3::component_max(a, b));
            if !run_out.nothing_reaches_below(min, max, best.value) {
                let found = run_out.min_along_segment(a, b)?;
                if found.value < best.value {
                    best = found;
                }
            }
        }

        Ok(best)
    }

    /// The pick's least over a triangle, lowered to any
    /// run-out that could reach below it and does.
    pub fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        let mut best = self.pick.min_over_triangle(a, b, c)?;
        for run_out in &self.run_outs {
            let min = Vector3::component_min(a, Vector3::component_min(b, c));
            let max = Vector3::component_max(a, Vector3::component_max(b, c));
            if !run_out.nothing_reaches_below(min, max, best.value) {
                let found = run_out.min_over_triangle(a, b, c)?;
                if found.value < best.value {
                    best = found;
                }
            }
        }

        Ok(best)
    }
}

impl ChainDistanceField for BandGroupFieldsWithRunOuts {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        BandGroupFieldsWithRunOuts::min_along_segment(self, a, b)
    }

    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        BandGroupFieldsWithRunOuts::min_over_triangle(self, a, b, c)
    }

    fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        BandGroupFieldsWithRunOuts::nothing_reaches_below(self, min, max, margin)
    }

    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        BandGroupFieldsWithRunOuts::max_along_segment(self, a, b, tolerance)
    }

    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        BandGroupFieldsWithRunOuts::max_along_segment_at_least(self, a, b, tolerance, at_least)
    }

    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        BandGroupFieldsWithRunOuts::max_over_triangle(self, a, b, c, tolerance)
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
        BandGroupFieldsWithRunOuts::max_over_triangle_bounded(
            self, a, b, c, tolerance, stop_above, at_least,
        )
    }
}

/// A group's point query.
pub type GroupSampler = Rc<dyn Fn(Vector3) -> Result<FieldSample, WorkError>>;

/// Each group's field and its sampler, keyed by group tag in the groups'
/// order. A pick of ONE chain (`only`) reads the bare chain field, so nothing moves under it; a
/// larger pick reads the [`PickField`] of the group's members that CUT (a grow-over member owns its
/// pieces - the loop walk's sample, over every member - but does not cut the face), which for a
/// group seeded by one chain answers bit for bit what that chain answers. A group named in
/// `run_outs` reads the least of that and its run-outs. A pick field refused for its arguments
/// is the error.
#[allow(clippy::type_complexity)]
pub fn of(
    groups: &[BandGroup],
    only: bool,
    fields: &[Rc<ChainField>],
    run_outs: Option<&IndexMap<i32, Vec<Rc<dyn ChainDistanceField>>>>,
) -> Result<
    (
        IndexMap<i32, Rc<dyn ChainDistanceField>>,
        IndexMap<i32, GroupSampler>,
    ),
    PickFieldArgumentError,
> {
    of_sides(groups, only, fields, None, None, run_outs)
}

/// An end region's own field, by its group's tag, read for the region's member chains in place of
/// their own: a single open chain's cap cut per side (`band_gap_setback_fields::CapFields`).
pub type CapFieldsByTag = IndexMap<i32, Rc<ChainField>>;

/// [`of`], where `side_fields`, if given, holds each chain's left and right fields and each group
/// reads its own side's, and an end region named in `caps` reads its cap field
/// ([`chain_fields_of_group`]).
#[allow(clippy::type_complexity)]
pub fn of_sides(
    groups: &[BandGroup],
    only: bool,
    fields: &[Rc<ChainField>],
    side_fields: Option<&[[Rc<ChainField>; 2]]>,
    caps: Option<&CapFieldsByTag>,
    run_outs: Option<&IndexMap<i32, Vec<Rc<dyn ChainDistanceField>>>>,
) -> Result<
    (
        IndexMap<i32, Rc<dyn ChainDistanceField>>,
        IndexMap<i32, GroupSampler>,
    ),
    PickFieldArgumentError,
> {
    let mut group_fields: IndexMap<i32, Rc<dyn ChainDistanceField>> = IndexMap::new();
    let mut samples: IndexMap<i32, GroupSampler> = IndexMap::new();
    for group in groups {
        let fields = chain_fields_of_group(group, fields, side_fields, caps);
        let fields = &fields[..];
        let (mut pick_field, mut sample, end_planes): (
            Rc<dyn ChainDistanceField>,
            GroupSampler,
            Option<Rc<dyn Fn(i32, Vector3) -> f64>>,
        ) = if only {
            let field = fields[0].clone();
            let reader = field.end_plane_reader();
            let sampled = field.clone();
            (field, Rc::new(move |p| sampled.evaluate(p)), reader)
        } else {
            let cutting: Vec<i32> = group
                .members
                .iter()
                .copied()
                .filter(|m| !group.grow_over_members.contains(m))
                .collect();
            let pick = Rc::new(PickField::new(fields.to_vec(), &cutting)?);
            let owning = if cutting.len() == group.members.len() {
                pick.clone()
            } else {
                Rc::new(PickField::new(fields.to_vec(), &group.members)?)
            };
            let reader = pick.end_plane_reader();
            (pick, Rc::new(move |p| owning.evaluate(p)), reader)
        };

        if let Some(here) = run_outs
            .and_then(|r| r.get(&group.tag))
            .filter(|h| !h.is_empty())
        {
            let with_run_outs = Rc::new(BandGroupFieldsWithRunOuts::new(
                pick_field,
                sample,
                end_planes,
                here.clone(),
            ));
            let wrapper = with_run_outs.clone();
            sample = Rc::new(move |p| wrapper.evaluate(p));
            pick_field = with_run_outs;
        }

        group_fields.insert(group.tag, pick_field);
        samples.insert(group.tag, sample);
    }

    Ok((group_fields, samples))
}

/// The chain fields `group` reads, one per chain of the pick: for each member the group is one
/// side of ([`side_member_of`]), that side's field from `side_fields`; for every other chain, or
/// with no side fields, the chain's own field from `fields`.
pub fn chain_fields_of(
    group: &BandGroup,
    fields: &[Rc<ChainField>],
    side_fields: Option<&[[Rc<ChainField>; 2]]>,
) -> Vec<Rc<ChainField>> {
    fields
        .iter()
        .enumerate()
        .map(|(k, own)| {
            let side = side_fields.zip(side_member_of(group, k as i32)).and_then(
                |(sides, side)| match side {
                    SideOfChain::Left => sides.get(k).map(|pair| &pair[0]),
                    SideOfChain::Right => sides.get(k).map(|pair| &pair[1]),
                },
            );
            Rc::clone(side.unwrap_or(own))
        })
        .collect()
}

/// [`chain_fields_of`], where a group whose tag `caps` names reads that cap field for each of its
/// member chains.
pub fn chain_fields_of_group(
    group: &BandGroup,
    fields: &[Rc<ChainField>],
    side_fields: Option<&[[Rc<ChainField>; 2]]>,
    caps: Option<&CapFieldsByTag>,
) -> Vec<Rc<ChainField>> {
    let mut read = chain_fields_of(group, fields, side_fields);
    if let Some(cap) = caps.and_then(|caps| caps.get(&group.tag)) {
        for &m in &group.members {
            if let Some(slot) = read.get_mut(m as usize) {
                *slot = Rc::clone(cap);
            }
        }
    }
    read
}

/// The field a single chain's group `tag` reads: a side's own (1 left, 2 right) where the ball
/// gives side fields, an end region's cap where it has one, and the chain's own `field` otherwise.
pub fn one_chains_field(
    tag: i32,
    field: &Rc<ChainField>,
    side_fields: Option<&[[Rc<ChainField>; 2]]>,
    caps: &CapFieldsByTag,
) -> Rc<ChainField> {
    let side = match (side_fields.and_then(|sides| sides.first()), tag) {
        (Some(pair), 1) => Some(&pair[0]),
        (Some(pair), 2) => Some(&pair[1]),
        _ => caps.get(&tag),
    };
    Rc::clone(side.unwrap_or(field))
}

/// Which side of a chain a group is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SideOfChain {
    /// The side whose triangle runs a chain edge in the chain's direction (`side_of_chain` 1).
    Left,
    /// The other side (`side_of_chain` 2).
    Right,
}

/// Which side of chain `k` `group` is, only where the group is that chain's side proper: a side
/// group (not a free end's end region), with `k` neither an end member (its end stands on the
/// group's face, the group's `side_of_chain` naming the group's own tag there, not a side), a
/// carried member (likewise), a grown-over member nor a both-sided member (both its sides are in
/// the group, so no one side's field stands for it). `None` everywhere else.
pub fn side_member_of(group: &BandGroup, k: i32) -> Option<SideOfChain> {
    let proper = group.end_of < 0
        && !group.end_members.contains(&k)
        && !group.carried_ends.contains_key(&k)
        && !group.grow_over_members.contains(&k)
        && !group.both_sided.contains(&k);
    if !proper {
        return None;
    }
    match group.side_of_chain.get(&k) {
        Some(1) => Some(SideOfChain::Left),
        Some(2) => Some(SideOfChain::Right),
        _ => None,
    }
}

/// Per group tag, the extra chain-distance fields its band reads, in the order the tags were
/// first met.
pub type BandGroupExtraFields = indexmap::IndexMap<i32, Vec<Rc<dyn ChainDistanceField>>>;

/// Every crease run-out at the picks' corners as a field, filed
/// under each group a triangle of its crease belongs to - or `None` with run-outs off or none
/// found. `standing` is the input's standing map where the caller already has one.
#[allow(clippy::too_many_arguments)]
pub fn run_outs(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    size: f64,
    tags: &[i32],
    standing: Option<&super::triangle_edges::StandingMap>,
) -> Option<BandGroupExtraFields> {
    if !run_outs_on() {
        return None;
    }

    let found = super::CreaseRunOut::find(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        size,
        16,
    );
    if found.is_empty() {
        return None;
    }

    let built;
    let standing = match standing {
        Some(standing) => standing,
        None => {
            built = super::triangle_edges::standing_map(indices);
            &built
        }
    };

    let mut by_tag = BandGroupExtraFields::new();
    for run_out in &found {
        let field: Rc<dyn ChainDistanceField> = Rc::new(super::RunOutField::of(run_out));

        // Both crease faces normally land in ONE group; should they not, each group that holds
        // one reads the wedge.
        let mut seen = Vec::new();
        for &t in &standing[&super::triangle_edges::key(run_out.corner, run_out.crease_end)] {
            let tag = tags[t as usize];
            if seen.contains(&tag) {
                continue;
            }

            seen.push(tag);
            by_tag.entry(tag).or_default().push(field.clone());
        }
    }

    Some(by_tag)
}
