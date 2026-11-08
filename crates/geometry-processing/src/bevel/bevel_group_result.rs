//! What one group did on the rebuild that has just run ([`BevelGroupResult`], with the rim loops it
//! refused while building the rest, [`BevelLoopFailure`]) and the second readings of a group
//! failure where the refusal the geometry produced is not what went wrong ([`bevel_group_failures`]).
//! A whole part's rows travel in `part::BevelPartOutcome`.
//!
//! Codes and numbers only. Every refusal and note is a [`BevelAdvisory`] - which thing happened and
//! the values it quotes - and the caller puts it into words (MatterCAD's `BevelAdvisoryText`), so
//! nothing here is a sentence and nothing here is localized. A caller asks by code, never by prose.
//!
//! A row carries the group's identity rather than a pre-formatted "Group 3 was skipped: ..."
//! string, because the property panel already knows which row it is drawing and needs the reason
//! on its own to put beside that row. Never serialized: it describes one particular rebuild
//! against one particular mesh, and a file that has just been loaded has not run that rebuild
//! yet: a stored "this group failed" would be a claim about geometry nobody has looked at.

use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::{BevelCornerGroup, BevelEdgeGroup};

/// The group a row is about, whichever leg of the operation resolved it: one of the two concrete
/// kinds ([`BevelGroup`] is a trait and cannot be held by value). A copy of the group as it was
/// resolved, so a caller can find the row again after the list is reordered by comparing values:
/// groups are immutable once resolved (every edit copies), so the copy is the group the row was
/// made from, and two groups equal field for field are the same selection.
///
/// [`BevelGroup`]: crate::topology::BevelGroup
#[derive(Debug, Clone, PartialEq)]
pub enum BevelResultGroup {
    /// A mesh leg group (`BevelFeatureObject3D.Groups`).
    Edge(BevelEdgeGroup),
    /// A 2D leg group (`BevelFeatureObject3D.CornerGroups`).
    Corner(BevelCornerGroup),
}

/// One row of a rebuild's report: what happened to one group. The three positional members,
/// `failure` and `warnings` are the constructor's ([`BevelGroupResult::new`]); the rest are left at
/// their defaults by `new` and set by the composer.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelGroupResult {
    /// Where the group sat in the selection it came from - `BevelFeatureObject3D.Groups` on the
    /// mesh leg, `BevelFeatureObject3D.CornerGroups` on the 2D one - when it was resolved.
    pub group_index: i32,

    /// The group itself, as it was resolved.
    pub group: BevelResultGroup,

    /// How many mesh edges it resolved to, or - on the 2D leg - how many path corners, which is one
    /// or none. Zero when it failed to resolve at all.
    pub edge_count: i32,

    /// Why it was skipped - WHICH refusal, with the values it quotes - or `None` when it was applied.
    pub failure: Option<BevelAdvisory>,

    /// What the group did that the user should know about but which did not stop it being
    /// applied: a run that stops short at one or both ends (a run can have two ends and a group
    /// several runs). Empty when there was nothing to say; a caller with room for one line says
    /// them in order, as one.
    pub warnings: Vec<BevelAdvisory>,

    /// The largest size that WOULD have built this group on its own, where a refused row has one -
    /// `None` on a row that built, and where no size builds it at all.
    ///
    /// The one thing a user can do about a refusal they did not ask for, so it travels beside the
    /// refusal rather than only inside it: a size refusal found the number on its way to saying no
    /// (`bevel_advisory_codes::try_the_most_that_fits`), and a BUILD refusal is bisected for it
    /// afterwards. A caller says it once: after the refusal's own words, unless the refusal already
    /// quotes it (the size refusals' room, `PointTooLargeRoomFor`).
    pub largest_size_that_fits: Option<f64>,

    /// The rim loops of this group that were refused while the rest of it built, empty or `None`
    /// on a row where every loop came to the same answer.
    ///
    /// A face group's rim is one closed loop per hole in the region - forty-five of them on the
    /// barrel of a flash hider, forty-one of which are port mouths - and those loops share no edge
    /// and no face, so they are independent bevels that happen to have been picked with one click.
    /// One of them that cannot be built is no reason for the other forty-four not to be, which is
    /// the same rule that keeps one refused GROUP from taking down the groups beside it; what it
    /// costs is that a row can now be built and still have something to say, which is what this
    /// carries.
    pub loop_failures: Option<Vec<BevelLoopFailure>>,

    /// How many rim loops the group's selection came to, which is what a refused loop is counted
    /// against - "one of forty-five" is the sentence, and one of one would not be worth saying.
    pub loop_count: i32,
}

impl BevelGroupResult {
    /// A row from its positional members: which group, how many edges, why it was skipped (`None`
    /// when it was applied) and what it has to say having been applied.
    pub fn new(
        group_index: i32,
        group: BevelResultGroup,
        edge_count: i32,
        failure: Option<BevelAdvisory>,
        warnings: Vec<BevelAdvisory>,
    ) -> Self {
        BevelGroupResult {
            group_index,
            group,
            edge_count,
            failure,
            warnings,
            largest_size_that_fits: None,
            loop_failures: None,
            loop_count: 0,
        }
    }

    /// Whether this group was skipped rather than cut.
    pub fn failed(&self) -> bool {
        self.failure.is_some()
    }

    /// Whether the group built but left one or more of its own rim loops unbevelled.
    pub fn partly_refused(&self) -> bool {
        !self.failed()
            && self
                .loop_failures
                .as_ref()
                .is_some_and(|failures| !failures.is_empty())
    }

    /// Whether this group was applied but with something worth saying about it.
    pub fn warned(&self) -> bool {
        !self.warnings.is_empty()
    }
}

/// One rim loop of a group that could not be built, on a row that built the rest of its loops
/// ([`BevelLoopFailure`]).
///
/// The same two things a refused ROW carries - the refusal and the largest size that would have
/// built it - said about a part of a row instead of all of it,
/// because that is what the user has to act on: a port mouth that did not round is answered by a
/// smaller size or a second operation, and neither is findable from a row that simply built.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelLoopFailure {
    /// Which refusal the loop got, with the values it quotes.
    pub advisory: BevelAdvisory,

    /// The largest size that would have built this loop alone, or `None` where none was found -
    /// including where the search for one was not run at all, since a part with dozens of refused
    /// loops must not spend a bisection on each of them. `Some(0.0)` where nothing fits at all (the
    /// loop's edges are gone from the part a first pass built), which is no size to offer.
    pub largest_size_that_fits: Option<f64>,

    /// How many mesh edges the loop is, which is what makes it a big loop or a small one.
    pub edge_count: i32,
}

/// Second readings of a group failure, where the refusal the geometry produced is true but is not
/// what went wrong. Kept beside [`BevelGroupResult`] because this is the same subject - what a
/// skipped group comes back with - and separate from the builder that composes them because the
/// builder is about tools and booleans.
pub mod bevel_group_failures {
    use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
    use crate::topology::pick_interpretations::{
        self, is_a_junction_at_the_coarsest_rung, COARSE_FLAT_ANGLE_DEGREES,
    };
    use crate::topology::{BevelEdgeGroupResolution, ChainStopReason, FeatureEdgeChain};

    /// Replaces a refusal about the SIZE with one about the SELECTION, where the selection is the
    /// thing that went wrong: a tangent chain that resolved to the single edge it was seeded with
    /// because both of its ends are corners at the tolerance the group was picked at.
    /// `failure_reason` is why it was skipped, or `None` when it was not.
    ///
    /// This is the tolerance ladder's problem (plan section 9) reaching the user as the wrong
    /// sentence. At the default flat tolerance every facet join of a tessellated wall is a feature
    /// edge of its own, so every vertex of a rim has three of them and `FeatureEdgeChain.TryFindNext`
    /// stops - rightly, because there is no single way on. The chain is then one facet long, which
    /// nothing of any size can be swept along without its two end caps meeting in the middle, and
    /// the sweep says so in the only terms it has: the most that fits. Retyping a smaller size does
    /// exactly what it says and bevels one facet of the wall, which is not what the click meant.
    ///
    /// Only for a chain that never grew AT ALL. A long chain that stops at genuine corners is the
    /// normal case, and its refusals really are about size.
    ///
    /// And only where the advice would be true, which the shape of the chain alone does not
    /// settle. A single edge of a CUBE, picked as a tangent chain - which a re-resolve onto a
    /// changed mesh, or an older file, can leave a group holding - stops at corners at both ends in
    /// exactly the same way, and telling its owner to read the part more coarsely would send them
    /// after a chain that does not exist at any tolerance. So the vertices the walk stopped at are
    /// asked whether they would still be corners at the coarser rung (`coarsening_would_open_the_way`);
    /// only a stop that a coarser reading really would dissolve gets the coarser-reading sentence.
    pub fn diagnose(
        resolution: &BevelEdgeGroupResolution,
        failure_reason: Option<BevelAdvisory>,
    ) -> Option<BevelAdvisory> {
        let Some(chain) = resolution.chain.as_ref() else {
            return failure_reason;
        };

        if failure_reason.is_none()
            || chain.count() != 1
            || chain.start().reason != ChainStopReason::Corner
            || chain.end().reason != ChainStopReason::Corner
        {
            return failure_reason;
        }

        // There has to BE a coarser reading to send anybody to. A group already picked at the top
        // of the ladder has nowhere further to go, and "choose a coarser reading" said to somebody
        // already at the coarsest rung the chooser offers is worse than the sentence it replaced.
        if resolution.group.flat_angle_degrees >= COARSE_FLAT_ANGLE_DEGREES
            || !coarsening_would_open_the_way(resolution, chain)
        {
            return failure_reason;
        }

        Some(BevelAdvisory::new(
            BevelAdvisoryCode::ChainNeverGrewPastPick,
            vec![
                BevelAdvisoryArg::Double(resolution.group.flat_angle_degrees),
                BevelAdvisoryArg::Double(COARSE_FLAT_ANGLE_DEGREES),
            ],
        ))
    }

    /// Whether reading the part at the ladder's coarsest rung would let this walk continue: the
    /// edge that was picked still exists there, and neither vertex it stopped at is still a
    /// junction of three.
    ///
    /// This is what tells a tessellated surface from a real corner without needing to know either
    /// is there. A feature edge set built at tolerance t holds the edges whose faces turn more than
    /// t, so an edge survives the coarse rung exactly when its own turn does - a 15 degree facet
    /// join of a 24 sided wall does not, and the 90 degree edges of a cube do. Count the survivors
    /// at each stop vertex: two or fewer and the coarser reading leaves a single way on, so the
    /// chain really would grow; three or more and the stop is a corner at every tolerance the
    /// ladder offers, and the refusal the geometry gave is the honest one after all.
    ///
    /// The picked edge is asked the same question first. A rung it is flat at has no reading of
    /// this click at all - `PickInterpretations` skips such a rung rather than answering with some
    /// other edge's chain - so there would again be nothing to send anyone to. The junction test is
    /// the picker's own ([`is_a_junction_at_the_coarsest_rung`]), so a stop this explains away is
    /// one the picker would not have handed over in the first place.
    fn coarsening_would_open_the_way(
        resolution: &BevelEdgeGroupResolution,
        chain: &FeatureEdgeChain,
    ) -> bool {
        let features = &resolution.features;

        if resolution.seed_edge_index < 0
            || !pick_interpretations::survives_the_coarsest_rung(
                &features.edges()[resolution.seed_edge_index as usize],
            )
        {
            return false;
        }

        !is_a_junction_at_the_coarsest_rung(features, chain.start().vertex)
            && !is_a_junction_at_the_coarsest_rung(features, chain.end().vertex)
    }
}
