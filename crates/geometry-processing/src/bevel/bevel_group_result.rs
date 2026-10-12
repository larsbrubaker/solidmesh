//! What one group did on the rebuild that has just run ([`BevelGroupResult`], with the rim loops
//! it refused while building the rest, [`BevelLoopFailure`]); the second readings of a group
//! failure where the sentence the geometry produced is not what went wrong
//! ([`bevel_group_failures`]); and everything one rebuild had to report, as one immutable thing
//! ([`BevelRebuildResults`]). Ports `Bevel/BevelGroupResult.cs`.
//!
//! A row carries the group's identity rather than a pre-formatted "Group 3 was skipped: ..."
//! string, because the property panel already knows which row it is drawing and needs the reason
//! on its own to put beside that row. Never serialized: it describes one particular rebuild
//! against one particular mesh, and a file that has just been loaded has not run that rebuild
//! yet: a stored "this group failed" would be a claim about geometry nobody has looked at.

use super::bevel_advisory_codes;
use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::{BevelCornerGroup, BevelEdgeGroup};

/// The group a row is about, whichever leg of the operation resolved it - C#'s abstract
/// `BevelGroup` reference, which is one of these two concrete kinds (the Rust [`BevelGroup`] is a
/// trait and cannot be held by value). A copy of the group as it was resolved: C# holds the
/// reference so a caller can find the row again after the list is reordered, and the copy answers
/// that by value (divergence 651).
///
/// [`BevelGroup`]: crate::topology::BevelGroup
#[derive(Debug, Clone, PartialEq)]
pub enum BevelResultGroup {
    /// A mesh leg group (`BevelFeatureObject3D.Groups`).
    Edge(BevelEdgeGroup),
    /// A 2D leg group (`BevelFeatureObject3D.CornerGroups`).
    Corner(BevelCornerGroup),
}

/// C# `BevelGroupResult` (a sealed record). The four positional members and `warning` are the
/// constructor's ([`BevelGroupResult::new`]); the rest are C#'s init-only properties, left at
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

    /// Why it was skipped, already localized, or `None` when it was applied.
    pub failure_reason: Option<String>,

    /// What the group did that the user should know about but which did not stop it being
    /// applied: an open run that had to run past a corner because no face there could trim it (see
    /// `BevelChainEnds`). Already localized; `None` when there was nothing to say.
    pub warning: Option<String>,

    /// The refusal `failure_reason` was written from - WHICH refusal this row carries, rather than
    /// the sentence it came out as. `None` where the row did not fail, and on the few refusals that
    /// were never a coded advisory to begin with.
    ///
    /// Beside the sentence and not instead of it. The sentence is what the panel draws and is
    /// localized where it is composed; the code is what a caller ASKS with - the panel deciding a
    /// row is refused for its size, a test naming the refusal it means - and neither can be got
    /// from the other without matching translated prose, which is how a pin came to be written
    /// against a phrase.
    pub failure_advisory: Option<BevelAdvisory>,

    /// The largest size that WOULD have built this group on its own, where a refused row has one -
    /// `None` on a row that built, and where no size builds it at all.
    ///
    /// The one thing a user can do about a refusal they did not ask for, so it travels beside the
    /// refusal rather than only inside the sentence: a size refusal found the number on its way to
    /// saying no ([`bevel_advisory_codes::try_the_most_that_fits`]), and a BUILD refusal is
    /// bisected for it afterwards. On the row it is said once - appended to the sentence where the
    /// sentence does not already quote it.
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
    /// C#'s positional constructor; `warning` is optional there so that the many callers with
    /// nothing to warn about read as the four things they are about.
    pub fn new(
        group_index: i32,
        group: BevelResultGroup,
        edge_count: i32,
        failure_reason: Option<String>,
        warning: Option<String>,
    ) -> Self {
        BevelGroupResult {
            group_index,
            group,
            edge_count,
            failure_reason,
            warning,
            failure_advisory: None,
            largest_size_that_fits: None,
            loop_failures: None,
            loop_count: 0,
        }
    }

    /// Whether this group was skipped rather than cut.
    pub fn failed(&self) -> bool {
        self.failure_reason.is_some()
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
        self.warning.is_some()
    }
}

/// One rim loop of a group that could not be built, on a row that built the rest of its loops
/// (C# `BevelLoopFailure`, a sealed record).
///
/// The same three things a refused ROW carries - the refusal, the sentence it was written as, and
/// the largest size that would have built it - said about a part of a row instead of all of it,
/// because that is what the user has to act on: a port mouth that did not round is answered by a
/// smaller size or a second operation, and neither is findable from a row that simply built.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelLoopFailure {
    /// Which refusal the loop got, for a caller that asks by code rather than by prose. `None` on
    /// a loop read back from a saved file (`BevelStoredResults` keeps the sentence, not the code).
    pub advisory: Option<BevelAdvisory>,

    /// The sentence it was written as, already localized.
    pub failure_reason: String,

    /// The largest size that would have built this loop alone, or `None` where none was found -
    /// including where the search for one was not run at all, since a part with dozens of refused
    /// loops must not spend a bisection on each of them.
    pub largest_size_that_fits: Option<f64>,

    /// How many mesh edges the loop is, which is what makes it a big loop or a small one.
    pub edge_count: i32,
}

/// Second readings of a group failure, where the sentence the geometry produced is true but is
/// not what went wrong (C# `internal static class BevelGroupFailures`; a module here). Kept beside
/// [`BevelGroupResult`] because this is the same subject - the words a skipped group comes back
/// with - and separate from the builder that composes them because the builder is about tools and
/// booleans.
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

/// Everything one rebuild had to report, as one immutable thing: what each group did, which of
/// those failed, and what went wrong with the cut as a whole (C# `BevelRebuildResults`).
///
/// One object rather than three lists on the operation, for the reason `BevelEdgePickModel.PickData`
/// is one object: it is composed on the rebuild's worker and read by the property panel, and three
/// separate writes have no ordering anybody can rely on. Published as one value, a reader sees the
/// whole of one rebuild or the whole of the one before it, never a mix. The lists are copied in, so
/// the builder may keep composing into its own lists afterwards.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BevelRebuildResults {
    group_results: Vec<BevelGroupResult>,
    group_failures: Vec<BevelGroupResult>,
    operation_failures: Vec<String>,
    operation_advisories: Vec<String>,
    construction_advisories: Vec<String>,
}

impl BevelRebuildResults {
    /// C#'s `Empty`: nothing reported.
    pub fn empty() -> Self {
        Self::default()
    }

    /// `whole_operation_advisory_codes` is what each of `whole_operation_advisories` was, code for
    /// code, so that [`Self::construction_advisories`] can tell the ones about the SOURCE from the
    /// ones about the bevel. The sentences are localized in the application and the codes are the
    /// kernel's, so the two arrive as two lists built in one place from one list of advisories;
    /// where they do not line up - a caller that knows only the sentences, which is every test that
    /// writes results by hand - nothing is filtered rather than the wrong thing being filtered.
    pub fn new(
        results: Option<&[BevelGroupResult]>,
        whole_operation_failures: Option<&[String]>,
        whole_operation_advisories: Option<&[String]>,
        whole_operation_advisory_codes: Option<&[BevelAdvisoryCode]>,
    ) -> Self {
        let group_results = results.map(<[_]>::to_vec).unwrap_or_default();
        let group_failures = group_results
            .iter()
            .filter(|result| result.failed())
            .cloned()
            .collect();
        let operation_failures = whole_operation_failures
            .map(<[_]>::to_vec)
            .unwrap_or_default();
        let operation_advisories: Vec<String> = whole_operation_advisories
            .map(<[_]>::to_vec)
            .unwrap_or_default();

        let construction_advisories = match whole_operation_advisory_codes {
            Some(codes) if codes.len() == operation_advisories.len() => operation_advisories
                .iter()
                .zip(codes)
                .filter(|(_, &code)| !bevel_advisory_codes::is_a_note_rather_than_a_complaint(code))
                .map(|(sentence, _)| sentence.clone())
                .collect(),
            _ => operation_advisories.clone(),
        };

        BevelRebuildResults {
            group_results,
            group_failures,
            operation_failures,
            operation_advisories,
            construction_advisories,
        }
    }

    /// What each group did, in the order the groups were resolved.
    pub fn group_results(&self) -> &[BevelGroupResult] {
        &self.group_results
    }

    /// The rows of [`Self::group_results`] that failed.
    pub fn group_failures(&self) -> &[BevelGroupResult] {
        &self.group_failures
    }

    /// What went wrong with the cut as a whole.
    pub fn operation_failures(&self) -> &[String] {
        &self.operation_failures
    }

    /// What the operation had to put right before it could work, which did not stop it working -
    /// the repairs `BevelWorkingMesh` makes to its copy of the source. Kept apart from
    /// [`Self::operation_failures`] because nothing failed: the part built, and the user is being
    /// told what their source needed rather than what went wrong.
    pub fn operation_advisories(&self) -> &[String] {
        &self.operation_advisories
    }

    /// The advisories that are about the BEVEL - [`Self::operation_advisories`] less the ones that
    /// only say what the working copy had to condition about the source before anything was built
    /// ([`bevel_advisory_codes::is_source_conditioning`]).
    ///
    /// The list to read when the question is "did this build?". A source that needs conditioning
    /// says so on every rebuild, for ever, and it is not something the construction did - so
    /// counting it as a complaint would mark every case on such a part as failing while the part
    /// builds perfectly.
    pub fn construction_advisories(&self) -> &[String] {
        &self.construction_advisories
    }
}
