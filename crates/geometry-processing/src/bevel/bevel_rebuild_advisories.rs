//! Everything a bevel rebuild has to SAY about itself: the repair the working mesh needed, plus
//! each place the rebuild succeeded while quietly delivering less than the user asked for.
//!
//! The list exists because of one failure mode that is worse than a failed rebuild: a rebuild
//! that reports success and hands back a part which is not the part that was asked for. Every
//! stage of the bevel has a fallback - a trim that could not be swept, a corner that could not be
//! rolled - and every one of those fallbacks was chosen because leaving the old shape standing is
//! safer than refusing the whole operation. Nothing in the finished mesh distinguishes "there was
//! no corner to round here" from "the corner could not be rounded", so each advisory is raised at
//! the stage that observed the fallback rather than inferred from the result.
//!
//! Each advisory is said ONCE for the operation however many chains hit it. A selection of twelve
//! edges that trips the same fallback on eight of them is one thing for the user to go and look
//! at, not eight, and a panel with eight copies of one sentence reads as noise and gets ignored.
//! The messages name what the user will SEE in the part rather than what the code could not do.
//!
//! Lives beside `BevelFeatureMeshBuilder` rather than in it because this is a whole small
//! subject (the wording, the once-per-operation rule and the reasons for both) that the builder
//! only ever calls into.

use super::{BevelAdvisory, BevelAdvisoryCode};

/// The advisories one rebuild has gathered, in the order they were raised.
#[derive(Debug, Clone, Default)]
pub struct BevelRebuildAdvisories {
    messages: Vec<BevelAdvisory>,
    said_bead_went_on_unclipped: bool,
    said_shadow_clip_was_refused: bool,
    said_shadow_body_was_given_up: bool,
    said_corner_went_unrounded: bool,
    said_second_pass_tool_was_not_a_bead: bool,
}

impl BevelRebuildAdvisories {
    /// Starts from whatever repairing the working mesh had to report
    /// (`BevelWorkingMesh.CreateFrom`; `None` for nothing), which travels with the rebuild for the
    /// same reason everything else here does: the repair is made on a copy and never on the design,
    /// so this is the only trace of it the user ever gets.
    pub fn new<I: IntoIterator<Item = BevelAdvisory>>(source_advisories: Option<I>) -> Self {
        Self {
            messages: source_advisories
                .map(|advisories| advisories.into_iter().collect())
                .unwrap_or_default(),
            ..Self::default()
        }
    }

    /// What to publish, live: one list for the whole rebuild, so every way out of the builder
    /// publishes whatever had been added by the time it took its way out.
    pub fn messages(&self) -> &[BevelAdvisory] {
        &self.messages
    }

    /// A bead was unioned on without its trim - either its void wedge would not sweep or the clip
    /// boolean would not run.
    ///
    /// The failure this names is a slab of tool quietly hanging out of the back of a thin wall. A
    /// user who is told the trim was skipped can at least go and look at the far side of their
    /// walls; a user who is told nothing finds it in a print.
    pub fn bead_went_on_unclipped(&mut self) {
        Self::say_once(
            &mut self.messages,
            &mut self.said_bead_went_on_unclipped,
            BevelAdvisoryCode::BeadWentOnUnclipped,
        );
    }

    /// A cutter that had a feature standing on one of its faces could not be clipped to the column
    /// that feature stands on (`BevelShadowClip`), so the cut went ahead unclipped.
    ///
    /// What this names is a rib, boss or nose sawn through where the fillet ran under it - the
    /// answer the pipeline gave before the clip existed, kept as the fallback because refusing the
    /// group instead would lose a fillet every other kernel builds. The damage is somewhere the
    /// user is not looking.
    pub fn shadow_clip_was_refused(&mut self) {
        Self::say_once(
            &mut self.messages,
            &mut self.said_shadow_clip_was_refused,
            BevelAdvisoryCode::ShadowClipWasRefused,
        );
    }

    /// The cutter WAS clipped to the columns the standing feature's footprint casts, but the fuller
    /// region that also carries the feature's own BODY (`BevelShadowBody`) was one the kernel would
    /// not cut with, so the footprint prisms alone were used.
    ///
    /// A different thing from [`Self::shadow_clip_was_refused`]: there the feature is sawn
    /// through, here it is kept and only its SHOULDERS are cut. Not a fault; said because the
    /// difference is small, is in a place the user is not looking, and is the one thing that tells
    /// a support case whether the fuller region ran.
    pub fn shadow_body_was_given_up(&mut self) {
        Self::say_once(
            &mut self.messages,
            &mut self.said_shadow_body_was_given_up,
            BevelAdvisoryCode::ShadowBodyWasGivenUp,
        );
    }

    /// Two fillets met over an unselected crease and the corner between them could not be given
    /// the shape a rolling ball leaves, because one of the two runs could not sweep the tube the
    /// trim subtracts (`BevelPairCornerTrim`).
    ///
    /// The part still builds, but the corner comes out as the union of two cuts - the ridge along
    /// the diagonal that the trim stage exists to remove - and nothing in the result says so.
    pub fn corner_went_unrounded(&mut self) {
        Self::say_once(
            &mut self.messages,
            &mut self.said_corner_went_unrounded,
            BevelAdvisoryCode::CornerWentUnrounded,
        );
    }

    /// The second pass re-read a concave crease on the cut part and the sweep handed back
    /// something that was not a bead, which there is nowhere left to use: the cut has already been
    /// made.
    ///
    /// Nothing known produces this. It is said rather than asserted because the alternative at
    /// this point in a rebuild is to drop a tool in silence, and because the pass reads a mesh a
    /// boolean produced: what the walk finds there is not something this side can enumerate.
    pub fn second_pass_tool_was_not_a_bead(&mut self) {
        Self::say_once(
            &mut self.messages,
            &mut self.said_second_pass_tool_was_not_a_bead,
            BevelAdvisoryCode::SecondPassToolWasNotABead,
        );
    }

    fn say_once(messages: &mut Vec<BevelAdvisory>, said: &mut bool, code: BevelAdvisoryCode) {
        if *said {
            return;
        }

        *said = true;

        messages.push(BevelAdvisory::new(code, vec![]));
    }
}
