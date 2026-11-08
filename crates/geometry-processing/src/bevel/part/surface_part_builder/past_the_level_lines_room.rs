// A pass the level line refused by SIZE,
// where the pick's own room was read off a feature edge beyond its face, asked again of the
// existing construction - which rolls the ball on over that edge.
//
// Here: the pick's own reading the level line leaves behind (`level_line` writes it), the gate
// (`goes_past_the_level_lines_room`) and the two readings of a refusal the swap is judged by
// (`the_level_line_blames_the_whole_pass`, `the_room_named`), and the swap itself
// (`the_existing_construction_past_the_level_lines_room`), which asks one attempt
// (`try_together`) again with the level line set aside.

use std::collections::HashSet;
use std::rc::Rc;

use crate::bevel::level_line::{level_line_routing, RoomReadingKind};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelGroupResult};
use solidmesh_mesh::{CancelToken, Mesh};

use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError};
use crate::bevel::part::self_intersection_check::the_answer_intersects_itself_where_the_source_did_not;

impl BevelSurfacePartBuilder {
    /// The route a pass built past the level line's room writes -
    /// the existing construction's, not a level-line win.
    pub const PAST_THE_FOREIGN_ROOM_ROUTE: &'static str =
        "existing: past the level line's Foreign room";

    /// Clear what the last refusal said about the pick's
    /// reading, before a new pass reads it.
    pub fn forget_the_last_refusals_reading(&mut self) {
        self.the_picks_own_reading = (None, RoomReadingKind::None);
        self.the_refusal_is_a_pinch = false;

        // A pass that never reaches the band (its pick does not link, say) must not act on the
        // last one's.
        self.the_band_refused_a_touch = false;
    }

    /// The memo key of the pick the level line last read (`None`: none read) and which reading
    /// named that pick's OWN room.
    pub fn the_picks_own_reading(&self) -> (Option<&str>, RoomReadingKind) {
        (
            self.the_picks_own_reading.0.as_deref(),
            self.the_picks_own_reading.1,
        )
    }

    /// Whether the level line's last size refusal was a pinch.
    pub fn the_refusal_is_a_pinch(&self) -> bool {
        self.the_refusal_is_a_pinch
    }

    /// Whether a level-line refusal goes on to the existing
    /// construction past its room - one about the size, of a pick whose own room reading was
    /// Foreign, that is not a pinch.
    ///
    /// A pinch is the band meeting itself or another band, which rolling on over an edge does
    /// nothing for, and its refusal names the one loop to drop: on a plate's rim beside a 0.5 mm
    /// block the short block's strip pinches at the room itself and is left alone. A floor a pinch
    /// set counts as that pinch, so a drag answers as the fresh build that pinched did; and a size
    /// the existing construction built lifts such floors (`remember_a_size_that_built`), so a drag
    /// across one is not held by a pinch a fresh build at its size would never see.
    pub fn goes_past_the_level_lines_room(&self, about_the_size: bool) -> bool {
        about_the_size
            && !self.the_refusal_is_a_pinch
            && self.the_picks_own_reading.1 == RoomReadingKind::Foreign
    }

    /// Whether the level line's size refusal is the whole
    /// pass's, and not one loop's: a pinch that names ONE loop of several drops just that loop and
    /// builds the rest, and trading it for a reading that blames everybody would drop loops that
    /// fit (BevelNamedPinchDropsOneLoopTests).
    pub fn the_level_line_blames_the_whole_pass(
        contributing: &[Rc<RimLoop>],
        guilty: &HashSet<i32>,
    ) -> bool {
        let blamed = Self::the_loops_holding(contributing, guilty).len();

        blamed == 0 || blamed == contributing.len()
    }

    /// The size a size refusal says fits, and NaN for any other advisory - NaN
    /// because every comparison with it is false, so a refusal naming no size never replaces one
    /// that does, nor is replaced by one.
    pub fn the_room_named(advisory: Option<&BevelAdvisory>) -> f64 {
        let Some(advisory) = advisory else {
            return f64::NAN;
        };

        if !matches!(
            advisory.code(),
            BevelAdvisoryCode::SizeExceedsFaceWidth
                | BevelAdvisoryCode::SizeCollidesAcrossFace
                | BevelAdvisoryCode::SizeReachesPastCrease
        ) {
            return f64::NAN;
        }

        match advisory.args().get(2) {
            Some(BevelAdvisoryArg::Double(room)) => *room,
            _ => f64::NAN,
        }
    }

    /// The existing construction's answer to a
    /// pass the level line refused by SIZE - its closed part where it builds one that crosses
    /// itself nowhere the source did not, and nothing otherwise; the level line's refusal then
    /// stands as the answer.
    ///
    /// The level line's room is how far ITS band reaches, not how far the part has room for: its
    /// foreign reading stops at the first feature edge the band would cross, while the existing
    /// construction rolls the ball on over that edge. Where neither builds, the refusal that
    /// stands (`guilty`, `reason`) is the one naming MORE room, and a refusal of the existing
    /// construction's that is not about the size never replaces the level line's. Bounded: one
    /// attempt per (pick, size) per rebuild, and none once the size search is out of its time.
    pub(super) fn the_existing_construction_past_the_level_lines_room(
        &mut self,
        contributing: &[Rc<RimLoop>],
        results: &mut [Option<BevelGroupResult>],
        trial_size: f64,
        cancellation_token: &CancelToken,
        guilty: &mut HashSet<i32>,
        reason: &mut Option<BevelAdvisory>,
    ) -> Result<Option<Mesh>, SurfacePartBuildError> {
        // Keyed by the pick and the size's bits, -0 folded onto 0 so the two zeros are one size.
        let size_key = if trial_size == 0.0 {
            0.0f64.to_bits()
        } else {
            trial_size.to_bits()
        };
        let attempt = (self.the_picks_own_reading.0.clone(), size_key);

        if self.the_legacy_attempts_that_failed.contains(&attempt)
            || (self.the_ladder_is_asking
                && self
                    .the_search_clock
                    .is_some_and(|clock| self.out_of_search_time(clock)))
        {
            return Ok(None);
        }

        self.the_level_line_is_set_aside = true;
        let existing =
            self.try_these_groups_together(contributing, results, trial_size, cancellation_token);
        let answer = self.judge_the_existing_construction(
            existing,
            attempt,
            contributing,
            trial_size,
            cancellation_token,
            guilty,
            reason,
        );
        self.the_level_line_is_set_aside = false;

        answer
    }

    /// The second half of [`Self::the_existing_construction_past_the_level_lines_room`], run
    /// while the level line is still set aside.
    #[allow(clippy::too_many_arguments)]
    fn judge_the_existing_construction(
        &mut self,
        existing: Result<super::PassAttempt, SurfacePartBuildError>,
        attempt: (Option<String>, u64),
        contributing: &[Rc<RimLoop>],
        trial_size: f64,
        cancellation_token: &CancelToken,
        guilty: &mut HashSet<i32>,
        reason: &mut Option<BevelAdvisory>,
    ) -> Result<Option<Mesh>, SurfacePartBuildError> {
        let existing = existing?;

        if let Some(built) = existing.mesh {
            // The level line's own last gate, on the same source: a closed part that newly
            // crosses itself is a part with a fold in it, and the level line's refusal is the
            // better answer.
            let (positions, indices) = self.the_kernels_mesh();

            if the_answer_intersects_itself_where_the_source_did_not(
                &built,
                &positions,
                &indices,
                cancellation_token,
            )? {
                level_line_routing::set_last_route(Some(
                    "level-line: refused by size; the existing construction's part intersects itself"
                        .to_string(),
                ));
                self.the_legacy_attempts_that_failed.insert(attempt);

                return Ok(None);
            }

            level_line_routing::set_last_route(Some(Self::PAST_THE_FOREIGN_ROOM_ROUTE.to_string()));
            let stamp = self.the_stamp();

            // A Foreign reading is only ever written with the pick's key, so the key is there;
            // a missing one is skipped rather than failing the build.
            if let Some(key) = self.the_picks_own_reading.0.clone() {
                self.memo
                    .remember_a_size_that_built(stamp, &key, trial_size);
            }

            return Ok(Some(built));
        }

        self.the_legacy_attempts_that_failed.insert(attempt);

        if existing.about_the_size
            && Self::the_room_named(existing.reason.as_ref())
                > Self::the_room_named(reason.as_ref())
            && Self::the_level_line_blames_the_whole_pass(contributing, guilty)
        {
            *guilty = existing.guilty;
            *reason = existing.reason;
        }

        Ok(None)
    }
}
