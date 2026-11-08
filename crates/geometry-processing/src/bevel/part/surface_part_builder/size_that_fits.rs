// The SIZE half of the surface part builder - what a
// refused pick would have taken, found by building it at smaller and smaller sizes until one of
// them works. Its own file because it is the one thing in the builder that is
// not about making the part: it runs only on a rebuild that has already refused, and only so that
// the sentence the user reads ends in a number they can act on.
//
// Here: the row's number (`already_said`, `the_refusal_offering`; the caller words the
// sentence),
// the offered size (`the_size_to_offer`), the search's bounds (halvings, the per-search and
// per-rebuild clocks, the level line's floor) and `out_of_search_time`, and the search's BUILDS -
// `the_largest_that_builds`, `the_settled_size_confirmed`, `the_whole_pick_confirms`,
// `whole_pick_builds` and `builds` - each of which asks one attempt (`try_together`) at a smaller
// size. `Say` (`build`) is what runs them.

use std::rc::Rc;

use crate::bevel::part::BevelSizeSearchRung;
use crate::bevel::BevelGroupResult;
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg};
use polygon_mesh::CancelToken;
use web_time::Instant;

use super::super::shown_size::the_shown_size;
use super::build::check;
use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError};

impl BevelSurfacePartBuilder {
    /// How many halvings a refused group is allowed, which is what bounds
    /// the cost of asking.
    ///
    /// Eight of them put the answer inside 1/256 of the size that was asked for - 0.0016 mm on a
    /// 0.4 mm fillet - which is finer than the three decimals the number is shown to. Each step is
    /// one plan, one room measure and one build of that group ALONE, measured at about 200 ms on
    /// the 48 edge ring chain of "Chain and Face Should Both work.mcx", so the whole search costs
    /// about 1.7 seconds there and is paid only by a rebuild that was going to refuse anyway.
    pub const HALVINGS_ALLOWED: i32 = 8;

    /// What [`Self::size_search_budget_ms`] is unless a test has
    /// taken the clock out.
    ///
    /// Lars's production target is about 20 minutes. Held here because raising it took Many Holes
    /// from 22 s to 95 s, mostly on "up to 0.498 mm" hints for loops that fit alone; raise it once
    /// those build.
    pub const DEFAULT_SIZE_SEARCH_BUDGET_MS: i32 = 2000;

    /// The smallest fraction of the asked size a LEVEL-LINE rung
    /// will try. Below it the honest answer is that the pick does not fit at any size worth
    /// offering, and no number is offered at all.
    ///
    /// A floor is needed because a rung's COST rises as the size falls: the band's refinement
    /// splits the mesh finer the thinner the band is (on "Many Holes.mcx" a ladder handed a quoted
    /// room of 0.0022 halved to 0.0011 and stayed inside BandRefinement past ten minutes). A COUNT,
    /// not a clock, so how many rungs run is the same on a loaded machine as on an idle one.
    pub const SMALLEST_FRACTION_WORTH_TRYING: f64 = 0.01;

    /// How long the whole search for a size that fits may take
    /// before it gives up with whatever it has.
    ///
    /// The halving count bounds the PRECISION; this bounds the COST, which depends entirely on how
    /// many edges the pick has. Checked BETWEEN steps rather than inside one, so the answer is
    /// always a size that really was built - stopping early gives a coarser number or none, never
    /// a wrong one.
    ///
    /// This builder's own, handed in by the operation it builds for
    /// (`BevelFeatureObject3D::size_search_budget_ms`): settable only so that a test which reads
    /// the pass call by call can take the clock out of it (which rungs run inside the budget
    /// decides which constructions a rung reaches), and per builder so that a test's setting
    /// changes only its own operation's searches, never those of a bevel building beside it.
    /// Nothing in the product sets it.
    pub fn size_search_budget_ms(&self) -> i32 {
        self.size_search_budget_ms
    }

    /// Sets [`Self::size_search_budget_ms`] for this builder.
    pub fn set_size_search_budget_ms(&mut self, milliseconds: i32) {
        self.size_search_budget_ms = milliseconds;
    }

    /// How long ALL the size searches of one rebuild may take
    /// between them - four searches' worth, so that the handful of refusals a user can read still
    /// carry their number and a part that refuses in bulk (a rim dropped one loop at a time can
    /// refuse forty-three times) stops paying for them.
    pub fn size_search_total_budget_ms(&self) -> i32 {
        // Saturating: a test that takes the clock out may hand in a budget near `i32::MAX`.
        self.size_search_budget_ms().saturating_mul(4)
    }

    /// Whether the refusal's own sentence already ends with this number, to the
    /// three decimals both are shown to - in which case saying it again would be the same size
    /// twice on one row. `quoted` is `TryTheMostThatFits`'s answer.
    pub fn already_said(quoted: Option<f64>, largest: Option<f64>) -> bool {
        match (quoted, largest) {
            (Some(most_that_fits), Some(largest)) => (most_that_fits - largest).abs() < 0.0005,
            _ => false,
        }
    }

    /// The refusal with the size the row offers as its own "most that
    /// fits", so a row says one number: the ladder's answer (or the settling's, where no ladder
    /// ran) where there is one, and otherwise NO number.
    ///
    /// A level-line refusal quotes the room itself, and where the room does not quite build the
    /// ladder lands just under it; left as it was, the row read "the most that fits there is about
    /// 0.5. Up to 0.494 mm fits here" - two sizes on one line. Where the ladder built nothing the
    /// room is taken OUT of the advisory - a missing third
    /// argument - and the sentence is the one that names no size. A number on a row is a
    /// promise that typing it builds. `quoted` is `TryTheMostThatFits`'s answer; `level_line` is
    /// the builder's routing ([`BevelRouting::level_line`](crate::bevel::part::BevelRouting)).
    pub fn the_refusal_offering(
        level_line: bool,
        refused: &BevelAdvisory,
        quoted: Option<f64>,
        largest: Option<f64>,
    ) -> BevelAdvisory {
        // Gated on the level line being routed, which is what the other constructions'
        // owner-file tests turn off to read the sentences they pin.
        if !level_line || quoted.is_none() {
            return refused.clone();
        }

        // Written in even where it is the quoted room to three decimals (`already_said`), so the
        // advisory carries exactly the size the row offers - the floored one that was built.
        let mut args: Vec<BevelAdvisoryArg> = refused.args().to_vec();

        if args.len() > 2 {
            match largest {
                Some(offering) => args[2] = BevelAdvisoryArg::Double(offering),
                // No size at index 2. The size advisories this is written for carry
                // exactly three arguments, so nothing after it is lost.
                None => args.truncate(2),
            }
        }

        let rewritten = BevelAdvisory::new(refused.code(), args);

        if refused.mesh_edge_index() == BevelAdvisory::NO_MESH_EDGE {
            rewritten
        } else {
            rewritten.on_mesh_edge(refused.mesh_edge_index())
        }
    }

    /// The number to put in front of the user for a room the settling read -
    /// the reading taken DOWN, or nothing where there is no room to offer.
    ///
    /// A reading is not a build. The two disagree by known but small amounts - a room of 1 where
    /// the construction refuses at 1 and carries 0.996 - so the number is taken a whole percent
    /// down, then rounded DOWN to three significant digits, which is what the sentence shows.
    /// Both moves are one way only: the advisory says "about", and the error it is allowed is the
    /// one that still fits.
    pub fn the_size_to_offer(room: f64) -> Option<f64> {
        // A room that is not positive is refused; NaN is refused too, which `is_finite` also covers.
        if room.is_nan() || room <= 0.0 || !room.is_finite() {
            return None;
        }

        let under = room * 0.99;
        // Floored to an int: a finite positive room's log is far inside an int.
        let places = under.log10().floor() as i32;
        let step = 10f64.powf((places - 2) as f64);
        let offered = (under / step).floor() * step;

        if offered > 0.0 {
            Some(offered)
        } else {
            None
        }
    }

    /// Whether this search has spent its own clock, or the rebuild's whole
    /// allowance for searches - checked before EVERY build, the up-front ones included, so neither
    /// budget can be run past by a search. `spent` is when the search started.
    pub fn out_of_search_time(&self, spent: Instant) -> bool {
        let elapsed = spent.elapsed().as_millis() as i64;

        elapsed >= self.size_search_budget_ms() as i64
            || self.size_search_ms_spent + elapsed >= self.size_search_total_budget_ms() as i64
    }

    /// How long the size searches of this rebuild have taken between them, in milliseconds.
    pub fn size_search_ms_spent(&self) -> i64 {
        self.size_search_ms_spent
    }

    /// Adds a finished search's time to [`Self::size_search_ms_spent`], which every search does on
    /// its way out.
    pub fn spend_search_time(&mut self, spent: Instant) {
        self.size_search_ms_spent += spent.elapsed().as_millis() as i64;
    }

    /// The largest size that builds these loops on their own, or
    /// `None` where none of the sizes tried does.
    ///
    /// Bisection and nothing cleverer, because what is being searched is not monotone in any way
    /// that can be proved, and every step is the real question asked of the real builder. The
    /// interval is (0, the size asked for]. This pick ALONE - every loop of a group where the whole
    /// row is refused, and the ONE loop where the rest of the row built - confirmed against `pick`,
    /// every loop still in the pass, before it is offered (`the_whole_pick_confirms`). `quoted` is
    /// what a refusal about the SIZE already worked out, tried first and otherwise the ceiling.
    pub(super) fn the_largest_that_builds(
        &mut self,
        alone: &[Rc<RimLoop>],
        pick: &[Rc<RimLoop>],
        quoted: Option<f64>,
        cancellation_token: &CancelToken,
    ) -> Result<Option<f64>, SurfacePartBuildError> {
        if alone.is_empty() || !self.offered[alone[0].group_index as usize] {
            // Never offered means it did not resolve to edges at all, and there is no size at
            // which a pick with nothing in it builds.
            return Ok(None);
        }

        if self.offers_no_size && self.routing.level_line {
            return Ok(None);
        }

        if self.size_search_ms_spent >= self.size_search_total_budget_ms() as i64 {
            // A rebuild has only so much of this to give: past the budget a refusal says what
            // happened and offers nothing, which is what a refusal that finds no size does.
            return Ok(None);
        }

        let mut scratch: Vec<Option<BevelGroupResult>> = vec![None; self.resolutions.len()];
        self.rungs_tried = 0;

        let mut low = 0.0;
        let mut high = self.size;
        let mut largest: Option<f64> = None;
        let mut steps_left = Self::HALVINGS_ALLOWED;
        let spent = Instant::now();
        self.the_search_clock = Some(spent);

        // LEVEL LINE ONLY. The legacy and sections constructions are FROZEN: with the level line
        // out of the routing they answer exactly what they answered before the level line existed.
        // A floor of zero is the ladder they run.
        let floor = if self.routing.level_line {
            self.size * Self::SMALLEST_FRACTION_WORTH_TRYING
        } else {
            0.0
        };

        if let Some(quoted) = quoted.filter(|&quoted| quoted > 0.0 && quoted < self.size) {
            // The quoted room first, since where it builds that one build is the whole search; a
            // level-line room can itself fail to build, so there the room taken a percent down
            // is tried next, before the halving.
            steps_left -= 1;

            if self.builds(alone, &mut scratch, quoted, cancellation_token)? {
                return self.the_whole_pick_confirms(
                    alone,
                    pick,
                    quoted,
                    &mut scratch,
                    spent,
                    floor,
                    cancellation_token,
                    true,
                );
            }

            high = quoted;

            if self.routing.level_line {
                if let Some(offered) = Self::the_size_to_offer(quoted)
                    .filter(|&offered| offered < quoted && !self.out_of_search_time(spent))
                {
                    steps_left -= 1;

                    if self.builds(alone, &mut scratch, offered, cancellation_token)? {
                        return self.the_whole_pick_confirms(
                            alone,
                            pick,
                            offered,
                            &mut scratch,
                            spent,
                            floor,
                            cancellation_token,
                            true,
                        );
                    }

                    high = offered;
                }
            }
        }

        for _ in 0..steps_left {
            if self.out_of_search_time(spent) {
                break;
            }

            let trial = (low + high) / 2.0;

            if trial < floor {
                // Below the floor there is no answer worth offering, and the rungs below it are
                // the expensive ones.
                break;
            }

            if self.builds(alone, &mut scratch, trial, cancellation_token)? {
                largest = Some(trial);
                low = trial;
            } else {
                high = trial;
            }
        }

        let Some(largest) = largest else {
            self.spend_search_time(spent);

            return Ok(None);
        };

        self.the_whole_pick_confirms(
            alone,
            pick,
            largest,
            &mut scratch,
            spent,
            floor,
            cancellation_token,
            true,
        )
    }

    /// The size a settling READ for these loops, confirmed by
    /// building the whole pick at it and floored to what the row shows - or `None` where it is
    /// not confirmed inside the search budget. The reading is the first guess, and usually the one
    /// build it takes.
    pub(super) fn the_settled_size_confirmed(
        &mut self,
        alone: &[Rc<RimLoop>],
        pick: &[Rc<RimLoop>],
        settled: Option<f64>,
        cancellation_token: &CancelToken,
    ) -> Result<Option<f64>, SurfacePartBuildError> {
        if !self.routing.level_line {
            return Ok(settled);
        }

        // A settled size that is not positive is refused; NaN is refused too.
        let Some(settled) = settled.filter(|&settled| settled > 0.0) else {
            return Ok(None);
        };

        if self.offers_no_size
            || alone.is_empty()
            || !self.offered[alone[0].group_index as usize]
            || self.size_search_ms_spent >= self.size_search_total_budget_ms() as i64
        {
            return Ok(None);
        }

        let mut scratch: Vec<Option<BevelGroupResult>> = vec![None; self.resolutions.len()];
        self.rungs_tried = 0;
        let spent = Instant::now();
        self.the_search_clock = Some(spent);
        let floor = self.size * Self::SMALLEST_FRACTION_WORTH_TRYING;

        self.the_whole_pick_confirms(
            alone,
            pick,
            settled,
            &mut scratch,
            spent,
            floor,
            cancellation_token,
            false,
        )
    }

    /// The size a search found for its loops ALONE, confirmed by
    /// building the whole pick at it - or, where the whole pick refuses there, the largest size
    /// under it the whole pick builds at, or `None` where none is found inside the budget.
    ///
    /// A number in a refusal is a promise that typing it in builds, and a size that builds one
    /// group alone can still be refused beside the rest of the pick. Every size built here is
    /// first floored to what the row shows (`the_shown_size`), so the number the user reads is the
    /// number that was built. `candidate_was_built` is false for a size nobody has built yet (a
    /// settling's reading), so it is built even where the loops are the whole pick. The search's
    /// time is added to the rebuild's on the way out, whatever the answer.
    #[allow(clippy::too_many_arguments)]
    fn the_whole_pick_confirms(
        &mut self,
        alone: &[Rc<RimLoop>],
        pick: &[Rc<RimLoop>],
        candidate: f64,
        scratch: &mut [Option<BevelGroupResult>],
        spent: Instant,
        floor: f64,
        cancellation_token: &CancelToken,
        candidate_was_built: bool,
    ) -> Result<Option<f64>, SurfacePartBuildError> {
        let answer = self.the_whole_pick_confirms_unspent(
            alone,
            pick,
            candidate,
            scratch,
            spent,
            floor,
            cancellation_token,
            candidate_was_built,
        );
        self.spend_search_time(spent);

        answer
    }

    /// The body of [`Self::the_whole_pick_confirms`], before its time is spent.
    #[allow(clippy::too_many_arguments)]
    fn the_whole_pick_confirms_unspent(
        &mut self,
        alone: &[Rc<RimLoop>],
        pick: &[Rc<RimLoop>],
        candidate: f64,
        scratch: &mut [Option<BevelGroupResult>],
        spent: Instant,
        floor: f64,
        cancellation_token: &CancelToken,
        candidate_was_built: bool,
    ) -> Result<Option<f64>, SurfacePartBuildError> {
        if !self.routing.level_line {
            return Ok(Some(candidate));
        }

        // The union of the pick and these loops: the pick's loops once each, then any of these not in it.
        let mut whole: Vec<Rc<RimLoop>> = Vec::with_capacity(pick.len() + alone.len());

        for one in pick.iter().chain(alone) {
            if !whole.contains(one) {
                whole.push(one.clone());
            }
        }

        let shown = the_shown_size(candidate);

        if candidate_was_built && whole.len() == alone.len() && shown == candidate {
            // Nothing else in the pass, so the loops alone ARE the whole pick, and this size was
            // built.
            return Ok(Some(candidate));
        }

        if shown > 0.0
            && !self.out_of_search_time(spent)
            && self.whole_pick_builds(alone, &whole, scratch, shown, cancellation_token)?
        {
            return Ok(Some(shown));
        }

        let mut low = 0.0;
        let mut high = shown;
        let mut largest = None;

        if let Some(just_under) = Self::the_size_to_offer(candidate)
            .map(the_shown_size)
            .filter(|&just_under| just_under > 0.0 && just_under < shown)
        {
            if !self.out_of_search_time(spent) {
                if self.whole_pick_builds(alone, &whole, scratch, just_under, cancellation_token)? {
                    return Ok(Some(just_under));
                }

                high = just_under;
            }
        }

        for _ in 0..Self::HALVINGS_ALLOWED {
            let trial = the_shown_size((low + high) / 2.0);

            if self.out_of_search_time(spent) || trial < floor || trial <= low {
                break;
            }

            if self.whole_pick_builds(alone, &whole, scratch, trial, cancellation_token)? {
                largest = Some(trial);
                low = trial;
            } else {
                high = trial;
            }
        }

        // `None` where nothing was seen to build with the rest of the pick: no number rather than
        // one that fails.
        Ok(largest)
    }

    /// Whether these loops build at this size beside the rest of the pick -
    /// asked the way the pass itself would answer it: the whole pick is built, and a loop the
    /// refusal names that is NOT one of these and refuses at this size on its own too is dropped
    /// and the rest built again, as the pass would drop it.
    ///
    /// Side-effect free for the pass that asked: the settling is kept off while it asks, and the
    /// touch and settling flags are put back as they were, whatever the answer.
    fn whole_pick_builds(
        &mut self,
        alone: &[Rc<RimLoop>],
        whole: &[Rc<RimLoop>],
        scratch: &mut [Option<BevelGroupResult>],
        trial: f64,
        cancellation_token: &CancelToken,
    ) -> Result<bool, SurfacePartBuildError> {
        let settling_was_off = self.the_settling_is_off;
        let touch_was_refused = self.the_band_refused_a_touch;

        self.the_settling_is_off = true;
        let answer = self.whole_pick_builds_with_the_settling_off(
            alone,
            whole,
            scratch,
            trial,
            cancellation_token,
        );
        self.the_settling_is_off = settling_was_off;
        self.the_band_refused_a_touch = touch_was_refused;

        answer
    }

    /// The body of [`Self::whole_pick_builds`], run with the settling off.
    fn whole_pick_builds_with_the_settling_off(
        &mut self,
        alone: &[Rc<RimLoop>],
        whole: &[Rc<RimLoop>],
        scratch: &mut [Option<BevelGroupResult>],
        trial: f64,
        cancellation_token: &CancelToken,
    ) -> Result<bool, SurfacePartBuildError> {
        let mut remaining = whole.to_vec();

        while remaining.len() > alone.len() {
            check(cancellation_token)?;
            self.rungs_tried += 1;
            let rung = BevelSizeSearchRung {
                loops: remaining.len() as i32,
                attempt: self.rungs_tried,
                attempts: Self::HALVINGS_ALLOWED,
            };
            self.bar
                .open_a_rung(Self::RUNG_SHARE_OF_WHAT_IS_LEFT, Some(rung));

            self.the_ladder_is_asking = true;
            let attempt =
                self.try_these_groups_together(&remaining, scratch, trial, cancellation_token);
            self.the_ladder_is_asking = false;
            let attempt = attempt?;

            if attempt.mesh.is_some() {
                return Ok(true);
            }

            let mut named = attempt.guilty;

            for chain in attempt.condemned.iter().flatten() {
                named.extend(chain.mesh_edges.iter().copied());
            }

            if attempt.reason.is_none() && named.is_empty() {
                return Ok(false);
            }

            let blamed = Self::the_loops_holding(&remaining, &named);

            // Nobody in particular, everybody, or these loops themselves: refused.
            if blamed.is_empty()
                || blamed.len() == remaining.len()
                || blamed.iter().any(|one| alone.contains(one))
                || self
                    .the_search_clock
                    .is_some_and(|clock| self.out_of_search_time(clock))
                || self.builds(&blamed, scratch, trial, cancellation_token)?
            {
                return Ok(false);
            }

            remaining.retain(|one| !blamed.contains(one));
        }

        self.builds(alone, scratch, trial, cancellation_token)
    }

    /// Whether these loops build at this one size, which is the question being
    /// bisected. A ladder's pick is ONE loop, under the many-loop floor, so the settling never runs
    /// here.
    fn builds(
        &mut self,
        alone: &[Rc<RimLoop>],
        scratch: &mut [Option<BevelGroupResult>],
        trial: f64,
        cancellation_token: &CancelToken,
    ) -> Result<bool, SurfacePartBuildError> {
        check(cancellation_token)?;
        self.rungs_tried += 1;
        let rung = BevelSizeSearchRung {
            loops: alone.len() as i32,
            attempt: self.rungs_tried,
            attempts: Self::HALVINGS_ALLOWED,
        };
        self.bar
            .open_a_rung(Self::RUNG_SHARE_OF_WHAT_IS_LEFT, Some(rung));

        self.the_ladder_is_asking = true;
        let attempt = self.try_these_groups_together(alone, scratch, trial, cancellation_token);
        self.the_ladder_is_asking = false;

        Ok(attempt?.mesh.is_some())
    }
}
