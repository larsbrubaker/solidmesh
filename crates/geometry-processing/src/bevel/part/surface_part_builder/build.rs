// The surface part builder's build and the passes it runs: what one part comes to in the surface
// construction - every buildable group's rim loops planned
// on the ORIGINAL working mesh and built in one pass, the loop a refusal names dropped and the rest
// tried again (or, where it names none, the smallest set that still refuses: `narrow_the_refusal`),
// and one refusal written against the loops it belongs to (`say`). One attempt is
// `try_together`; a pass the sections split is built in two by `pass_in_two`; the sizes a refused
// row offers are found by `size_that_fits`. The part's prologue (`prologue`) calls
// [`BevelSurfacePartBuilder::build`].
//
// The build reads STOPPED ends: a round on one piece of a faceted curve stops at that piece's
// ends, cut flat with its whole profile, and the unpicked neighbours stay sharp. The kernel keeps
// that behind a per-flow switch (`turn_stopped_ends_on`), off by default, because its tests still
// read a pick both with and without stopped ends. `build` opens the switch's scope for its whole
// run, so every budget made under it - the room readings, the pick-wide build, the chains in
// turn, the size search - and every memo key (`level_line_memo`, which appends `|stopped`) see the
// same setting. The scope is synchronous (nothing in `build` waits), so it never spans another
// task's work on the executor. The builder's routing (`BevelRouting`) is read where each choice is
// made, and its two corner cap switches are held for a pass the same way
// (`build_the_groups_together`, `hold_corner_cap_switches`).

use std::rc::Rc;

use super::no_group_without_a_row::refuse_the_groups_with_no_row;
use crate::bevel::bevel_advisory_codes;
use crate::bevel::level_line::corner_cap_hole_fill_switch::hold_corner_cap_switches;
use crate::bevel::level_line::{turn_stopped_ends_on, WorkError};
use crate::bevel::part::{BevelProgress, BevelProgressPhase};
use crate::bevel::{
    bevel_group_failures, BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelLoopFailure,
    BevelResultGroup,
};
use indexmap::IndexMap;
use solidmesh_math::Color;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::{take_uniform_face_color, CancelToken, Mesh};

use super::super::outcome::BevelPartOutcome;
use super::level_line::LevelLineRouteError;
use super::rim_after_its_partner::TheRimAfterItsPartner;
use super::{BevelSurfacePartBuilder, RimLoop};

/// Why a surface build could not run at all: the rebuild's
/// cancel, a kernel argument check, an exhausted work budget or a section point that is not a
/// number. A refusal is not an error: it is a row with a reason.
#[derive(Debug)]
pub enum SurfacePartBuildError {
    /// The rebuild was cancelled.
    Cancelled,
    /// The level line could not run (a kernel argument check).
    LevelLine(LevelLineRouteError),
    /// A room reading ran out of its work budget.
    BudgetExceeded,
    /// A section point that is not a number (the kernel's `NanSectionPoint` carries nothing
    /// more).
    NotANumber,
}

impl From<LevelLineRouteError> for SurfacePartBuildError {
    fn from(error: LevelLineRouteError) -> Self {
        match error {
            LevelLineRouteError::Cancelled => SurfacePartBuildError::Cancelled,
            other => SurfacePartBuildError::LevelLine(other),
        }
    }
}

impl From<WorkError> for SurfacePartBuildError {
    fn from(error: WorkError) -> Self {
        match error {
            WorkError::Cancelled => SurfacePartBuildError::Cancelled,
            WorkError::BudgetExceeded(_) => SurfacePartBuildError::BudgetExceeded,
        }
    }
}

impl From<solidmesh_mesh::Cancelled> for SurfacePartBuildError {
    fn from(_: solidmesh_mesh::Cancelled) -> Self {
        SurfacePartBuildError::Cancelled
    }
}

/// `Err(Cancelled)` once the rebuild has been cancelled.
pub(super) fn check(cancellation_token: &CancelToken) -> Result<(), SurfacePartBuildError> {
    if cancellation_token.is_cancelled() {
        Err(SurfacePartBuildError::Cancelled)
    } else {
        Ok(())
    }
}

impl BevelSurfacePartBuilder {
    /// What this part comes to in the surface construction - the mesh its
    /// buildable chains rewrote, and a row per group saying what became of it. Nothing in it
    /// reaches the boolean kernel. Synchronous: nothing in it waits, and the cancellation token
    /// is polled between the passes and inside the long-running walkback and stitch.
    pub fn build(
        &mut self,
        cancellation_token: &CancelToken,
    ) -> Result<BevelPartOutcome, SurfacePartBuildError> {
        check(cancellation_token)?;
        // Held to the end of the build: every reading and build below reads stopped ends.
        let _stopped_ends = turn_stopped_ends_on();

        let mut results: Vec<Option<BevelGroupResult>> = vec![None; self.resolutions.len()];
        let mut building = Vec::with_capacity(self.resolutions.len());

        for (index, row_slot) in results.iter_mut().enumerate() {
            check(cancellation_token)?;

            if !self.offered[index] {
                // The refusal the boolean path writes for a group it never builds, said the same
                // way: what went wrong is the SELECTION. The fallback is there so that a future
                // way of being skipped without a reason is a bare sentence rather than a throw.
                let resolution = &self.resolutions[index];
                let reason =
                    bevel_group_failures::diagnose(resolution, resolution.failure_reason.clone())
                        .unwrap_or_else(|| {
                            BevelAdvisory::new(BevelAdvisoryCode::NoEdgesResolved, vec![])
                        });

                *row_slot = Some(BevelGroupResult::new(
                    index as i32,
                    BevelResultGroup::Edge(self.groups[index].clone()),
                    resolution.edges.len() as i32,
                    Some(reason),
                    Vec::new(),
                ));
                continue;
            }

            building.push(index as i32);
        }

        let built = self.build_the_groups_together(&building, &mut results, cancellation_token)?;
        let edge_count_of = |index: usize| self.resolutions[index].edges.len() as i32;
        refuse_the_groups_with_no_row(&building, &mut results, &self.groups, edge_count_of);

        // The rebuild's work is done, built or refused: the bar ends at its end rather than
        // wherever the last window left it.
        self.bar.tell(
            1.0,
            &BevelProgress::plain(BevelProgressPhase::CheckingTheResult),
        );
        let advisories = Some(self.advisories.messages().to_vec());

        let Some(mut built) = built else {
            return Ok(BevelPartOutcome::new(
                None,
                Color::default(),
                results,
                advisories,
            ));
        };

        if self.item_matrix != Matrix4X4::IDENTITY {
            // Out of the space the size was measured in and into the operation's own, the same
            // way the kernel carries an operand matrix for the boolean path. Transformed in place:
            // this mesh was made here, out of vertices nothing else holds.
            built.transform(self.item_matrix);
        }

        // No cleaning pass: this construction shares every seam vertex by index as it makes it,
        // so there is nothing to weld away.
        let color = take_uniform_face_color(&mut built).unwrap_or(if built.face_colors.is_some() {
            Color::TRANSPARENT
        } else {
            self.source_color
        });

        Ok(BevelPartOutcome::new(
            Some(built),
            color,
            results,
            advisories,
        ))
    }

    /// Every buildable group's rim loops, planned on the ORIGINAL
    /// working mesh and built in one pass - dropping and retrying the RIM LOOP a refusal names
    /// rather than refusing the whole pick together. With the sections on, a pass whose plans the
    /// router splits is built IN TWO (`pass_in_two`); off, or where the split is empty or
    /// everybody, this is one pass. `building` are the groups that resolved, as indices into this
    /// part's own lists; `results` the row per group, filled in for every group it touches.
    pub fn build_the_groups_together(
        &mut self,
        building: &[i32],
        results: &mut [Option<BevelGroupResult>],
        cancellation_token: &CancelToken,
    ) -> Result<Option<Mesh>, SurfacePartBuildError> {
        if building.is_empty() {
            return Ok(None);
        }
        // Held to the end of the pass: every corner and end the level line closes under it reads
        // this builder's routing.
        let _corner_cap =
            hold_corner_cap_switches(self.routing.flat_ends, self.routing.corner_hole_fill);

        let mut contributing = self.the_rim_loops_of(building);
        let mut refused_loops: IndexMap<i32, Vec<BevelLoopFailure>> = IndexMap::new();

        let for_legacy = self.the_loops_the_sections_decline(&contributing);

        if !for_legacy.is_empty() && for_legacy.len() < contributing.len() {
            return self.build_in_two(
                &contributing,
                &for_legacy,
                results,
                &mut refused_loops,
                cancellation_token,
            );
        }

        // What blends today keeps its look: the pass is built as it always was, and only where
        // that refuses a group while a face rim could not blend at a cove-to-round corner
        // beside groups that are not face rims is it built again in two, the face rims second
        // (`rim_after_its_partner`) - and that answer taken only where it refuses fewer groups.
        let picked = contributing.clone();
        let rows_before = results.to_vec();
        let loops_before = refused_loops.clone();
        self.a_rim_refused_at_its_flips_in_the_pass = false;
        let built = self.the_one_pass(
            &mut contributing,
            results,
            &mut refused_loops,
            cancellation_token,
            Self::PASSES_END_ON_THE_BAR,
        )?;
        // A group that was building and has no row has nothing to show for it: it counts as
        // refused, so an answer that drops a row never wins on the count.
        let refused = |rows: &[Option<BevelGroupResult>]| {
            building
                .iter()
                .filter(|&&index| rows[index as usize].as_ref().is_none_or(|row| row.failed()))
                .count()
        };
        if !self.a_rim_refused_at_its_flips_in_the_pass || refused(results) == 0 {
            return Ok(built);
        }

        let mut rows_in_two = rows_before;
        let mut loops_in_two = loops_before;
        let in_two = self.the_rim_after_its_partner(
            &picked,
            &mut rows_in_two,
            &mut loops_in_two,
            cancellation_token,
        )?;
        match in_two {
            TheRimAfterItsPartner::Built(in_two) if refused(&rows_in_two) < refused(results) => {
                results.clone_from_slice(&rows_in_two);
                Ok(in_two.map(|mesh| *mesh))
            }
            _ => Ok(built),
        }
    }

    /// One pass over these loops, dropping and retrying the loop a refusal names - or, where it
    /// names none, the smallest set of loops that still refuses on its own - until what is left
    /// builds or nothing is left.
    pub(super) fn the_one_pass(
        &mut self,
        contributing: &mut Vec<Rc<RimLoop>>,
        results: &mut [Option<BevelGroupResult>],
        refused_loops: &mut IndexMap<i32, Vec<BevelLoopFailure>>,
        cancellation_token: &CancelToken,
        passes_end: f64,
    ) -> Result<Option<Mesh>, SurfacePartBuildError> {
        // A pass of its own: a loop left alone in another pass is not this one's to put back.
        self.left_alone_for_touching.clear();
        let picked = contributing.clone();

        while self.put_back_the_loops_whose_partner_went(
            &picked,
            contributing,
            results,
            refused_loops,
        ) {
            check(cancellation_token)?;

            // Each attempt takes most of what is left of the passes' span: one that builds first
            // time fills nearly all of it, and a retry after a refusal still has room to move.
            self.open_an_attempt(refused_loops, passes_end);

            let attempt = self.try_these_groups_together(
                contributing,
                results,
                self.size,
                cancellation_token,
            )?;

            if let Some(built) = attempt.mesh {
                Self::write_what_the_loops_came_to(results, refused_loops);

                return Ok(Some(built));
            }

            // The level line settled a many-loop rim in one go: every loop that cannot stay is
            // named at once, each with its own sentence, and they all go this turn.
            if let Some(condemned) = attempt.condemned.as_ref().filter(|named| !named.is_empty()) {
                if self.say_what_the_settling_named(
                    contributing,
                    condemned,
                    results,
                    refused_loops,
                    cancellation_token,
                )? {
                    continue;
                }

                // The settling named somebody the pass could not write rows for. It must not end
                // here in silence: the settling is switched off for the rest of this rebuild and
                // the same loops go round again down yesterday's path, which always has an answer.
                self.the_settling_is_off = true;

                continue;
            }

            let Some(mut reason) = attempt.reason else {
                return Ok(None);
            };

            let mut blamed = Self::the_loops_holding(contributing, &attempt.guilty);
            let mut about_the_size = attempt.about_the_size;

            if blamed.is_empty() {
                // Nobody in particular: a place about equally near two or more picked edges (a
                // ramp meeting a post) names no loop. The smallest set of loops that still refuses
                // on its own is refused instead, so one corner that cannot fit does not take
                // every other edge of the pick down with it.
                if let Some(narrowed) = self.the_loops_that_still_refuse(
                    contributing,
                    &reason,
                    about_the_size,
                    refused_loops,
                    passes_end,
                    cancellation_token,
                )? {
                    blamed = narrowed.loops;
                    reason = narrowed.reason;
                    about_the_size = narrowed.about_the_size;
                }
            }

            if blamed.is_empty() || blamed.len() == contributing.len() {
                // Nobody in particular, or everybody: the pass IS the answer and every row says so.
                let everybody = contributing.clone();
                self.say(
                    &everybody,
                    &everybody,
                    results,
                    refused_loops,
                    &reason,
                    about_the_size,
                    cancellation_token,
                    None,
                    false,
                )?;
                return Ok(None);
            }

            self.say(
                &blamed,
                contributing,
                results,
                refused_loops,
                &reason,
                about_the_size,
                cancellation_token,
                None,
                false,
            )?;
            contributing.retain(|one| !blamed.contains(one));
        }

        Ok(None)
    }

    /// One refusal written against the loops it belongs to. A refusal about the SIZE
    /// goes through `bevel_group_failures::diagnose`, which reads a chain that never grew past the
    /// picked edge as a coarser question about the pick.
    ///
    /// Where the blamed loops are ALL that is left of a group, the group is refused and the
    /// sentence is its row's; where the group has loops left to build, the sentence is the LOOP's
    /// and goes under a row that builds (`BevelLoopFailure`). `settled` is the size these loops fit
    /// at where it is already KNOWN - the settling's own reading - and `have_settled` whether it
    /// is an answer at all, `None` included: given, no ladder is run, and the reading is confirmed
    /// by building the whole pick at it (`the_settled_size_confirmed`).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn say(
        &mut self,
        saying: &[Rc<RimLoop>],
        contributing: &[Rc<RimLoop>],
        results: &mut [Option<BevelGroupResult>],
        refused_loops: &mut IndexMap<i32, Vec<BevelLoopFailure>>,
        reason: &BevelAdvisory,
        about_the_size: bool,
        cancellation_token: &CancelToken,
        settled: Option<f64>,
        have_settled: bool,
    ) -> Result<(), SurfacePartBuildError> {
        for index in Self::the_groups_of(saying) {
            let blamed: Vec<Rc<RimLoop>> = saying
                .iter()
                .filter(|one| one.group_index == index)
                .cloned()
                .collect();
            let survives = contributing
                .iter()
                .any(|one| one.group_index == index && !blamed.contains(one));
            let refused = if about_the_size {
                bevel_group_failures::diagnose(
                    &self.resolutions[index as usize],
                    Some(reason.clone()),
                )
                .unwrap_or_else(|| reason.clone())
            } else {
                reason.clone()
            };
            let quoted = bevel_advisory_codes::try_the_most_that_fits(Some(&refused));

            if survives {
                for one in &blamed {
                    let alone = [one.clone()];
                    let for_the_loop = if have_settled {
                        self.the_settled_size_confirmed(
                            &alone,
                            contributing,
                            settled,
                            cancellation_token,
                        )?
                    } else {
                        self.the_largest_that_builds(
                            &alone,
                            contributing,
                            quoted,
                            cancellation_token,
                        )?
                    };

                    let said_for_the_loop = Self::the_refusal_offering(
                        self.routing.level_line,
                        &refused,
                        quoted,
                        for_the_loop,
                    );

                    refused_loops
                        .entry(index)
                        .or_default()
                        .push(BevelLoopFailure {
                            advisory: said_for_the_loop,
                            largest_size_that_fits: for_the_loop,
                            edge_count: one.edges.len() as i32,
                        });
                }

                continue;
            }

            // The size that WOULD fit, which is the only thing the user can act on - and found by
            // BUILDING at it, because that is the claim being made. A refusal about the size has
            // a number of its own and it goes in as the first guess, but it is the ROOM's answer
            // and not the build's.
            let the_groups_loops = self.the_rim_loops_of_the_group(index);
            let largest = if have_settled {
                self.the_settled_size_confirmed(
                    &the_groups_loops,
                    contributing,
                    settled,
                    cancellation_token,
                )?
            } else {
                self.the_largest_that_builds(
                    &the_groups_loops,
                    contributing,
                    quoted,
                    cancellation_token,
                )?
            };

            let refused =
                Self::the_refusal_offering(self.routing.level_line, &refused, quoted, largest);
            let mut row = BevelGroupResult::new(
                index,
                BevelResultGroup::Edge(self.groups[index as usize].clone()),
                self.resolutions[index as usize].edges.len() as i32,
                Some(refused),
                Vec::new(),
            );
            row.largest_size_that_fits = largest;
            row.loop_count = the_groups_loops.len() as i32;
            results[index as usize] = Some(row);
        }

        Ok(())
    }
}
