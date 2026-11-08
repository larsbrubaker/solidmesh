// The level-line route's second half: the build - the pick cut and
// filled in ONE call at the size, and what came of it: the pass's mesh, a size refusal carrying
// the room, or nothing at all and the pass falls through. A combination the pick-wide call cannot
// take is built a chain at a time, a knife tip rounds first, two rounds that overlap across a
// face meet in a crest (`band_gap_crest`), and a rim whose fill refuses because a rail runs
// backward next to a cove-to-round corner is built again cut into runs at its flips
// (`chain_on_the_mesh::sense_flips_near`, `WorkBudget::sense_flips_to_cut`, `cove_round_retry`),
// whichever reading named the room; every answer is read for closure and for crossing itself
// before it is published. `level_line` (the first half) reads the pick and
// its room and calls this.

use std::cell::RefCell;
use std::rc::Rc;

use crate::bevel::level_line::band_gap::BandGapBuildError;
use crate::bevel::level_line::band_gap_crest::build_crest_in_pick;
use crate::bevel::level_line::band_gap_entry_points::build_pick;
use crate::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use crate::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn, BuildAndFillError,
};
use crate::bevel::level_line::band_gap_rounds_first::{build_rounds_first, takes_rounds_first};
use crate::bevel::level_line::band_side_groups::has_a_both_sided_member;
use crate::bevel::level_line::gap_fill::fill_with_budget;
use crate::bevel::level_line::gap_fill::GapFillError;
use crate::bevel::level_line::open_gap_fill;
use crate::bevel::level_line::seam_edges::FeatureEdgeTest;
use crate::bevel::level_line::{
    level_line_routing, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult,
    LevelLineWorkPhase, RoomReadingKind, RoomSource, WorkBudget,
};
use crate::bevel::part::answer_mesh::the_mesh_of;
use crate::bevel::part::self_intersection_check::the_answer_intersects_itself_where_the_source_did_not as crosses_itself;
use crate::bevel::part::BevelProgressPhase;
use crate::bevel::surface::surface_mesh_closure;
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use super::cove_round_retry::{
    the_flips_to_cut, the_rim_cut_at_its_flips, TheOneChainRefusal, ThePick,
};
use super::level_line::{check, LevelLineAnswer, LevelLineRouteError};
use super::level_line_kernel_handoff::{
    the_budget_for, the_kept_triangles, the_size_refusal_for, the_vertex_to_blame,
};
use super::level_line_memo::THE_ROUTES_SNAP;
use super::BevelSurfacePartBuilder;

/// The kernel's filled mesh as a `Mesh`, carrying only its kept triangles (`the_mesh_of` over
/// `the_kept_triangles`).
fn the_answer_of(filled: &GapFillResult) -> Mesh {
    match filled.mesh.as_ref() {
        Some(mesh) => {
            let mesh = mesh.borrow();
            the_mesh_of(&mesh.vertices, &the_kept_triangles(&mesh))
        }
        // A filled answer always carries its mesh; an empty one is an open part, read as such.
        None => Mesh::new(),
    }
}

/// What the in-turn build said when it refused, for the route.
fn in_turn_said(filled: &GapFillResult, chains: usize) -> String {
    let site = if matches!(
        filled.refusal,
        GapFillRefusal::OpenChainEnds | GapFillRefusal::GrowingNeighbourClipRefused
    ) {
        format!(" ({})", open_gap_fill::ends_site().unwrap_or_default())
    } else {
        String::new()
    };

    format!(
        "; in turn: chain {} of {} {:?}{}",
        filled.offending_chain, chains, filled.refusal, site
    )
}

/// Whether a crest attempt stopped because the build was cancelled.
fn the_crest_was_cancelled(error: &BuildAndFillError) -> bool {
    matches!(
        error,
        BuildAndFillError::Build(BandGapBuildError::Cancelled)
            | BuildAndFillError::Fill(GapFillError::Cancelled)
    )
}

impl BevelSurfacePartBuilder {
    /// A band that FLOWED over an end face narrower than the setback
    /// is an improvement only when it makes a part. Where it does not, the pick answers exactly as
    /// it did before the flow: refused by size, naming the room it had without it. `None`,
    /// touching nothing, for every band that did not flow.
    #[allow(clippy::too_many_arguments)]
    fn the_unflowed_size_refusal(
        &mut self,
        gap: &BandGapResult,
        pick: &[Vec<i32>],
        closed: &[bool],
        profile: BandProfile,
        room_set_by: RoomSource,
        trial_size: f64,
        positions: &[Vector3],
    ) -> Option<BevelAdvisory> {
        let room = gap.unflowed_room?;

        self.remember_a_smaller_room(
            pick,
            closed,
            profile,
            room,
            room_set_by,
            -1,
            Some(trial_size),
        );
        level_line_routing::set_last_route(Some("level-line: refused by size".to_string()));

        Some(BevelAdvisory::new(
            the_size_refusal_for(&room_set_by),
            vec![
                BevelAdvisoryArg::Double(trial_size),
                BevelAdvisoryArg::Vector3(room_set_by.crease_at.unwrap_or_else(|| {
                    positions[the_vertex_to_blame(pick, &room_set_by) as usize]
                })),
                BevelAdvisoryArg::Double(room),
            ],
        ))
    }

    /// The pick cut and filled in ONE call at the size. `known_room` is
    /// the room to hand the build, which the build TRUSTS; `known_room_set_by` is which reading
    /// named it - a build handed its room reads none of its own, and a size refusal needs the
    /// source to say which of its two sentences is true.
    #[allow(clippy::too_many_arguments)]
    pub fn the_level_line_build(
        &mut self,
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        pick: &[Vec<i32>],
        closed: &[bool],
        is_feature_edge: FeatureEdgeTest,
        profile: BandProfile,
        fillet_segments: i32,
        trial_size: f64,
        known_room: Option<f64>,
        known_room_set_by: RoomSource,
        cancellation_token: &CancelToken,
    ) -> Result<LevelLineAnswer, LevelLineRouteError> {
        let mut answer = LevelLineAnswer::default();
        self.the_end_capped_chain = None;
        self.the_rim_refused_at_its_flips = false;
        let chains = pick.len() as i32;

        // Said before the call: a build handed a room it refuses at once reports nothing of its
        // own, and the row would jump from whatever it said last straight to the size search.
        self.bar.tell(
            self.bar.high_water(),
            &self.the_progress_for(LevelLineWorkPhase::Sides, chains),
        );

        let seam_epsilon = self.the_seam_epsilon();
        let sharp_turn_degrees = self.the_sharp_turn_degrees;
        let words_for = |builder: &Self, from: f64, to: f64| {
            builder
                .bar
                .slice(from, to, move |phase| BevelProgressPhase::Building {
                    phase,
                    loops: chains,
                })
        };

        let gap = build_pick(
            positions,
            indices,
            pick,
            closed,
            Some(is_feature_edge.clone()),
            trial_size,
            profile,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: THE_ROUTES_SNAP,
                known_room,
                budget: Some(the_budget_for(
                    indices.len() as i32,
                    cancellation_token,
                    words_for(
                        self,
                        Self::READING_ENDS_IN_THE_PASS,
                        Self::BUILD_ENDS_IN_THE_PASS,
                    ),
                )),
                seam_epsilon,
                sharp_turn_degrees,
                fillet_segments,
            },
        )?;
        // Read by `the_touching_loops_left_alone`.
        self.the_band_refused_a_touch = gap.refusal == BandGapRefusal::ChainsTouch;

        // A pick whose bands meet at EVERY size the pinch ladder tried carries a room of zero, and
        // that is not a size: it is the level line saying it cannot build this pick at all.
        if gap.refusal == BandGapRefusal::Size && gap.pinches_at_every_size_tried {
            self.the_level_line_passes_it_on(
                "existing: the band's fronts meet at every size".to_string(),
            );
            return Ok(answer);
        }

        // A pick holding a BOTH-SIDED member is new to the level line, and its room is not yet a
        // promise, so such a pick refused by size falls through as it did.
        if gap.refusal == BandGapRefusal::Size
            && has_a_both_sided_member(
                positions,
                indices,
                Some(pick),
                closed,
                Some(&|a: i32, b: i32| is_feature_edge(a, b)),
            )?
        {
            self.the_level_line_passes_it_on(
                "existing: a pick with a both-sided member refused by size".to_string(),
            );
            return Ok(answer);
        }

        // Which reading named the room this build refused at: its own where it read one, and the
        // room it was handed otherwise.
        let room_set_by = if gap.room_set_by.kind != RoomReadingKind::None {
            gap.room_set_by
        } else {
            known_room_set_by
        };

        // Two rounds that overlap across the face between them meet in a crest there instead,
        // where the crest builds, closes and does not cross itself; anywhere else the size
        // refusal below stands exactly as it is. The overlap is read off the pick's shape, not off
        // which reading named the room: at half the face two readings tie and the last bits
        // decide which one wins.
        if gap.refusal == BandGapRefusal::Size {
            let crest = build_crest_in_pick(
                positions,
                indices,
                pick,
                closed,
                Some(is_feature_edge.clone()),
                trial_size,
                profile,
                &BandGapBuildOptions {
                    chord_tolerance: 0.0,
                    snap: THE_ROUTES_SNAP,
                    known_room: None,
                    budget: Some(the_budget_for(
                        indices.len() as i32,
                        cancellation_token,
                        words_for(self, Self::BUILD_ENDS_IN_THE_PASS, 1.0),
                    )),
                    seam_epsilon,
                    sharp_turn_degrees,
                    fillet_segments,
                },
                &room_set_by,
            );
            // The attempt is an offer, never a new way to fail: a fault anywhere in it is no
            // crest, and the refusal below follows word for word. Only a cancel stops the pass.
            let crest = match crest {
                Ok(crest) => crest,
                Err(error) if the_crest_was_cancelled(&error) => {
                    return Err(LevelLineRouteError::Cancelled)
                }
                Err(_) => None,
            };
            check(cancellation_token)?;
            if let Some(crest) = crest.filter(GapFillResult::filled) {
                let mesh = the_answer_of(&crest);
                if surface_mesh_closure::is_closed(&mesh).0
                    && !crosses_itself(&mesh, positions, indices, cancellation_token)?
                {
                    level_line_routing::set_last_route(Some(format!(
                        "{}: crest",
                        level_line_routing::LEVEL_LINE_ROUTE
                    )));
                    let stamp = self.the_stamp();
                    let key = self.the_memo_key_of(pick, closed, profile, THE_ROUTES_SNAP);
                    self.memo
                        .remember_a_size_that_built(stamp, &key, trial_size);
                    answer.mesh = Some(mesh);
                    return Ok(answer);
                }
            }
        }

        if gap.refusal == BandGapRefusal::Size {
            // Does not fall through by name: the room is a true number about the level line's
            // band and the user is owed it. A refusal can name a room SMALLER than the one read
            // off the sides' width - a side that pinches is found by the build - remembered as a
            // floor for this size and every size above it.
            self.the_chain_the_refusal_names = self.the_chain_named_with_its_room_remembered(
                &gap,
                room_set_by,
                pick,
                closed,
                profile,
                known_room,
                trial_size,
            );
            let key = self.the_memo_key_of(pick, closed, profile, THE_ROUTES_SNAP);
            self.the_refusal_is_a_pinch = gap.pinch_probes > 0
                || self
                    .memo
                    .the_pinched_chain_under_the_floor(&key, known_room, Some(trial_size))
                    >= 0;

            level_line_routing::set_last_route(Some("level-line: refused by size".to_string()));

            // The room itself, NOT taken down by `the_size_to_offer`: this refusal goes to Say,
            // whose ladder BUILDS at the quoted number and falls back to the percent only where
            // that fails.
            answer.reason = Some(BevelAdvisory::new(
                the_size_refusal_for(&room_set_by),
                vec![
                    BevelAdvisoryArg::Double(trial_size),
                    BevelAdvisoryArg::Vector3(room_set_by.crease_at.unwrap_or_else(|| {
                        positions[the_vertex_to_blame(pick, &room_set_by) as usize]
                    })),
                    BevelAdvisoryArg::Double(gap.room),
                ],
            ));
            answer.about_the_size = true;
            return Ok(answer);
        }

        // A COMBINATION the pick-wide call cannot take is built a chain at a time before it falls
        // through, each exactly as it is alone, on the part the chains before it left. The answer
        // is read for closure below like every other.
        let mut said_in_turn: Option<String> = None;
        let in_turn_budgets = RefCell::new(self.the_in_turn_budgets(chains, cancellation_token));
        let budget_for = |count: i32| -> Rc<WorkBudget> { (in_turn_budgets.borrow_mut())(count) };

        let the_chains_in_turn = |said_in_turn: &mut Option<String>| -> Result<
            Option<GapFillResult>,
            LevelLineRouteError,
        > {
            if pick.len() <= 1 {
                return Ok(None);
            }

            let in_turn = build_open_chains_in_turn(
                positions,
                indices,
                pick,
                Some(is_feature_edge.clone()),
                trial_size,
                profile,
                fillet_segments,
                None,
                THE_ROUTES_SNAP,
                seam_epsilon,
                sharp_turn_degrees,
                Some(&budget_for),
                Some(closed),
            )?;

            if !in_turn.filled() {
                *said_in_turn = Some(in_turn_said(&in_turn, pick.len()));
                return Ok(None);
            }

            Ok(Some(in_turn))
        };

        // A KNIFE TIP - a convex chain's end meeting concave chains' ends at one vertex - is built
        // rounds first: the convex chains alone, then the concave creases read again on what they
        // left. Held to the same closure and self-intersection checks; where it refuses or fails
        // them, the pick goes on to today's route exactly as before.
        if takes_rounds_first(positions, indices, pick, Some(closed)) {
            let rounds_first = build_rounds_first(
                positions,
                indices,
                pick,
                Some(is_feature_edge.clone()),
                trial_size,
                profile,
                fillet_segments,
                None,
                THE_ROUTES_SNAP,
                seam_epsilon,
                sharp_turn_degrees,
                Some(&budget_for),
                Some(closed),
            )?;
            check(cancellation_token)?;
            if rounds_first.filled() {
                let mesh = the_answer_of(&rounds_first);
                if surface_mesh_closure::is_closed(&mesh).0
                    && !crosses_itself(&mesh, positions, indices, cancellation_token)?
                {
                    level_line_routing::set_last_route(Some(format!(
                        "{}: rounds first",
                        level_line_routing::LEVEL_LINE_ROUTE
                    )));
                    let stamp = self.the_stamp();
                    let key = self.the_memo_key_of(pick, closed, profile, THE_ROUTES_SNAP);
                    self.memo
                        .remember_a_size_that_built(stamp, &key, trial_size);
                    answer.mesh = Some(mesh);
                    return Ok(answer);
                }
            }
        }

        let mut took_in_turn = takes_open_chains_in_turn(Some(&gap), closed);
        let mut filled;

        if !gap.built() {
            match the_chains_in_turn(&mut said_in_turn)? {
                Some(in_turn) => filled = in_turn,
                None => {
                    self.the_level_line_passes_it_on(format!(
                        "existing: the band refused, {:?}{}",
                        gap.refusal,
                        said_in_turn.as_deref().unwrap_or_default()
                    ));
                    return Ok(answer);
                }
            }

            took_in_turn = true;
        } else if took_in_turn {
            // A pick of several open chains that share no loop is built a chain at a time, each
            // exactly as it is alone, under the pick-wide room.
            filled = build_open_chains_in_turn(
                positions,
                indices,
                pick,
                Some(is_feature_edge.clone()),
                trial_size,
                profile,
                fillet_segments,
                Some(gap.room),
                THE_ROUTES_SNAP,
                seam_epsilon,
                sharp_turn_degrees,
                Some(&budget_for),
                None,
            )?;
        } else {
            let triangles = gap
                .mesh
                .as_ref()
                .map_or(indices.len() as i32, |mesh| mesh.borrow().triangle_count());
            let budget = the_budget_for(
                triangles,
                cancellation_token,
                words_for(self, Self::BUILD_ENDS_IN_THE_PASS, 1.0),
            );
            filled = fill_with_budget(&gap, profile, fillet_segments, Some(budget))?;
        }

        // The flips next to where the pick-wide one-chain fill refused a face rim because a rail ran
        // backward (`cove_round_retry::the_flips_to_cut`), read once: the offer below cuts the rim
        // there, and where the offer does not build, the pass may build the rim after its partner
        // (`rim_after_its_partner`).
        let the_pick = ThePick {
            positions,
            indices,
            chains: pick,
            closed,
            is_feature_edge: is_feature_edge.clone(),
        };
        let flips = if !filled.filled() && !took_in_turn {
            the_flips_to_cut(
                &TheOneChainRefusal {
                    gap: &gap,
                    filled: &filled,
                },
                &the_pick,
            )
        } else {
            Vec::new()
        };

        if !filled.filled() && !took_in_turn {
            if let Some(after_the_fill) = the_chains_in_turn(&mut said_in_turn)? {
                filled = after_the_fill;
                took_in_turn = true;
            }
        }

        // A rim whose blend cannot be built at a cove-to-round corner (`cove_round_retry`): the
        // pick-wide one-chain fill refused it because a rail ran backward next to such a corner,
        // so the rim is offered again cut at those flips. Not after the build a chain at a time:
        // its refusal's vertex is on a different mesh, and the route's words name that build.
        if !filled.filled() && !took_in_turn && !flips.is_empty() {
            let budget = the_budget_for(
                indices.len() as i32,
                cancellation_token,
                words_for(self, Self::BUILD_ENDS_IN_THE_PASS, 1.0),
            );
            if let Some(mesh) = the_rim_cut_at_its_flips(
                &the_pick,
                flips.clone(),
                trial_size,
                profile,
                fillet_segments,
                seam_epsilon,
                sharp_turn_degrees,
                budget,
                cancellation_token,
            )? {
                level_line_routing::set_last_route(Some(format!(
                    "{}: cut at its cove-to-round corners",
                    level_line_routing::LEVEL_LINE_ROUTE
                )));
                let stamp = self.the_stamp();
                let key = self.the_memo_key_of(pick, closed, profile, THE_ROUTES_SNAP);
                self.memo
                    .remember_a_size_that_built(stamp, &key, trial_size);
                answer.mesh = Some(mesh);
                return Ok(answer);
            }
        }

        // Noted for the pass only where the offer did not build, and only for a build at the
        // pass's own size: a narrowing's or a size trial's refusal is not the pass's.
        self.the_rim_refused_at_its_flips =
            !filled.filled() && !flips.is_empty() && trial_size == self.size;
        self.a_rim_refused_at_its_flips_in_the_pass |= self.the_rim_refused_at_its_flips;

        if !filled.filled() {
            if let Some(unflowed) = self.the_unflowed_size_refusal(
                &gap,
                pick,
                closed,
                profile,
                room_set_by,
                trial_size,
                positions,
            ) {
                answer.reason = Some(unflowed);
                answer.about_the_size = true;
                return Ok(answer);
            }

            let site = if matches!(
                filled.refusal,
                GapFillRefusal::OpenChainEnds | GapFillRefusal::GrowingNeighbourClipRefused
            ) {
                format!(" ({})", open_gap_fill::ends_site().unwrap_or_default())
            } else {
                String::new()
            };
            self.the_level_line_passes_it_on(format!(
                "existing: the fill refused, {:?}{}{}",
                filled.refusal,
                site,
                said_in_turn.as_deref().unwrap_or_default()
            ));
            return Ok(answer);
        }

        if filled.ends_capped_by_end_faces && pick.len() == 1 {
            self.the_end_capped_chain = Some(pick[0].clone());
        }

        // Not written back over positions and indices: the chains in turn read the part as it
        // came in.
        check(cancellation_token)?;
        let mut mesh = the_answer_of(&filled);
        check(cancellation_token)?;

        if !took_in_turn && !surface_mesh_closure::is_closed(&mesh).0 {
            if let Some(after_the_closure) = the_chains_in_turn(&mut said_in_turn)? {
                mesh = the_answer_of(&after_the_closure);
            }
        }

        self.bar
            .finish_the_window(&self.bar.said(BevelProgressPhase::CheckingTheResult));
        check(cancellation_token)?;

        let built_in_turn = |said_in_turn: &Option<String>, took_in_turn: bool| -> String {
            said_in_turn.clone().unwrap_or_else(|| {
                if took_in_turn {
                    "; built in turn".to_string()
                } else {
                    String::new()
                }
            })
        };

        // Read on the answer, as the existing path reads its own: a construction that shares
        // every seam vertex by index either closes or has a defect in it. Here it falls THROUGH
        // rather than refusing, because the flag must never make a pick worse than it is today.
        if !surface_mesh_closure::is_closed(&mesh).0 {
            if let Some(unflowed) = self.the_unflowed_size_refusal(
                &gap,
                pick,
                closed,
                profile,
                room_set_by,
                trial_size,
                positions,
            ) {
                answer.reason = Some(unflowed);
                answer.about_the_size = true;
                return Ok(answer);
            }

            self.the_level_line_passes_it_on(format!(
                "existing: the level line's mesh did not close{}",
                built_in_turn(&said_in_turn, took_in_turn)
            ));
            return Ok(answer);
        }

        // A closed answer can still cross itself. A self-intersecting part is a WRONG SHAPE, so
        // it is read here like an unclosed one - a validity check that falls through, never a
        // repair. A pick-wide answer that crosses itself is tried again a chain at a time before
        // it falls through; the pick-wide verdict is written first, so the first route says why
        // it went in turn.
        let mut crosses = crosses_itself(&mesh, positions, indices, cancellation_token)?;
        if crosses && !took_in_turn && pick.len() > 1 {
            self.the_level_line_passes_it_on(
                "existing: the level line's mesh intersects itself; built in turn".to_string(),
            );
            let Some(after_the_crossing) = the_chains_in_turn(&mut said_in_turn)? else {
                self.the_level_line_passes_it_on(format!(
                    "existing: the level line's mesh intersects itself{}",
                    said_in_turn.as_deref().unwrap_or_default()
                ));
                return Ok(answer);
            };

            took_in_turn = true;
            mesh = the_answer_of(&after_the_crossing);
            check(cancellation_token)?;
            if !surface_mesh_closure::is_closed(&mesh).0 {
                self.the_level_line_passes_it_on(
                    "existing: the level line's mesh intersects itself; in turn: the mesh did not close"
                        .to_string());
                return Ok(answer);
            }

            crosses = crosses_itself(&mesh, positions, indices, cancellation_token)?;
        }

        if crosses {
            if let Some(unflowed) = self.the_unflowed_size_refusal(
                &gap,
                pick,
                closed,
                profile,
                room_set_by,
                trial_size,
                positions,
            ) {
                answer.reason = Some(unflowed);
                answer.about_the_size = true;
                return Ok(answer);
            }

            self.the_level_line_passes_it_on(format!(
                "existing: the level line's mesh intersects itself{}",
                built_in_turn(&said_in_turn, took_in_turn)
            ));
            return Ok(answer);
        }

        level_line_routing::set_last_route(Some(level_line_routing::LEVEL_LINE_ROUTE.to_string()));
        let stamp = self.the_stamp();
        let key = self.the_memo_key_of(pick, closed, profile, THE_ROUTES_SNAP);
        self.memo
            .remember_a_size_that_built(stamp, &key, trial_size);

        answer.mesh = Some(mesh);
        Ok(answer)
    }
}
