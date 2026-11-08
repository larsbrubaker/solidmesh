// The level-line route's first half: the level line offered a pass, behind the level-line
// routing flag - the pick linked into runs, settled where it is a many-loop
// rim, its room read once, and handed to the one build (`level_line_build`, the second half).
// Only a pass whose picked edges LINK into runs - closed loops, such as a
// face rim, and open runs - reaches the kernel; anything else, and anything the kernel refuses for
// a reason that is not the size, falls through to the existing path unchanged, so the flag can
// never make a pick worse than it is today. A SIZE refusal does not fall through: the room the
// kernel names is the honest answer, said with the existing size advisory.
//
// The pass's loops are handed to the kernel as ONE PICK - one room, one `BandGap.Build`, one
// `GapFill.Fill` - so a face rim with a hole in it is cut against a field that knows about both
// loops at once.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use crate::bevel::bevel_chain_linker;
use crate::bevel::level_line::band_gap::BandGapBuildError;
use crate::bevel::level_line::band_gap_in_turn::BuildAndFillError;
use crate::bevel::level_line::band_gap_reading::{self, ReadRoomError};
use crate::bevel::level_line::band_side_tagging_ends::SideTaggingError;
use crate::bevel::level_line::gap_fill::GapFillError;
use crate::bevel::level_line::level_line_routing;
use crate::bevel::level_line::seam_edges::FeatureEdgeTest;
use crate::bevel::level_line::{
    BandProfile, LevelLineChainWatch, RoomReadingKind, RoomSource, WorkBudget,
};
use crate::bevel::part::{BevelProgressPhase, SettledPick};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg};
use crate::topology::FeatureEdge;
use indexmap::IndexSet;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use super::level_line_kernel_handoff::{
    the_budget_for, the_edge_key, the_room_to_hand_the_build, the_size_refusal_for,
};
use super::level_line_memo::THE_ROUTES_SNAP;
use super::{BevelSurfacePartBuilder, RimLoop};

/// Why a level-line pass could not run at all: the rebuild's cancel or a kernel argument check.
#[derive(Debug)]
pub enum LevelLineRouteError {
    /// The rebuild was cancelled.
    Cancelled,
    /// A room reading could not run.
    ReadRoom(ReadRoomError),
    /// The pick-wide cut could not run.
    Build(BandGapBuildError),
    /// A fill could not run.
    Fill(GapFillError),
    /// A build a chain at a time (or rounds first) could not run.
    BuildAndFill(BuildAndFillError),
    /// The both-sided reading could not run.
    SideTagging(SideTaggingError),
}

impl From<polygon_mesh::Cancelled> for LevelLineRouteError {
    fn from(_: polygon_mesh::Cancelled) -> Self {
        LevelLineRouteError::Cancelled
    }
}

impl From<ReadRoomError> for LevelLineRouteError {
    fn from(error: ReadRoomError) -> Self {
        LevelLineRouteError::ReadRoom(error)
    }
}

impl From<BandGapBuildError> for LevelLineRouteError {
    fn from(error: BandGapBuildError) -> Self {
        LevelLineRouteError::Build(error)
    }
}

impl From<GapFillError> for LevelLineRouteError {
    fn from(error: GapFillError) -> Self {
        LevelLineRouteError::Fill(error)
    }
}

impl From<BuildAndFillError> for LevelLineRouteError {
    fn from(error: BuildAndFillError) -> Self {
        LevelLineRouteError::BuildAndFill(error)
    }
}

impl From<SideTaggingError> for LevelLineRouteError {
    fn from(error: SideTaggingError) -> Self {
        LevelLineRouteError::SideTagging(error)
    }
}

/// `Err(Cancelled)` once the rebuild has been cancelled.
pub(super) fn check(cancellation_token: &CancelToken) -> Result<(), LevelLineRouteError> {
    if cancellation_token.is_cancelled() {
        Err(LevelLineRouteError::Cancelled)
    } else {
        Ok(())
    }
}

/// What one pass through the level line came to: whether it built, and what it says either way.
#[derive(Default)]
pub struct LevelLineAnswer {
    /// The pass's mesh where it built.
    pub mesh: Option<Mesh>,
    /// The size advisory where the kernel refused by size; `None` otherwise.
    pub reason: Option<BevelAdvisory>,
    /// Whether `reason` is a size refusal, as the caller reads it.
    pub about_the_size: bool,
    /// The loops a settling named, where it named any (the caller drops them and calls again).
    pub condemned: Option<Vec<TheChainCondemned>>,
}

impl LevelLineAnswer {
    /// Whether the pass built.
    pub fn built(&self) -> bool {
        self.mesh.is_some()
    }
}

/// One chain the settling condemned, as the caller needs it - the mesh
/// edges that say WHICH loops it is, and the sentence those loops are owed.
#[derive(Debug, Clone)]
pub struct TheChainCondemned {
    /// The mesh edges of the chain.
    pub mesh_edges: HashSet<i32>,
    /// The sentence the chain's loops are owed.
    pub reason: BevelAdvisory,
    /// The largest size this chain fits at beside the neighbours it was read with - the
    /// settling's own number, margin already off - or `None` where the reading names none, which
    /// is what a chain that TOUCHES another has.
    pub settled: Option<f64>,
}

/// The chains of a pick as three lists: each run's vertices, whether it closes, and its mesh edges.
pub type TheChains = (Vec<Vec<i32>>, Vec<bool>, Vec<HashSet<i32>>);

impl BevelSurfacePartBuilder {
    /// Fillet rows where the operation has no resolution of
    /// its own to lend.
    pub const LEVEL_LINE_DEFAULT_FILLET_SEGMENTS: i32 = 8;

    /// The working mesh's positions and triangles as the kernel reads them: every vertex as a
    /// double-precision position and every face as its three vertex indices.
    pub(super) fn the_kernels_mesh(&self) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
        let positions = self
            .working_mesh
            .vertices
            .iter()
            .map(|vertex| Vector3::new(vertex.x as f64, vertex.y as f64, vertex.z as f64))
            .collect();
        let indices = self
            .working_mesh
            .faces
            .iter()
            .map(|face| (face.v0, face.v1, face.v2))
            .collect();

        (positions, indices)
    }

    /// The level line passing this pick on, `route` saying why (the level line's route record, a
    /// test seam: it starts "existing: ", the construction the pass goes on to).
    pub fn the_level_line_passes_it_on(&self, route: String) {
        level_line_routing::set_last_route(Some(route));
    }

    /// This pass through the level line, or nothing and why not.
    pub fn try_the_level_line(
        &mut self,
        contributing: &[Rc<RimLoop>],
        trial_size: f64,
        cancellation_token: &CancelToken,
    ) -> Result<LevelLineAnswer, LevelLineRouteError> {
        let answer = LevelLineAnswer::default();
        self.the_chain_the_refusal_names = -1;
        self.forget_the_last_refusals_reading();

        if !level_line_routing::use_level_line() {
            self.the_level_line_passes_it_on("existing: the flag is off".to_string());
            return Ok(answer);
        }

        // The kernel polls its own work; these whole-part passes outside it are polled here, so a
        // Cancel lands within a pass wherever the build is.
        check(cancellation_token)?;
        let Some((chains, closed, chain_edges)) = Self::the_chains_of(contributing) else {
            self.the_level_line_passes_it_on(
                "existing: the pick does not link into chains".to_string(),
            );
            return Ok(answer);
        };

        self.the_chain_edges_of_the_pass = Some(chain_edges.clone());

        check(cancellation_token)?;

        let group_indices = Self::the_groups_of(contributing);

        if group_indices.is_empty() {
            self.the_level_line_passes_it_on("existing: no groups".to_string());
            return Ok(answer);
        }

        // The operation's OWN feature reading, at the bend threshold every resolution was read
        // at - the same set the flood is not allowed to cross.
        let features = self.resolutions[group_indices[0] as usize].features.clone();

        // The same threshold is the chain field's sharp-turn angle: a turn the operation calls a
        // bend is one whose outside the band mitres. It must reach the room and the build together.
        self.the_sharp_turn_degrees = features.flat_angle_degrees();
        // In the order they are added, so every run sees the same order - the
        // watch hands the list on.
        let mut feature_edges: IndexSet<(i32, i32)> = IndexSet::new();

        for edge in features.edges() {
            feature_edges.insert(the_edge_key(edge.start_vertex, edge.end_vertex));
        }

        let feature_edges = Rc::new(feature_edges);
        let is_feature_edge: FeatureEdgeTest = {
            let feature_edges = feature_edges.clone();
            Rc::new(move |a: i32, b: i32| feature_edges.contains(&the_edge_key(a, b)))
        };

        let (positions, indices) = self.the_kernels_mesh();
        check(cancellation_token)?;
        let profile = if self.round_profile {
            BandProfile::Fillet
        } else {
            BandProfile::Chamfer
        };
        let fillet_segments = if self.segments > 0 {
            self.segments
        } else {
            Self::LEVEL_LINE_DEFAULT_FILLET_SEGMENTS
        };

        let pick = chains;

        if let Some(watch) = level_line_routing::watch() {
            let seam_epsilon = self.the_seam_epsilon();
            watch(&LevelLineChainWatch {
                pick: None,
                positions: positions.clone(),
                indices: indices.clone(),
                chains: pick.clone(),
                closed: closed.clone(),
                feature_edges: feature_edges.iter().copied().collect(),
                size: trial_size,
                fillet_segments,
                round_profile: self.round_profile,
                seam_epsilon,
                sharp_turn_degrees: self.the_sharp_turn_degrees,
            });
        }

        check(cancellation_token)?;

        // A many-loop RIM is settled before anything is built: the kernel is asked, in one call,
        // who cannot be built at this size. What it names goes back to the caller as loops to
        // drop, and the pass that follows builds everybody who is left. The settling declines a
        // pick it is not entitled to judge - one where two chains touch - and every verdict it
        // does give is used only where it maps cleanly onto the user's LOOPS. Settled ONCE for a
        // drag, not at every step: the memo says when its answer still holds at another size.
        let settle_key =
            if pick.len() as i32 >= Self::THE_MANY_LOOP_RIM_FLOOR && !self.the_settling_is_off {
                Some(self.the_memo_key_of(&pick, &closed, profile, THE_ROUTES_SNAP))
            } else {
                None
            };

        let mut settled: Option<Arc<SettledPick>> = settle_key
            .as_ref()
            .and_then(|key| self.memo.the_settling_to_reuse(key, trial_size));

        if let (Some(key), None) = (settle_key.as_ref(), settled.as_ref()) {
            let loops = pick.len() as i32;
            let slice = self
                .bar
                .slice(0.0, Self::READING_ENDS_IN_THE_PASS, move |_| {
                    BevelProgressPhase::CheckingWhichLoopsFit { loops }
                });
            let budget = the_budget_for(indices.len() as i32, cancellation_token, slice);
            let seam_epsilon = self.the_seam_epsilon();

            settled = Self::the_settled_pick(
                &positions,
                &indices,
                &pick,
                &closed,
                Some(is_feature_edge.clone()),
                profile,
                trial_size,
                Some(budget),
                seam_epsilon,
                self.the_sharp_turn_degrees,
            )?
            .map(Arc::new);

            if let Some(settled) = settled.as_ref() {
                let survivors_room = the_room_to_hand_the_build(settled.last.as_ref());
                let stamp = self.the_stamp();

                self.memo.remember_the_settling(
                    stamp,
                    key,
                    trial_size,
                    settled.clone(),
                    survivors_room,
                );

                // The pass that follows a verdict comes back with the survivors alone. Its
                // settling is this settling's LAST reading - a reading of exactly that set - so it
                // is written under the survivors' own key rather than paid for twice.
                if !settled.condemned.is_empty() && settled.unresolved.is_empty() {
                    self.remember_the_survivors(
                        settled,
                        &pick,
                        &closed,
                        profile,
                        trial_size,
                        survivors_room,
                    );
                }
            }
        }

        if let Some(settled) = settled.as_ref().filter(|one| one.unresolved.is_empty()) {
            let loop_edges: Vec<HashSet<i32>> = contributing
                .iter()
                .map(|one| one.edges.iter().map(|edge| edge.mesh_edge_index).collect())
                .collect();
            let condemned_edges: Vec<HashSet<i32>> = settled
                .condemned
                .iter()
                .map(|one| chain_edges[one.chain as usize].clone())
                .collect();

            if !settled.condemned.is_empty()
                && !settled.chains.is_empty()
                && Self::the_loops_the_settling_condemns(&loop_edges, &condemned_edges).is_some()
            {
                // Named, not built: the caller drops these loops - writing each its own row with
                // the reason and the size the settling read - and calls again with what is left.
                let condemned =
                    Self::the_condemned_loops(settled, &pick, &chain_edges, &positions, trial_size);
                level_line_routing::set_last_route(Some(format!(
                    "level-line: {} of {} chains cannot stay",
                    condemned.len(),
                    pick.len()
                )));

                return Ok(LevelLineAnswer {
                    condemned: Some(condemned),
                    ..LevelLineAnswer::default()
                });
            }

            if settled.condemned.is_empty() && settled.fits {
                let key = settle_key.clone().unwrap_or_default();
                let settled_room = the_room_to_hand_the_build(settled.last.as_ref());
                self.the_picks_own_reading = (
                    settle_key.clone(),
                    settled
                        .last
                        .as_ref()
                        .map_or(RoomReadingKind::None, |last| last.room_set_by.kind),
                );
                let known_room =
                    self.memo
                        .the_room_under_its_floor(&key, settled_room, Some(trial_size));
                let known_room_set_by = self.memo.the_source_under_its_floor(
                    &key,
                    settled_room,
                    settled
                        .last
                        .as_ref()
                        .map_or_else(RoomSource::nothing, |last| last.room_set_by),
                    Some(trial_size),
                );

                return self.the_level_line_build(
                    &positions,
                    &indices,
                    &pick,
                    &closed,
                    is_feature_edge,
                    profile,
                    fillet_segments,
                    trial_size,
                    known_room,
                    known_room_set_by,
                    cancellation_token,
                );
            }
        }

        // The room is a statement about the part and the pick and not about the size, so it is
        // read ONCE for the whole pick - on the working mesh as it came in - and every later size
        // of the drag re-uses it.
        let slice = self
            .bar
            .slice(0.0, Self::READING_ENDS_IN_THE_PASS, move |phase| {
                BevelProgressPhase::Building { phase, loops: 0 }
            });
        let budget = the_budget_for(indices.len() as i32, cancellation_token, slice);
        let known_room = self.the_room_of_the_pick(
            &positions,
            &indices,
            &pick,
            &closed,
            is_feature_edge.clone(),
            profile,
            trial_size,
            budget,
        )?;

        if let Some(watch) = level_line_routing::watch_room() {
            watch(known_room);
        }

        let key = self.the_memo_key_of(&pick, &closed, profile, THE_ROUTES_SNAP);
        let own_kind = self.memo.the_readings_own_source(&key).kind;
        let known_room_set_by = self.memo.the_room_source(&key, Some(trial_size));
        self.the_picks_own_reading = (Some(key), own_kind);

        self.the_level_line_build(
            &positions,
            &indices,
            &pick,
            &closed,
            is_feature_edge,
            profile,
            fillet_segments,
            trial_size,
            known_room,
            known_room_set_by,
            cancellation_token,
        )
    }

    /// The whole pick as ordered runs of mesh vertex indices, each with whether
    /// it closes on itself and the MESH edges it is made of, or `None` where the pick
    /// links into no run this route can hand over.
    ///
    /// The pick's edges are linked AS ONE SET rather than loop by loop, which is Lars's decision
    /// that two picked edges meeting end to end are one continuous bevel whatever the turn between
    /// them. A closed run's vertices are the vertex each edge is ENTERED at; an OPEN run needs the
    /// last edge's far vertex as well. Three edges is still the floor for a closed run, but an
    /// open run of ONE edge is exactly the owner's single-edge pick and is handed over as it is.
    /// The mesh edges are how a chain the settling condemns is put back onto the rim loops it came
    /// from (`the_loops_holding`).
    pub fn the_chains_of(contributing: &[Rc<RimLoop>]) -> Option<TheChains> {
        let mut chains = Vec::new();
        let mut closed = Vec::new();
        let mut chain_edges = Vec::new();

        let mut edges: Vec<FeatureEdge> = Vec::new();
        let mut already = HashSet::new();

        for one in contributing {
            for edge in &one.edges {
                // Once each: two groups that name the same mesh edge would otherwise make a vertex
                // look like a junction to the linker and cut the run there.
                if already.insert(edge.mesh_edge_index) {
                    edges.push(*edge);
                }
            }
        }

        let linked = bevel_chain_linker::link(&edges);

        if linked.is_empty() {
            return None;
        }

        for chain in &linked {
            if chain.is_closed && chain.edges.len() < 3 {
                return None;
            }

            let mut vertices: Vec<i32> =
                chain.edges.iter().map(|edge| edge.from_vertex()).collect();

            if !chain.is_closed {
                vertices.push(chain.edges[chain.edges.len() - 1].to_vertex());
            }

            chains.push(vertices);
            closed.push(chain.is_closed);
            chain_edges.push(
                chain
                    .edges
                    .iter()
                    .map(|edge| edge.edge.mesh_edge_index)
                    .collect(),
            );
        }

        Some((chains, closed, chain_edges))
    }

    /// Every chain the settling condemned, turned into the mesh edges and
    /// the advisory the pass writes its rows from.
    ///
    /// The advisory is the route's own size advisory. Nothing BUILDS to find the number any more:
    /// it is the reading's, taken DOWN by `the_size_to_offer` so the user who types it is not
    /// refused again.
    pub fn the_condemned_loops(
        settled: &SettledPick,
        chains: &[Vec<i32>],
        chain_edges: &[HashSet<i32>],
        positions: &[Vector3],
        trial_size: f64,
    ) -> Vec<TheChainCondemned> {
        let mut condemned = Vec::new();

        for one in &settled.condemned {
            if one.chain < 0 || one.chain as usize >= chain_edges.len() {
                continue;
            }

            // One number for one answer: the offered one is the number that actually builds, so
            // it is the one both the advisory and the row say.
            let offered = Self::the_size_to_offer(one.room);

            condemned.push(TheChainCondemned {
                mesh_edges: chain_edges[one.chain as usize].clone(),
                reason: BevelAdvisory::new(
                    the_size_refusal_for(&one.because),
                    vec![
                        BevelAdvisoryArg::Double(trial_size),
                        // The sentence stands ON the chain it is about: a rim of many loops
                        // blamed at the pick's first vertex points the owner at whichever loop
                        // happened to come first.
                        BevelAdvisoryArg::Vector3(
                            positions[chains[one.chain as usize][0] as usize],
                        ),
                        BevelAdvisoryArg::Double(offered.unwrap_or(one.room)),
                    ],
                ),
                // The reading's own number is the answer, so no build ladder is run for a
                // condemned loop.
                settled: offered,
            });
        }

        condemned
    }

    /// The whole pick's room, read once per (working mesh, pick, profile)
    /// and kept for every later size. Read under the CALLER'S allowance, so the rebuild's cancel
    /// reaches inside it and it says what it is doing while it runs.
    #[allow(clippy::too_many_arguments)]
    pub fn the_room_of_the_pick(
        &mut self,
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        is_feature_edge: FeatureEdgeTest,
        profile: BandProfile,
        size: f64,
        budget: Rc<WorkBudget>,
    ) -> Result<Option<f64>, LevelLineRouteError> {
        let key = self.the_memo_key_of(chains, closed, profile, THE_ROUTES_SNAP);

        if let Some(remembered) = self.memo.try_the_room(&key, Some(size)) {
            return Ok(remembered);
        }

        // ReadRoom and not RoomOf: RoomOf's convenience overload makes up an allowance of its
        // own, with no token in it.
        let seam_epsilon = self.the_seam_epsilon();
        let reading = band_gap_reading::read_room_of_pick(
            positions,
            indices,
            chains,
            closed,
            Some(is_feature_edge),
            profile,
            THE_ROUTES_SNAP,
            Some(budget),
            f64::NAN,
            seam_epsilon,
            self.the_sharp_turn_degrees,
        )?;
        let room = the_room_to_hand_the_build(Some(&reading));

        let stamp = self.the_stamp();
        self.memo
            .remember_the_room(stamp, &key, room, reading.room_set_by);

        Ok(room)
    }

    /// Write the survivors of a verdict into the memo as a settling of
    /// their OWN, so the pass that comes back with them alone does not settle from scratch what
    /// has just been read. Sound because the settling's last reading IS a reading of these very
    /// chains at this very size, and it named nobody.
    pub fn remember_the_survivors(
        &mut self,
        settled: &SettledPick,
        chains: &[Vec<i32>],
        closed: &[bool],
        profile: BandProfile,
        size: f64,
        room: Option<f64>,
    ) {
        let survivors: Vec<Vec<i32>> = settled
            .chains
            .iter()
            .map(|&chain| chains[chain as usize].clone())
            .collect();
        let survivors_closed: Vec<bool> = settled
            .chains
            .iter()
            .map(|&chain| closed[chain as usize])
            .collect();

        let stamp = self.the_stamp();
        let key = self.the_memo_key_of(&survivors, &survivors_closed, profile, THE_ROUTES_SNAP);
        self.memo.carry_the_settling_to(
            stamp,
            &key,
            size,
            Arc::new(SettledPick {
                chains: (0..survivors.len() as i32).collect(),
                condemned: Vec::new(),
                unresolved: Vec::new(),
                last: settled.last.clone(),
                fits: settled.fits,
                readings: 0,
            }),
            room,
        );
    }
}
