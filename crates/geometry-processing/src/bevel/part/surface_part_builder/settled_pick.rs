// A pick of many loops settled in a few readings -
// ONE read of who cannot be built at the size, and what is left is what builds. The kernel names
// everybody a size condemns in one go (`PickThatFits`); this asks it, maps the verdict back onto
// the pick and onto the pass's loops. The answer is the kernel's `SettledPick`, which the
// operation's memo (`crate::bevel::part::pick_memo`) keeps; the level-line route
// asks for it; `say_what_the_settling_named` writes its verdict onto the rows
// through `Say` (`build`).

use std::collections::HashSet;

use crate::bevel::level_line::band_gap_reading::ReadRoomError;
use crate::bevel::level_line::pick_that_fits::PickThatFits;
use crate::bevel::level_line::pick_validation;
use crate::bevel::level_line::seam_edges::FeatureEdgeTest;
use crate::bevel::level_line::{
    BandProfile, ChainCannotFit, RoomReadingKind, RoomSource, WorkBudget,
};
use crate::bevel::part::{self, SettledPick};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelLoopFailure};
use indexmap::IndexMap;
use solidmesh_math::Vector3;
use solidmesh_mesh::CancelToken;
use std::rc::Rc;

use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError, TheChainCondemned};

impl BevelSurfacePartBuilder {
    /// Where the line is drawn: a pick of this many chains or more is a many-loop RIM and is
    /// settled, and anything smaller goes on exactly as it does today.
    ///
    /// Below the floor is where the stage-3 corner lives - two or three picked edges meeting at
    /// one vertex - and that pick must keep its present answer: the kernel refuses it by name and
    /// the pass falls through to the old construction, which builds it. A rim's loops share no
    /// vertex unless something is wrong.
    pub const THE_MANY_LOOP_RIM_FLOOR: i32 = part::THE_MANY_LOOP_RIM_FLOOR as i32;

    /// Settles the pick at `size` - asks the kernel, in ONE call that reads
    /// and re-reads until a reading names nobody, who cannot be built at that size - or answers
    /// `Ok(None)` where this pick is not the settling's to judge and must take yesterday's path
    /// untouched.
    ///
    /// `budget` is the caller's allowance, SHARED across the readings so a pick that cannot be
    /// settled inside it comes back unsettled rather than spending without limit; `None` asks the
    /// kernel for its own default. `seam_epsilon` is the part's own (`the_seam_epsilon`), which
    /// the settling must read at because its verdict is memoised and handed to the build as a
    /// known room (0 by default); `sharp_turn_degrees` is infinity where unread.
    /// The kernel's argument check failing is the error.
    #[allow(clippy::too_many_arguments)]
    pub fn the_settled_pick(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        is_feature_edge: Option<FeatureEdgeTest>,
        profile: BandProfile,
        size: f64,
        budget: Option<Rc<WorkBudget>>,
        seam_epsilon: f64,
        sharp_turn_degrees: f64,
    ) -> Result<Option<SettledPick>, ReadRoomError> {
        // A pick where two chains TOUCH is not this step's to settle. The kernel refuses a touch
        // that is not a corner by name (ChainsTouch) and the pass falls through to the legacy
        // construction, which builds touching loops. A corner pick is chains that touch too: it is
        // built whole by the pass that follows. A settling that condemned one of every touching
        // pair would take a bevel away from a user who had it yesterday.
        if !pick_validation::every_touching_pair(chains).is_empty() {
            return Ok(None);
        }

        let alive: Vec<i32> = (0..chains.len() as i32).collect();
        let alive_chains: Vec<Vec<i32>> = alive
            .iter()
            .map(|&chain| chains[chain as usize].clone())
            .collect();
        let alive_closed: Vec<bool> = alive.iter().map(|&chain| closed[chain as usize]).collect();

        let fit = PickThatFits::read(
            positions,
            indices,
            &alive_chains,
            &alive_closed,
            is_feature_edge,
            profile,
            size,
            1e-9,
            budget,
            seam_epsilon,
            sharp_turn_degrees,
        )?;

        let mut condemned: Vec<ChainCannotFit> = fit
            .dropped
            .iter()
            .map(|one| {
                // The kernel answered about the pick it was GIVEN, so every chain number in its
                // answer is a position in that list and means nothing here until looked up.
                ChainCannotFit {
                    chain: alive[one.chain as usize],
                    because: Self::in_terms_of_the_whole_pick(one.because, &alive),
                    room: one.room,
                    round: one.round,
                }
            })
            .collect();

        // A stable sort, so chains named twice keep the order they were named in.
        condemned.sort_by_key(|one| one.chain);

        Ok(Some(SettledPick {
            chains: fit
                .chains
                .iter()
                .map(|&chain| alive[chain as usize])
                .collect(),
            condemned,
            unresolved: fit
                .unresolved
                .iter()
                .map(|group| group.iter().map(|&chain| alive[chain as usize]).collect())
                .collect(),
            last: fit.last,
            fits: fit.fits,
            readings: fit.rounds,
        }))
    }

    /// The settling's verdict written onto the rows - every loop a
    /// condemned chain stands for refused with that chain's own sentence and the size it was read
    /// at, and all of them taken out of the pass at once. `false` (nothing written, nothing taken
    /// out) where the verdict names nobody still in the pass, or everybody.
    pub(super) fn say_what_the_settling_named(
        &mut self,
        contributing: &mut Vec<Rc<RimLoop>>,
        condemned: &[TheChainCondemned],
        results: &mut [Option<BevelGroupResult>],
        refused_loops: &mut IndexMap<i32, Vec<BevelLoopFailure>>,
        cancellation_token: &CancelToken,
    ) -> Result<bool, SurfacePartBuildError> {
        let mut going: Vec<Rc<RimLoop>> = Vec::new();
        let mut said: Vec<(Vec<Rc<RimLoop>>, BevelAdvisory, Option<f64>)> = Vec::new();

        for one in condemned {
            let blamed: Vec<Rc<RimLoop>> = Self::the_loops_holding(contributing, &one.mesh_edges)
                .into_iter()
                .filter(|candidate| !going.contains(candidate))
                .collect();

            if blamed.is_empty() {
                continue;
            }

            going.extend(blamed.iter().cloned());
            said.push((blamed, one.reason.clone(), one.settled));
        }

        if going.is_empty() || going.len() == contributing.len() {
            return Ok(false);
        }

        for (loops, reason, settled) in said {
            // Said against the pass as it stands BEFORE any of them is dropped, so a group that
            // keeps a loop gets a loop's sentence under a row that builds, and one that keeps none
            // is refused as a group. A loop left alone for TOUCHING another is not about the size,
            // so it is not put through the size diagnosis.
            let about_the_size = reason.code() != BevelAdvisoryCode::LoopTouchesAnotherPickedLoop;
            self.say(
                &loops,
                contributing,
                results,
                refused_loops,
                &reason,
                about_the_size,
                cancellation_token,
                settled,
                true,
            )?;
        }

        contributing.retain(|one| !going.contains(one));

        Ok(true)
    }

    /// Which of the pass's LOOPS a settling's condemned chains
    /// stand for, or `None` where the verdict does not map onto the user's loops at all.
    ///
    /// A chain is not a loop: one loop can split into several runs, and one chain can be linked
    /// across two loops that meet end to end, so a verdict about chains can name HALF of a loop -
    /// and half a loop is not something the pass can drop or the user can be told about. The
    /// verdict is used only when it is clean: every loop wholly condemned or wholly surviving,
    /// somebody condemned, somebody left. `loop_edges` are the mesh edges of each loop still in
    /// the pass, `condemned_edges` those of each condemned chain; the answer indexes `loop_edges`.
    pub fn the_loops_the_settling_condemns(
        loop_edges: &[HashSet<i32>],
        condemned_edges: &[HashSet<i32>],
    ) -> Option<Vec<i32>> {
        let mut condemned_loops = Vec::new();

        let named: HashSet<i32> = condemned_edges
            .iter()
            .flat_map(|chain| chain.iter().copied())
            .collect();

        for (index, edges) in loop_edges.iter().enumerate() {
            let held = edges.iter().filter(|edge| named.contains(edge)).count();

            if held == 0 {
                continue;
            }

            if held != edges.len() {
                return None;
            }

            condemned_loops.push(index as i32);
        }

        if condemned_loops.is_empty() || condemned_loops.len() == loop_edges.len() {
            // Nobody to drop, or everybody - the second is an answer about the whole pick, which
            // yesterday's path says better.
            return None;
        }

        Some(condemned_loops)
    }

    /// A reason the kernel stated in the numbering of the pick it was
    /// given, put back into the whole pick's - the same translation `PickThatFits` does between
    /// its own rounds.
    pub fn in_terms_of_the_whole_pick(local: RoomSource, alive: &[i32]) -> RoomSource {
        let original = |chain: i32| -> i32 {
            if chain >= 0 && (chain as usize) < alive.len() {
                alive[chain as usize]
            } else {
                -1
            }
        };

        match local.kind {
            // Only the chain changes; every other member is kept, the crease included.
            RoomReadingKind::Foreign => RoomSource {
                chain: original(local.chain),
                ..local
            },
            RoomReadingKind::SelfRoom => RoomSource::self_room(original(local.chain)),
            RoomReadingKind::ChainToChain => {
                RoomSource::chain_to_chain(original(local.chain), original(local.other_chain))
            }
            _ => RoomSource::nothing(),
        }
    }
}
