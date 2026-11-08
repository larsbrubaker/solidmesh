// The surface part builder: one PART built by rewriting its surface instead of cutting and gluing
// solids. The struct, its constructor and the shared helpers live here; each concern is its own
// file beside it - `settled_pick`, `level_line_memo`, `rim_loops`, `level_line_kernel_handoff`,
// `level_line_rows`, `level_line` and `level_line_build` (the level-line route, with
// `cove_round_retry`, its offer for a rim that cannot blend at a cove-to-round corner), `size_that_fits`,
// `past_the_level_lines_room`, `touching_loops`, `pass_in_two`, `rim_after_its_partner` (a pass
// built in two where a face rim cannot blend at a cove-to-round corner) and `narrow_the_refusal` (the
// smallest set of loops a refusal that names nobody belongs to). The passes are split by
// responsibility: `build` (the build, the groups built together, the one pass, `say`) and
// `try_together` (one attempt).

pub mod build;
pub mod cove_round_retry;
pub mod level_line;
pub(crate) mod level_line_build;
pub mod level_line_kernel_handoff;
pub mod level_line_memo;
pub(crate) mod level_line_rows;
pub(crate) mod narrow_the_refusal;
#[doc(hidden)]
pub mod no_group_without_a_row;
pub(crate) mod pass_in_two;
pub(crate) mod past_the_level_lines_room;
pub mod rim_after_its_partner;
pub(crate) mod rim_loops;
pub(crate) mod settled_pick;
pub(crate) mod size_that_fits;
pub mod touching_loops;
pub mod try_together;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod narrow_the_refusal_tests;

use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::sync::Arc;

use crate::bevel::level_line::RoomReadingKind;
use crate::bevel::part::refusal_place::the_edge_nearest_the_place_among;
use crate::bevel::part::{BevelPickMemo, BevelProgressBar, BevelProgressReport, BevelRouting};
use crate::bevel::BevelRebuildAdvisories;
use crate::bevel::{BevelAdvisory, BevelGroupResult, BevelLoopFailure};
use crate::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FaceRegionCache, FeatureEdge};
use indexmap::IndexMap;
use solidmesh_math::Color;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

pub use build::SurfacePartBuildError;
pub use level_line::{LevelLineAnswer, LevelLineRouteError, TheChainCondemned};
pub use rim_loops::RimLoop;
pub use try_together::PassAttempt;

/// The surface part builder for one part.
///
/// The other half of the part's prologue (`prologue`): handed the work the two constructions share - the
/// welded working mesh, the resolved groups, which of them fit - at the moment the boolean path
/// would start building tools, and it answers in the same `BevelPartOutcome` shape. Every chain of
/// every group that fits is planned against the ORIGINAL working mesh and built in ONE pass;
/// anything the plan, the walkback or the strip refuses comes back as the advisory it refused
/// with, and the part publishes no mesh at all. It never reaches the boolean kernel.
pub struct BevelSurfacePartBuilder {
    groups: Vec<BevelEdgeGroup>,

    resolutions: Vec<BevelEdgeGroupResolution>,

    /// Which groups resolved to edges, which is everything this construction is offered. Whether
    /// there is ROOM for them is its own business and no longer the boolean fit's.
    offered: Vec<bool>,

    /// Shared with the caller and a second pass, never written here.
    advisories: Rc<BevelRebuildAdvisories>,

    /// The welded working mesh, in the space the size is quoted in. It is not modified.
    working_mesh: Arc<Mesh>,

    working_mesh_graph: MeshEdgeGraph,

    /// The fillet radius or chamfer setback.
    size: f64,

    /// The arc samples per quarter turn.
    segments: i32,

    round_profile: bool,

    /// The translation held back from the working mesh, which the result is carried out by - the
    /// same matrix the boolean path hands the kernel as its operand matrix.
    item_matrix: Matrix4X4,

    source_color: Color,

    /// One region reading per tolerance, shared across the attempts of one build. Keyed by the
    /// tolerance's bits (-0 folded onto 0, so the two zeros share one cache).
    region_caches: HashMap<u64, Arc<FaceRegionCache>>,

    /// One group's picked edges cut into rim loops, made once per group (`rim_loops`).
    rim_loops: HashMap<i32, Vec<Rc<RimLoop>>>,

    /// How long the size searches of ONE rebuild have taken between them, which is what stops a
    /// part with dozens of refused loops paying a bisection for every one of them.
    size_search_ms_spent: i64,

    /// How long one size search may take (`size_that_fits`), the operation's own.
    size_search_budget_ms: i32,

    /// Which constructions this builder may take: the request's own ([`Self::set_routing`]),
    /// MatterCAD's routes unless it says otherwise.
    routing: BevelRouting,

    /// The faces of `working_mesh` no region flood may enter, or `None` where there are none: the
    /// blend an EARLIER pass laid on this same mesh (`pass_in_two`).
    barrier_faces: Option<Vec<i32>>,

    /// Which faces of the mesh the last successful build returned are the blend it laid - the
    /// fence the next pass over that mesh is given. Empty from the legacy construction, which does
    /// not mark its own faces yet.
    blend_faces_of_the_last_build: Vec<i32>,

    /// The bar in front of the user: every kernel call of this rebuild reports into a slice of
    /// it, so it moves forward once across the whole rebuild instead of restarting at each call.
    bar: BevelProgressBar,

    /// What the OPERATION already knows about this pick that the size does not change - the
    /// reason a drag measures the room once and not at every step. Never absent; see `new`.
    memo: Arc<BevelPickMemo>,

    // .SettledPick
    /// Set for the rest of a rebuild where the settling named loops the pass could not write rows
    /// for, so the same loops go round again down yesterday's path rather than the pass ending in
    /// silence. Set by the one pass (`build`).
    the_settling_is_off: bool,

    // .SizeThatFits
    /// How many rungs the search running now has tried, for the words under the bar.
    rungs_tried: i32,

    /// Whether this builder's refusals offer no size at all - set on the SECOND pass of a pass
    /// built in two, whose builds stand on the first pass's output at the size asked. Typing a
    /// smaller size changes the first pass too, so a size found on its mesh was never built the
    /// way the user would get it. Level line only: the frozen constructions answer what they did.
    offers_no_size: bool,

    // .LevelLineMemo
    /// The working mesh's content stamp, read once per builder, as the memo keys' mesh half.
    the_mesh_stamp: Option<u64>,

    // .LevelLine.KernelHandoff
    the_seam_epsilon: Option<f64>,

    /// The operation's bend threshold, read with its features; no mitre until it is read (set by
    /// the level line, `level_line`).
    the_sharp_turn_degrees: f64,

    // .LevelLineRows
    /// The one open chain of the last level-line build whose ends were closed by end-face caps.
    the_end_capped_chain: Option<Vec<i32>>,

    /// The mesh edges of each chain of the last level-line pass, in the order handed to the
    /// kernel.
    the_chain_edges_of_the_pass: Option<Vec<std::collections::HashSet<i32>>>,

    /// The chain of the last level-line pass a size refusal named, or -1 where none was named.
    the_chain_the_refusal_names: i32,

    // .PastTheLevelLinesRoom
    /// The memo key of the pick the level line last read, and which reading named that pick's OWN
    /// room - never a floor's source (`None` key: no pick read yet).
    the_picks_own_reading: (Option<String>, RoomReadingKind),

    /// Whether the level line's last size refusal was a PINCH - found by its build at the size
    /// asked, or handed a floor a pinch set that holds at this size
    /// (`BevelPickMemo::the_pinched_chain_under_the_floor`).
    the_refusal_is_a_pinch: bool,

    /// The (pick, size) attempts the existing construction has already failed this rebuild - the
    /// pick's memo key (`None`: no key) and the size's bits.
    the_legacy_attempts_that_failed: HashSet<(Option<String>, u64)>,

    /// Set while `the_existing_construction_past_the_level_lines_room` asks the pass again.
    the_level_line_is_set_aside: bool,

    /// Set while the size search's `builds` asks, whose clock is `the_search_clock`.
    the_ladder_is_asking: bool,

    /// The running size search's clock, so a legacy rung is not started past its budget.
    the_search_clock: Option<web_time::Instant>,

    // .TouchingLoops
    /// Whether the level line's last band refused the pick because two of its chains touch
    /// (`ChainsTouch`), which is what `the_touching_loops_left_alone` answers before the pass is
    /// handed to the older construction.
    the_band_refused_a_touch: bool,
    /// Whether the last level-line build refused because a face rim's rail ran backward next to
    /// one of its cove-to-round corners (`rim_after_its_partner`).
    the_rim_refused_at_its_flips: bool,
    /// Whether any level-line build of the pass so far did ([`Self::the_rim_refused_at_its_flips`]).
    a_rim_refused_at_its_flips_in_the_pass: bool,

    /// This pass's loops left alone for touching, each with the loop it touched that stayed and the
    /// sentence it was given - so it is put back once that loop is dropped itself. In the order
    /// they were left alone (insertion-ordered, because the putting back iterates it).
    left_alone_for_touching: IndexMap<Rc<RimLoop>, touching_loops::LeftAlone>,

    /// The loops already put back once in this rebuild, which bounds the putting back.
    put_back_once: HashSet<Rc<RimLoop>>,
}

impl BevelSurfacePartBuilder {
    /// A builder for one part. `barrier_faces`, `reporter` and `memo` are optional (`None` where
    /// the caller has none); `reporter` is told the bar's phases, which the caller words.
    ///
    /// A builder with no memo handed to it gets one of its own, which is a memo that can only ever
    /// be written and never hit: the builder is constructed fresh per rebuild. That is exactly the
    /// behaviour of the route before the memo existed, and it is what a caller who is not a bevel
    /// operation - a test, the second pass of a two-pass build - should get unless it says
    /// otherwise.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        groups: Vec<BevelEdgeGroup>,
        resolutions: Vec<BevelEdgeGroupResolution>,
        offered: Vec<bool>,
        advisories: Rc<BevelRebuildAdvisories>,
        working_mesh: Arc<Mesh>,
        working_mesh_graph: MeshEdgeGraph,
        size: f64,
        segments: i32,
        round_profile: bool,
        item_matrix: Matrix4X4,
        source_color: Color,
        barrier_faces: Option<Vec<i32>>,
        reporter: Option<BevelProgressReport>,
        memo: Option<Arc<BevelPickMemo>>,
    ) -> Self {
        BevelSurfacePartBuilder {
            memo: memo.unwrap_or_default(),
            bar: BevelProgressBar::new(reporter),
            barrier_faces: barrier_faces.filter(|faces| !faces.is_empty()),
            groups,
            resolutions,
            offered,
            advisories,
            working_mesh,
            working_mesh_graph,
            size,
            segments,
            round_profile,
            item_matrix,
            source_color,
            region_caches: HashMap::new(),
            rim_loops: HashMap::new(),
            size_search_ms_spent: 0,
            size_search_budget_ms: Self::DEFAULT_SIZE_SEARCH_BUDGET_MS,
            routing: BevelRouting::default(),
            blend_faces_of_the_last_build: Vec::new(),
            the_settling_is_off: false,
            rungs_tried: 0,
            offers_no_size: false,
            the_mesh_stamp: None,
            the_seam_epsilon: None,
            the_sharp_turn_degrees: f64::INFINITY,
            the_end_capped_chain: None,
            the_chain_edges_of_the_pass: None,
            the_chain_the_refusal_names: -1,
            the_picks_own_reading: (None, RoomReadingKind::None),
            the_refusal_is_a_pinch: false,
            the_legacy_attempts_that_failed: HashSet::new(),
            the_level_line_is_set_aside: false,
            the_ladder_is_asking: false,
            the_search_clock: None,
            the_band_refused_a_touch: false,
            the_rim_refused_at_its_flips: false,
            a_rim_refused_at_its_flips_in_the_pass: false,
            left_alone_for_touching: IndexMap::new(),
            put_back_once: HashSet::new(),
        }
    }

    /// Which constructions this builder may take.
    pub fn routing(&self) -> BevelRouting {
        self.routing
    }

    /// Sets [`Self::routing`] for this builder (and the second builders it makes for a pass built
    /// in two).
    pub fn set_routing(&mut self, routing: BevelRouting) {
        self.routing = routing;
    }

    /// The memo this builder reads and writes - the operation's own, or the builder's private one.
    pub fn memo(&self) -> &Arc<BevelPickMemo> {
        &self.memo
    }

    /// The loops that were refused written onto the rows that
    /// BUILT, which is the one thing a row can have to say having built: part of what was picked
    /// is not there.
    ///
    /// Only onto a row that built. A row every loop of which was refused already carries that
    /// refusal as its own sentence, and saying it twice is one problem said twice.
    pub fn write_what_the_loops_came_to(
        results: &mut [Option<BevelGroupResult>],
        refused_loops: &indexmap::IndexMap<i32, Vec<BevelLoopFailure>>,
    ) {
        for (&group, loops) in refused_loops {
            let Some(row) = results[group as usize].as_mut() else {
                continue;
            };

            if row.failed() {
                continue;
            }

            row.loop_failures = Some(loops.clone());
        }
    }

    /// The picked mesh edge nearest the place a refusal says it
    /// happened, where it says one at all - among the CONTRIBUTING groups' edges. A fallback and
    /// not a rule: the edge's own stamp is exact and this is a nearest guess, so where both exist
    /// the stamp wins. `None` where there is no clear winner.
    pub fn the_edge_nearest_the_place(
        resolutions: &[BevelEdgeGroupResolution],
        contributing: &[i32],
        reason: Option<&BevelAdvisory>,
        clear_by: f64,
    ) -> Option<i32> {
        let units: Vec<&[FeatureEdge]> = contributing
            .iter()
            .map(|&index| resolutions[index as usize].edges.as_slice())
            .collect();

        the_edge_nearest_the_place_among(&units, reason, clear_by)
    }

    /// The region cache for one tolerance, made once. The retry can come round
    /// with a different group first and so with a different curve tolerance, and re-flooding the
    /// whole mesh for a reading an earlier turn already made would be the price of scoping a
    /// refusal.
    pub(super) fn regions_at(&mut self, tolerance_degrees: f64) -> Arc<FaceRegionCache> {
        // -0 and 0 are the same tolerance, so they share one key.
        let key = if tolerance_degrees == 0.0 {
            0.0f64.to_bits()
        } else {
            tolerance_degrees.to_bits()
        };

        if let Some(cache) = self.region_caches.get(&key) {
            return cache.clone();
        }

        // Fenced where this builder is the SECOND half of a pass built in two: the faces the first
        // half's fillet laid are barriers no flood enters, and the edge to one reads as a rim
        // edge. Without the fence a region picked on that mesh runs over the blend.
        let cache = Arc::new(FaceRegionCache::with_barriers(
            self.working_mesh.clone(),
            Some(self.working_mesh_graph.clone()),
            tolerance_degrees,
            self.barrier_faces.clone(),
        ));
        self.region_caches.insert(key, cache.clone());

        cache
    }
}
