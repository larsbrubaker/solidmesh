// What the level-line route hands the kernel and takes back from it - the mesh's seam epsilon
// and bend threshold, the room worth trusting, the triangles a band kept, and the allowance and
// bar phases for a call. The route itself (`level_line`) calls these; the kernel's answer becomes
// a `Mesh` through `crate::bevel::part::answer_mesh`, and the bar is the kernel's
// `BevelProgressBar`.

use std::rc::Rc;

use crate::bevel::bevel_working_mesh;
use crate::bevel::level_line::level_line_routing;
use crate::bevel::level_line::{
    LevelLineMesh, LevelLineProgressReport, LevelLineWorkPhase, RoomReading, RoomReadingKind,
    RoomSource, WorkBudget,
};
use crate::bevel::part::{BevelProgress, BevelProgressBar, BevelProgressPhase};
use crate::bevel::{BevelAdvisoryCode, BevelLoopFailure};
use indexmap::IndexMap;
use solidmesh_mesh::CancelToken;

use super::BevelSurfacePartBuilder;

/// The triangles the band did not take, in the working mesh's own vertex
/// numbering.
pub fn the_kept_triangles(mesh: &LevelLineMesh) -> Vec<(i32, i32, i32)> {
    let mut kept = Vec::with_capacity(mesh.triangle_count().max(0) as usize);

    for i in 0..mesh.triangle_count() {
        if mesh.is_removed(i) {
            continue;
        }

        let triangle = mesh.triangles[i as usize];
        kept.push((triangle.a, triangle.b, triangle.c));
    }

    kept
}

/// Which room, if any, this reading is worth handing the build - the
/// number when the reading is a statement about SIZE, and nothing at all when it is not.
///
/// The build TRUSTS a room it is handed and refuses a size above it at once. A reading that is
/// not about size still carries a room of ZERO, and handing that zero in would turn every one of
/// those into "your bevel is too big", where the route used to fall THROUGH to the legacy
/// construction, which may well build. Nothing is `None` and not NaN: NaN would sail past the
/// build's comparison unrefused.
pub fn the_room_to_hand_the_build(reading: Option<&RoomReading>) -> Option<f64> {
    reading
        .filter(|reading| reading.is_a_size_statement())
        .map(|reading| reading.room)
}

/// Which size refusal a room names - two bevels' fronts meeting across a
/// face where two chains of the pick closed on each other, a growing neighbour's far crease, and
/// one bevel reaching the far side of a face too narrow for it otherwise. Only a chain-to-chain
/// reading has a second bevel in it; telling the user about "the bevel coming the other way"
/// elsewhere sends them looking for a bevel that is not there.
pub fn the_size_refusal_for(set_by: &RoomSource) -> BevelAdvisoryCode {
    // A growing neighbour's far crease is not the face's width: it stands off the face, where
    // another face meets the one the band grows over, and the sentence stands on it (CreaseAt).
    if set_by.kind == RoomReadingKind::ChainToChain {
        BevelAdvisoryCode::SizeCollidesAcrossFace
    } else if set_by.is_a_growing_crease() {
        BevelAdvisoryCode::SizeReachesPastCrease
    } else {
        BevelAdvisoryCode::SizeExceedsFaceWidth
    }
}

/// Where to stand the size advisory - a vertex of the chain the room's
/// reading named, and the pick's first vertex where it named none. A pick of several chains reads
/// its room off ONE of them, and an advisory standing on the outer rim when the hole is what ran
/// out of room points the owner at the wrong feature.
pub fn the_vertex_to_blame(chains: &[Vec<i32>], set_by: &RoomSource) -> i32 {
    let chain = set_by.chain;

    if chain >= 0 && (chain as usize) < chains.len() && !chains[chain as usize].is_empty() {
        chains[chain as usize][0]
    } else {
        chains[0][0]
    }
}

/// An undirected edge as its (low, high) vertex pair.
pub fn the_edge_key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// The allowance one kernel call gets, carrying the rebuild's own
/// cancellation so the stop button reaches INSIDE the call, and the bar's report so a build that
/// takes seconds says where it is. Handed to `LevelLineRouting.WatchBudget` where a test watches.
pub fn the_budget_for(
    triangle_count: i32,
    cancellation_token: &CancelToken,
    tell_the_bar: LevelLineProgressReport,
) -> Rc<WorkBudget> {
    let mut budget = WorkBudget::with_cancellation(
        WorkBudget::default_limit(triangle_count),
        cancellation_token.clone(),
        Some(tell_the_bar),
    )
    .expect("the default limit is a positive count of work");

    // The app's parts are stored as Vector3Float, so the kernel reads their vertices at a float's
    // precision.
    budget.input_is_float_stored = true;
    let budget = Rc::new(budget);

    if let Some(watch) = level_line_routing::watch_budget() {
        watch(&budget);
    }

    budget
}

impl BevelSurfacePartBuilder {
    /// Where the passes that build the part end on the bar; publishing takes the rest.
    pub const PASSES_END_ON_THE_BAR: f64 = 0.97;

    /// How much of what is left of the passes' span one attempt takes. Half, because nothing knows
    /// up front whether an attempt builds or is the first of many retries (Many Holes: the LAST
    /// attempt, built chain by chain, was 77% of the rebuild).
    pub const PASS_SHARE_OF_WHAT_IS_LEFT: f64 = 0.5;

    /// How much of an attempt its kernel calls report into; a size search after it takes the rest.
    pub const KERNEL_PART_OF_AN_ATTEMPT: f64 = 0.7;

    /// Where pass A of a build in two ends, leaving pass B the rest of the passes' span.
    pub const PASS_A_END_ON_THE_BAR: f64 = 0.5;

    /// How much of what is left of an attempt's tail one size-search rung takes: a search is up to
    /// eight rungs and nobody knows up front how many will run.
    pub const RUNG_SHARE_OF_WHAT_IS_LEFT: f64 = 0.25;

    /// Where the room reading (or the settling) ends within a pass's window of the bar.
    pub const READING_ENDS_IN_THE_PASS: f64 = 0.2;

    /// Where the pick-wide cut ends within a pass's window; the fill, or the chains in turn, take
    /// the rest.
    pub const BUILD_ENDS_IN_THE_PASS: f64 = 0.75;

    /// How far a feature edge may stand from a picked chain and still be that
    /// chain's own seam rather than a wall - the mesh's own number. NOT in the memo's key: the
    /// epsilon is a function of the working mesh alone, and the key already carries its stamp.
    pub fn the_seam_epsilon(&mut self) -> f64 {
        *self
            .the_seam_epsilon
            .get_or_insert_with(|| bevel_working_mesh::seam_epsilon_for(&self.working_mesh))
    }

    /// The operation's bend threshold as the level line read it with its features (infinity, no
    /// mitre, until it is read).
    pub fn the_sharp_turn_degrees(&self) -> f64 {
        self.the_sharp_turn_degrees
    }

    /// The allowances for the chains built in turn - a cut and a fill per
    /// chain, each the next of 2N equal slices of the pass's fill span, so the bar walks across
    /// the chains rather than restarting per chain. The returned maker takes a call's triangle
    /// count.
    pub fn the_in_turn_budgets(
        &self,
        chain_count: i32,
        cancellation_token: &CancelToken,
    ) -> impl FnMut(i32) -> Rc<WorkBudget> {
        let calls = (2 * chain_count).max(1);
        let mut made = 0;
        let bar = self.bar.clone();
        let cancellation_token = cancellation_token.clone();

        move |count| {
            let k = made.min(calls - 1);
            made += 1;
            let span = 1.0 - Self::BUILD_ENDS_IN_THE_PASS;
            let slice = bar.slice(
                Self::BUILD_ENDS_IN_THE_PASS + (span * k as f64 / calls as f64),
                Self::BUILD_ENDS_IN_THE_PASS + (span * (k + 1) as f64 / calls as f64),
                move |_| BevelProgressPhase::ShapingLoop {
                    loop_number: (k / 2) + 1,
                    loops: chain_count,
                },
            );

            the_budget_for(count, &cancellation_token, slice)
        }
    }

    /// The phase a build of `loops` loops (0 where the count is not the point) is in, as the bar
    /// reports it now ([`BevelProgressBar::said`]).
    pub fn the_progress_for(&self, phase: LevelLineWorkPhase, loops: i32) -> BevelProgress {
        self.bar.said(BevelProgressPhase::Building { phase, loops })
    }

    /// The bar this rebuild reports into.
    pub fn bar(&self) -> &BevelProgressBar {
        &self.bar
    }

    /// Opens one pass attempt on the bar, carrying how many loops earlier
    /// attempts left out so every phase of the retry says so.
    pub fn open_an_attempt(
        &self,
        refused_loops: &IndexMap<i32, Vec<BevelLoopFailure>>,
        passes_end: f64,
    ) {
        let left_out: usize = refused_loops.values().map(Vec::len).sum();

        self.bar.open_an_attempt(
            passes_end,
            Self::PASS_SHARE_OF_WHAT_IS_LEFT,
            Self::KERNEL_PART_OF_AN_ATTEMPT,
            left_out as i32,
        );
    }
}
