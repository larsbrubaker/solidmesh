//! What a bevel rebuild tells its caller as it goes ([`BevelProgress`]): the phase it is in
//! ([`BevelProgressPhase`]), how many loops an earlier attempt left out, and the size-search rung
//! open now ([`BevelSizeSearchRung`]). Only numbers and phases: the caller words them in its own
//! language. The bar (`progress_bar`) reports them through a [`BevelProgressReport`].

use std::sync::Arc;

use crate::bevel::level_line::work_budget::LevelLineWorkPhase;

/// What the rebuild is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BevelProgressPhase {
    /// Settling which loops of a many-loop pick can take the size.
    CheckingWhichLoopsFit {
        /// The loops in the pick.
        loops: i32,
    },
    /// A level-line phase of a build over `loops` loops (0 where the count is not the point).
    Building {
        /// The level line's phase.
        phase: LevelLineWorkPhase,
        /// The loops built together.
        loops: i32,
    },
    /// A pick built a loop at a time: loop `loop_number` (from 1) of `loops`.
    ShapingLoop {
        /// Which loop, counted from 1.
        loop_number: i32,
        /// How many loops are built in turn.
        loops: i32,
    },
    /// The self-intersection and closure checks and the publish that follow a build.
    CheckingTheResult,
}

/// One rung of the size search for loops that did not fit at the size asked for: try `attempt`
/// of at most `attempts`, for `loops` loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BevelSizeSearchRung {
    /// The loops the rung searches a size for.
    pub loops: i32,
    /// Which try this is, counted from 1.
    pub attempt: i32,
    /// The most tries the search makes.
    pub attempts: i32,
}

/// One report: what the rebuild is doing, and what the attempt open now adds to it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BevelProgress {
    /// What the rebuild is doing.
    pub doing: BevelProgressPhase,
    /// How many loops earlier attempts left out because they did not fit (0 on a first attempt).
    pub leaving_out: i32,
    /// The size-search rung open now, which stands in for `doing` while it is open: a rung's own
    /// build is the same work as a pass's, and saying so would read as the rebuild starting over.
    pub smaller_size: Option<BevelSizeSearchRung>,
}

impl BevelProgress {
    /// `doing` with nothing added.
    pub fn plain(doing: BevelProgressPhase) -> Self {
        BevelProgress {
            doing,
            leaving_out: 0,
            smaller_size: None,
        }
    }
}

/// Where a rebuild's reports go: how far it is (0..1, never going down) and what it is doing.
pub type BevelProgressReport = Arc<dyn Fn(f64, &BevelProgress) + Send + Sync>;
