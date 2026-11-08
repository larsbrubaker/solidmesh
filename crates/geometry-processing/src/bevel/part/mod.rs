//! The pieces a whole PART's bevel is orchestrated with, above the constructions: what a rebuild
//! reports as it goes ([`BevelProgress`], `progress`) through its one forward-only bar
//! ([`BevelProgressBar`], `progress_bar`); what an operation remembers about its picks across
//! rebuilds ([`BevelPickMemo`], `pick_memo`) and the settled many-loop pick it keeps
//! ([`SettledPick`], `settled_pick`); which touching closed loops of a many-loop pick to leave
//! alone (`touching_loops`); which picked edge a refusal's place blames (`refusal_place`); and
//! the gate every level-line answer passes, whether it crosses itself where its source did not
//! (`self_intersection_check`, over the answer as a mesh, `answer_mesh`).

pub mod answer_mesh;
pub mod pick_memo;
mod progress;
pub mod progress_bar;
pub mod refusal_place;
pub mod self_intersection_check;
mod settled_pick;
pub mod touching_loops;

pub use pick_memo::BevelPickMemo;
pub use progress::{BevelProgress, BevelProgressPhase, BevelProgressReport, BevelSizeSearchRung};
pub use progress_bar::BevelProgressBar;
pub use settled_pick::SettledPick;

/// Below this many loops a pick is the owner's corner pick, not a many-loop rim, and keeps the
/// route it has: it is neither settled in one reading nor has touching loops left alone.
pub const THE_MANY_LOOP_RIM_FLOOR: usize = 4;
