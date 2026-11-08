//! A whole PART's bevel, above the constructions, and its one entry point: [`bevel_part`]
//! (`entry_point`), asked a [`BevelPartRequest`] (one part's mesh, where it sits, its picks and the
//! operation's settings, `request`, with the constructions it may take, [`BevelRouting`],
//! `routing`) and answering a [`BevelPartOutcome`] (the mesh or none, a row
//! per group and the advisories, as codes, `outcome`) or a
//! [`BevelPartError`] (a cancel, or a build that could not run: an opaque [`BevelBuildFailure`],
//! `error`).
//!
//! Behind it: the part's prologue (`prologue`: the welded working copy carried into the
//! operation's space and the groups resolved on it), the surface construction that builds every
//! fitting chain in one pass (`surface_part_builder`) and Point mode's corner picks
//! (`point_groups`). The pieces they are orchestrated with: what a rebuild reports as it goes
//! ([`BevelProgress`], `progress`) through its one forward-only bar ([`BevelProgressBar`],
//! `progress_bar`); what an operation remembers about its picks across rebuilds ([`BevelPickMemo`],
//! `pick_memo`) and the settled many-loop pick it keeps ([`SettledPick`], `settled_pick`); which
//! touching closed loops of a many-loop pick to leave alone (`touching_loops`); which picked edge
//! a refusal's place blames (`refusal_place`); the size a refused row offers, as a row shows it
//! ([`the_shown_size`], `shown_size`); and the gate every level-line answer passes, whether it
//! crosses itself where its source did not (`self_intersection_check`, over the answer as a mesh,
//! `answer_mesh`).

pub mod answer_mesh;
mod entry_point;
mod error;
mod outcome;
pub mod pick_memo;
pub mod point_groups;
mod progress;
pub(crate) mod progress_bar;
pub mod prologue;
pub mod refusal_place;
mod request;
mod routing;
pub mod self_intersection_check;
mod settled_pick;
mod shown_size;
pub mod surface_part_builder;
pub mod touching_loops;

pub use entry_point::bevel_part;
pub use error::{BevelBuildFailure, BevelBuildFailureKind, BevelPartError};
pub use outcome::BevelPartOutcome;
pub use pick_memo::BevelPickMemo;
pub use progress::{BevelProgress, BevelProgressPhase, BevelProgressReport, BevelSizeSearchRung};
pub use progress_bar::BevelProgressBar;
pub use request::BevelPartRequest;
pub use routing::BevelRouting;
pub use settled_pick::SettledPick;
pub use shown_size::the_shown_size;

/// Below this many loops a pick is the owner's corner pick, not a many-loop rim, and keeps the
/// route it has: it is neither settled in one reading nor has touching loops left alone.
pub const THE_MANY_LOOP_RIM_FLOOR: usize = 4;
