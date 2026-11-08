//! The bevel part tests (`part_*`): the pieces a whole part's bevel is orchestrated with
//! (`geometry_processing::bevel::part`), one file each. They share no helpers with the other
//! groups, so `bevel.rs` does not re-export this one.

#[path = "part_pick_memo_tests.rs"]
pub(crate) mod part_pick_memo_tests;
#[path = "part_progress_bar_tests.rs"]
pub(crate) mod part_progress_bar_tests;
#[path = "part_refusal_place_tests.rs"]
pub(crate) mod part_refusal_place_tests;
#[path = "part_self_intersection_check_tests.rs"]
pub(crate) mod part_self_intersection_check_tests;
#[path = "part_touching_loops_tests.rs"]
pub(crate) mod part_touching_loops_tests;
