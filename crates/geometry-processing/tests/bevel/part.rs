//! The bevel part tests (`part_*`): a whole part's bevel (`solidmesh::bevel::part`) -
//! the pieces it is orchestrated with, one file each beside this one, and its entry point, its
//! prologue and its surface construction, in `part/`. Nothing else uses their helpers, so
//! `bevel.rs` does not re-export this group; the fixtures they read are the shared ones.

#[path = "part/part_bevel_part_tests.rs"]
pub(crate) mod part_bevel_part_tests;
#[path = "part/part_bevel_platform_bits_tests.rs"]
pub(crate) mod part_bevel_platform_bits_tests;
#[path = "part/part_bevel_routing_tests.rs"]
pub(crate) mod part_bevel_routing_tests;
#[path = "part/part_cove_round_next_flip_tests.rs"]
pub(crate) mod part_cove_round_next_flip_tests;
#[path = "part/part_level_line_known_room_tests.rs"]
pub(crate) mod part_level_line_known_room_tests;
#[path = "part/part_level_line_settle_tests.rs"]
pub(crate) mod part_level_line_settle_tests;
#[path = "part/part_no_group_without_a_row_tests.rs"]
pub(crate) mod part_no_group_without_a_row_tests;
#[path = "part/part_partial_round_tests.rs"]
pub(crate) mod part_partial_round_tests;
#[path = "part_pick_memo_tests.rs"]
pub(crate) mod part_pick_memo_tests;
#[path = "part_progress_bar_tests.rs"]
pub(crate) mod part_progress_bar_tests;
#[path = "part/part_prologue_tests.rs"]
pub(crate) mod part_prologue_tests;
#[path = "part_refusal_place_tests.rs"]
pub(crate) mod part_refusal_place_tests;
#[path = "part/part_replay_tests.rs"]
pub(crate) mod part_replay_tests;
#[path = "part_self_intersection_check_tests.rs"]
pub(crate) mod part_self_intersection_check_tests;
#[path = "part/part_surface_part_builder_s3_tests.rs"]
pub(crate) mod part_surface_part_builder_s3_tests;
#[path = "part/part_surface_part_builder_tests.rs"]
pub(crate) mod part_surface_part_builder_tests;
#[path = "part_touching_loops_tests.rs"]
pub(crate) mod part_touching_loops_tests;
