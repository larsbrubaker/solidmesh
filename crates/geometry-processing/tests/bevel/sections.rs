//! The `Section*` test classes of `R/Tests/GeometryProcessingTests` (the vertex-centric construction:
//! `SectionAssembly`, `SectionRoom`, `SectionBuilder.TryBuildLoop`/`TryBuildRun` readings, the
//! pierced-face parts, surface walks), one file each, with the Rust-only section tests and the
//! section readers they share. The `bevel.rs` root re-exports every member, so test files in any
//! group reach them as `crate::<module>`.

#[path = "open_mouth_arc_walk.rs"]
pub(crate) mod open_mouth_arc_walk;
#[path = "rust_only_section_smooth_seam_tests.rs"]
pub(crate) mod rust_only_section_smooth_seam_tests;
#[path = "section_assembly_tests.rs"]
pub(crate) mod section_assembly_tests;
#[path = "section_face_rebuild_tests.rs"]
pub(crate) mod section_face_rebuild_tests;
#[path = "section_fan_remesh_tests.rs"]
pub(crate) mod section_fan_remesh_tests;
#[path = "section_hanger_top_rim_tests.rs"]
pub(crate) mod section_hanger_top_rim_tests;
#[path = "section_loft_junction_tests.rs"]
pub(crate) mod section_loft_junction_tests;
#[path = "section_loft_tests.rs"]
pub(crate) mod section_loft_tests;
#[path = "section_loop_entry_tests.rs"]
pub(crate) mod section_loop_entry_tests;
#[path = "section_many_holes_neighbour_tests.rs"]
pub(crate) mod section_many_holes_neighbour_tests;
#[path = "section_many_holes_tests.rs"]
pub(crate) mod section_many_holes_tests;
#[path = "section_open_mouth_arc_cut_tests.rs"]
pub(crate) mod section_open_mouth_arc_cut_tests;
#[path = "section_open_mouth_arc_tests.rs"]
pub(crate) mod section_open_mouth_arc_tests;
#[path = "section_pierced_face_junction_tests.rs"]
pub(crate) mod section_pierced_face_junction_tests;
#[path = "section_robustness_tests.rs"]
pub(crate) mod section_robustness_tests;
#[path = "section_room_split_crease_tests.rs"]
pub(crate) mod section_room_split_crease_tests;
#[path = "section_room_tests.rs"]
pub(crate) mod section_room_tests;
#[path = "section_routing_opt_in_tests.rs"]
pub(crate) mod section_routing_opt_in_tests;
#[path = "section_routing_partition_tests.rs"]
pub(crate) mod section_routing_partition_tests;
#[path = "section_several_runs_tests.rs"]
pub(crate) mod section_several_runs_tests;
#[path = "section_smooth_side_bridged_holes_tests.rs"]
pub(crate) mod section_smooth_side_bridged_holes_tests;
#[path = "section_smooth_side_tests.rs"]
pub(crate) mod section_smooth_side_tests;
#[path = "section_split_chord_tests.rs"]
pub(crate) mod section_split_chord_tests;
#[path = "section_straight_bore_tests.rs"]
pub(crate) mod section_straight_bore_tests;
#[path = "section_surface_walk_arc_join_tests.rs"]
pub(crate) mod section_surface_walk_arc_join_tests;
#[path = "section_surface_walk_fold_tests.rs"]
pub(crate) mod section_surface_walk_fold_tests;
#[path = "section_surface_walk_roof.rs"]
pub(crate) mod section_surface_walk_roof;
#[path = "section_tapered_barrel_mouth_reading_tests.rs"]
pub(crate) mod section_tapered_barrel_mouth_reading_tests;
#[path = "section_tapered_barrel_tests.rs"]
pub(crate) mod section_tapered_barrel_tests;
#[path = "section_walked_end_vertex_tests.rs"]
pub(crate) mod section_walked_end_vertex_tests;
