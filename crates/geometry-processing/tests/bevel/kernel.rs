//! The `Bevel*` test classes of `R/Tests/GeometryProcessingTests` (and the remaining top-level
//! classes and probes, such as `BevelVolumeProbes` and `ManyHolesRimReading`), one file each. The
//! `bevel.rs` root re-exports every member, so test files in any group reach them as
//! `crate::<module>`.

#[path = "bevel_arc_chording_tests.rs"]
pub(crate) mod bevel_arc_chording_tests;
#[path = "bevel_bead_runs_off_its_wall_tests.rs"]
pub(crate) mod bevel_bead_runs_off_its_wall_tests;
#[path = "bevel_chain_linker_rim_closure_tests.rs"]
pub(crate) mod bevel_chain_linker_rim_closure_tests;
#[path = "bevel_folded_flap_repair_tests.rs"]
pub(crate) mod bevel_folded_flap_repair_tests;
#[path = "bevel_gusset_poke_through_tests.rs"]
pub(crate) mod bevel_gusset_poke_through_tests;
#[path = "bevel_knife_edge_rim_tests.rs"]
pub(crate) mod bevel_knife_edge_rim_tests;
#[path = "bevel_mitre_split_tests.rs"]
pub(crate) mod bevel_mitre_split_tests;
#[path = "bevel_patch_refinement_tests.rs"]
pub(crate) mod bevel_patch_refinement_tests;
#[path = "bevel_rebuild_advisories_tests.rs"]
pub(crate) mod bevel_rebuild_advisories_tests;
#[path = "bevel_recess_apex_run_out_tests.rs"]
pub(crate) mod bevel_recess_apex_run_out_tests;
#[path = "bevel_recess_rim_run_off_tests.rs"]
pub(crate) mod bevel_recess_rim_run_off_tests;
#[path = "bevel_sweep_back_depth_tests.rs"]
pub(crate) mod bevel_sweep_back_depth_tests;
#[path = "bevel_t_vertex_sliver_repair_tests.rs"]
pub(crate) mod bevel_t_vertex_sliver_repair_tests;
#[path = "bevel_thin_sliver_conditioning_tests.rs"]
pub(crate) mod bevel_thin_sliver_conditioning_tests;
#[path = "bevel_tool_plane_accuracy_tests.rs"]
pub(crate) mod bevel_tool_plane_accuracy_tests;
#[path = "bevel_volume_probes.rs"]
pub(crate) mod bevel_volume_probes;
#[path = "knife_edge_rim_measures.rs"]
pub(crate) mod knife_edge_rim_measures;
#[path = "many_holes_rim_reading.rs"]
pub(crate) mod many_holes_rim_reading;
