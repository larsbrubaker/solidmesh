//! `MatterCAD.GeometryProcessing/Bevel`: the bevel itself.
//!
//! - **Reporting.** The advisories the kernel reports refusals and warnings with
//!   ([`BevelAdvisory`], [`BevelAdvisoryCode`], [`bevel_advisory_codes`]), what a rebuild has to
//!   say about itself ([`BevelRebuildAdvisories`]) and what each group did on it
//!   ([`BevelGroupResult`], [`BevelRebuildResults`], [`bevel_group_failures`]), and the rebuild's
//!   boolean budget ([`bevel_rebuild_step_budget`]).
//! - **Tools and sweeps.** The profile and
//!   frame an edge's tool is drawn in ([`bevel_edge_tool_builder`]), arc chording
//!   ([`bevel_arc_chording`]), the cross-section ear clip ([`bevel_profile_polygon`]), the
//!   analytic round ([`BevelRoundDescriptor`]), a run's taper ([`BevelRunTaper`]), one edge to one
//!   tool ([`bevel_shared_edges`]), adjacent selections joined into shared sweeps
//!   ([`bevel_sweep_batches`]), each run's tool and what the boolean needs ([`BevelTool`]), and
//!   the sweep's chains, sections and segments ([`BevelSweptChain`], [`BevelSweptSection`],
//!   [`bevel_chain_sweep_mesh`]).
//! - **Run ends.** The end cap plane ([`BevelEndCap`]), a run's end treatment
//!   ([`BevelChainEnds`]), what one end can afford ([`bevel_chain_end_budget`]), the corner
//!   handover read first ([`bevel_corner_handover`]) and the seams an end stops on: a blend's
//!   handover ([`bevel_blend_seam`]) and a corner patch's drive-through and cap stand-off
//!   ([`bevel_corner_seam`]).
//! - **Corners.** The corner face ring ([`bevel_corner_face_ring`]), the shared seam ring
//!   ([`bevel_seam_ring`]), the corner patches ([`BevelCornerPatch`], [`BevelCornerPatchSet`])
//!   with their shell and void cone ([`bevel_corner_patch_mesher`]) and interior refinement
//!   ([`bevel_patch_refinement`]), the void each 5.3 corner opens ([`BevelCornerVoids`]); the
//!   mixed corner's selected ends ([`BevelMixedCornerEnds`]), blend records ([`BevelMixedBlend`],
//!   [`BevelMixedBlendSet`]), blend tool mesh ([`bevel_mixed_blend_tool`]) and crowding warning
//!   ([`bevel_mixed_corners`]); and the pair corner's crease ([`bevel_pair_corner_crease`]),
//!   partner tube ([`BevelPartnerTube`]), reach ([`bevel_pair_corner_reach`]) and trim
//!   ([`bevel_pair_corner_trim`]).
//! - **Beads and rounds.** The bead batch and clip ([`BevelBeadBatch`], [`bevel_bead_clip`]),
//!   the cap clip ([`bevel_cap_clip`]), the partnered bead's run, ends and stations
//!   ([`bevel_partnered_bead`]), which faces lie on a round ([`BevelRoundFaces`]), the rounds a
//!   part arrives with ([`bevel_source_rounds`]) and the cut where a crease's ball enters a round
//!   ([`bevel_round_entry`]).
//! - **Fitting and cleaning.** The face-width check ([`BevelFaceFit`]), the run-out and its
//!   bound ([`BevelFaceRunOut`], [`BevelRunOutBound`]), the interrupted-crease readings
//!   ([`bevel_interrupted_crease`]), the second pass's crease merge ([`bevel_crease_merge`]) and
//!   run plan ([`BevelSecondPassRunPlan`]) and the pass itself, re-beading the concave creases on
//!   the cut part ([`bevel_second_pass`]), the shadow clips' obstructions and apply step
//!   ([`bevel_shadow_clip`]), the T-vertex repair
//!   ([`bevel_t_vertex_slivers`]), the sliver recut ([`bevel_sliver_collapse`]), the published
//!   mesh's cleaning ([`bevel_published_mesh`]), the working mesh's verdict cache
//!   ([`bevel_working_mesh`]) and one mesh state prepared for picking ([`BevelPickData`]).
//! - **Shared readings.** The per-tolerance surface labels ([`BevelSurfaceLabels`]), the
//!   straight surface walk ([`BevelSurfaceWalk`]) and the float grid step ([`float_grid`]).
//! - **Constructions.** The level-line build ([`level_line`]), the vertex-centric sections
//!   ([`sections`]) and the surface construction ([`surface`]).
//!
//! Stateless helpers are modules of free functions.

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;
pub mod bevel_arc_chording;
pub mod bevel_bead_batch;
pub mod bevel_bead_clip;
pub mod bevel_bead_corner_pieces;
pub mod bevel_bead_exit;
pub mod bevel_blend_seam;
pub mod bevel_cap_clip;
pub mod bevel_chain_end_budget;
pub mod bevel_chain_end_cap;
pub mod bevel_chain_end_plane;
pub mod bevel_chain_linker;
pub mod bevel_chain_sweep;
pub mod bevel_chain_sweep_mesh;
pub mod bevel_chain_sweep_rings;
pub mod bevel_chain_sweep_sections;
pub mod bevel_corner_face_ring;
pub mod bevel_corner_handover;
pub mod bevel_corner_patch;
pub mod bevel_corner_patch_mesher;
pub mod bevel_corner_seam;
mod bevel_corner_voids;
pub mod bevel_crease_merge;
pub mod bevel_edge_tool_builder;
mod bevel_end_cap;
pub mod bevel_face_fit;
pub mod bevel_face_fit_check;
pub mod bevel_face_fit_exemptions;
pub mod bevel_face_fit_walls;
pub mod bevel_face_run_out;
mod bevel_group_result;
pub mod bevel_group_tools;
pub mod bevel_interrupted_crease;
pub mod bevel_mitre_fit;
mod bevel_mixed_blend;
pub mod bevel_mixed_blend_tool;
pub mod bevel_mixed_blends;
mod bevel_mixed_corner_ends;
pub mod bevel_mixed_corners;
pub mod bevel_overshoot_rings;
pub mod bevel_pair_corner_crease;
pub mod bevel_pair_corner_reach;
pub mod bevel_pair_corner_trim;
mod bevel_partner_tube;
pub mod bevel_partner_tube_clip;
pub mod bevel_partnered_bead;
pub mod bevel_partnered_bead_partners;
pub mod bevel_partnered_bead_section;
pub mod bevel_partnered_bead_stations;
pub mod bevel_partnered_bead_tolerances;
pub mod bevel_patch_refinement;
pub mod bevel_pick_data;
pub mod bevel_profile_polygon;
pub mod bevel_published_mesh;
mod bevel_rebuild_advisories;
pub mod bevel_rebuild_step_budget;
mod bevel_round_descriptor;
pub mod bevel_round_entry;
mod bevel_round_faces;
pub mod bevel_run_partners;
pub mod bevel_run_taper;
pub mod bevel_seam_ring;
pub mod bevel_second_pass;
pub mod bevel_second_pass_run_plan;
pub mod bevel_shadow_body;
pub mod bevel_shadow_clip;
pub mod bevel_shared_edges;
pub mod bevel_sliver_collapse;
pub mod bevel_source_hull;
pub mod bevel_source_rounds;
mod bevel_source_rounds_fit;
pub mod bevel_split_chords;
pub mod bevel_straight_run;
mod bevel_surface_labels;
mod bevel_surface_walk;
pub mod bevel_sweep_back_depth;
pub mod bevel_sweep_batches;
pub mod bevel_sweep_fold_check;
pub mod bevel_t_vertex_slivers;
pub mod bevel_tapered_sweep;
pub mod bevel_thin_slivers;
mod bevel_tool;
pub mod bevel_valley_fold_mitre;
pub mod bevel_working_mesh;
pub mod float_grid;
pub mod level_line;
pub mod sections;
pub mod surface;

pub use bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
pub use bevel_advisory_code::BevelAdvisoryCode;
pub use bevel_bead_batch::BevelBeadBatch;
pub use bevel_chain_end_cap::BevelChainEnds;
pub use bevel_chain_linker::BevelSweptChain;
pub use bevel_chain_sweep::BevelSweptSection;
pub use bevel_corner_face_ring::CornerFaceRing;
pub use bevel_corner_patch::{BevelCornerPatch, BevelCornerPatchSet};
pub use bevel_corner_voids::BevelCornerVoids;
pub use bevel_end_cap::BevelEndCap;
pub use bevel_face_fit::{BevelFaceFit, BevelFaceFitEndCaps};
pub use bevel_face_run_out::{BevelFaceRunOut, BevelRunOutBound, BevelRunOutMeasure};
pub use bevel_group_result::{
    bevel_group_failures, BevelGroupResult, BevelLoopFailure, BevelRebuildResults, BevelResultGroup,
};
pub use bevel_mixed_blend::{BevelMixedBlend, BevelMixedBlendSet};
pub use bevel_mixed_corner_ends::BevelMixedCornerEnds;
pub use bevel_pair_corner_crease::PairFaces;
pub use bevel_partner_tube::BevelPartnerTube;
pub use bevel_pick_data::BevelPickData;
pub use bevel_rebuild_advisories::BevelRebuildAdvisories;
pub use bevel_round_descriptor::{BevelRoundDescriptor, RoundStation};
pub use bevel_round_faces::BevelRoundFaces;
pub use bevel_run_taper::BevelRunTaper;
pub use bevel_second_pass::{BevelSecondPassRebead, BevelSecondPassRun};
pub use bevel_second_pass_run_plan::BevelSecondPassRunPlan;
pub use bevel_surface_labels::BevelSurfaceLabels;
pub use bevel_surface_walk::{BevelSurfaceWalk, SurfaceWalkStop};
pub use bevel_sweep_batches::BevelSweepBatch;
pub use bevel_tool::BevelTool;
