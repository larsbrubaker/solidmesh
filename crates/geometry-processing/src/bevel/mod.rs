//! `MatterCAD.GeometryProcessing/Bevel`: the bevel itself. Ported so far: the advisories the
//! kernel reports refusals and warnings with ([`BevelAdvisory`], [`BevelAdvisoryCode`],
//! [`bevel_advisory_codes`]), and the first leaf cluster of the core: arc chording
//! ([`bevel_arc_chording`]), the construction mode ([`BevelConstructionMode`]), the corner face
//! ring ([`bevel_corner_face_ring`]), the second pass's crease merge ([`bevel_crease_merge`])
//! and the end cap plane ([`BevelEndCap`]); and the second: the cross-section ear clip
//! ([`bevel_profile_polygon`]), the selected ends at a corner ([`BevelMixedCornerEnds`]), the
//! crease between two runs ([`bevel_pair_corner_crease`]), the partner's ball tube
//! ([`BevelPartnerTube`]) and the analytic round ([`BevelRoundDescriptor`]); and the third: the
//! shared seam ring ([`bevel_seam_ring`]), one-edge-one-tool ([`bevel_shared_edges`]), the
//! per-tolerance surface labels ([`BevelSurfaceLabels`]), the straight surface walk
//! ([`BevelSurfaceWalk`]), the float grid step ([`float_grid`]) and the level line's leaf records
//! ([`level_line`]); and the fourth: more level-line leaves (field bounds and samples, the field
//! traits, the segment box tree, gap loops and fill refusals), the section point pool
//! ([`sections`]) and the surface construction's leaf records ([`surface`]); and the fifth: the
//! seams a run's end stops on - a blend's handover ([`bevel_blend_seam`]) and a corner patch's
//! drive-through and cap stand-off ([`bevel_corner_seam`]) - with the surface construction's
//! vertex pool, closure gate, joint step and standing-feature reading ([`surface`]); and the
//! sixth: which faces lie on a round ([`BevelRoundFaces`]), the convex cell bound, the flat
//! end-face fill and the room's source ([`level_line`]), and the mixed corner's spine and the
//! region face grid ([`surface`]); and the seventh: the profile and frame an edge's tool is drawn
//! in ([`bevel_edge_tool_builder`]), the corner handover a run's end reads first
//! ([`bevel_corner_handover`]), the mixed corner's blend records ([`BevelMixedBlend`],
//! [`BevelMixedBlendSet`]) and the ring tidying after a clip ([`surface`]); and the eighth: what a rebuild has to
//! say about itself ([`BevelRebuildAdvisories`]), the cut where a crease's ball enters a round
//! ([`bevel_round_entry`]), the rounds a part arrives with ([`bevel_source_rounds`]) and the walk
//! of a side's fan at a chain vertex ([`sections`]); and the ninth: what one end of a run can
//! afford ([`bevel_chain_end_budget`]), what each group did on a rebuild ([`BevelGroupResult`],
//! [`BevelRebuildResults`], [`bevel_group_failures`]) and the clip, run-off readings and planless
//! loop of the surface construction ([`surface`]); and the tenth: the mixed corner's blend tool
//! mesh ([`bevel_mixed_blend_tool`]) and how far a tube runs to reach its partner's axis at a pair
//! corner ([`bevel_pair_corner_reach`]). The cyclic core's type definitions land ahead of its
//! methods: the bead batch and clip ([`BevelBeadBatch`], [`bevel_bead_clip`]), the cap clip
//! ([`bevel_cap_clip`]), a run's end treatment ([`BevelChainEnds`]), the sweep's chains, sections
//! and segments ([`BevelSweptChain`], [`BevelSweptSection`], [`bevel_chain_sweep_mesh`]), the
//! corner patches ([`BevelCornerPatch`], [`BevelCornerPatchSet`]), the face-width check
//! ([`BevelFaceFit`]), the run-out and its bound ([`BevelFaceRunOut`], [`BevelRunOutBound`]), the
//! partnered bead's run, ends and stations ([`bevel_partnered_bead`]), the interrupted-crease readings ([`bevel_interrupted_crease`]), the
//! shadow clip's obstructions ([`bevel_shadow_clip`]), the T-vertex repair
//! ([`bevel_t_vertex_slivers`]) and the working mesh's verdict cache ([`bevel_working_mesh`]).
//! C# `internal static class` helpers are modules of free functions here.

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;
pub mod bevel_arc_chording;
pub mod bevel_bead_batch;
pub mod bevel_bead_clip;
pub mod bevel_blend_seam;
pub mod bevel_cap_clip;
pub mod bevel_chain_end_budget;
pub mod bevel_chain_end_cap;
pub mod bevel_chain_linker;
pub mod bevel_chain_sweep;
pub mod bevel_chain_sweep_mesh;
mod bevel_construction_mode;
pub mod bevel_corner_face_ring;
pub mod bevel_corner_handover;
pub mod bevel_corner_patch;
pub mod bevel_corner_seam;
pub mod bevel_crease_merge;
pub mod bevel_edge_tool_builder;
mod bevel_end_cap;
pub mod bevel_face_fit;
pub mod bevel_face_run_out;
mod bevel_group_result;
pub mod bevel_interrupted_crease;
mod bevel_mixed_blend;
pub mod bevel_mixed_blend_tool;
mod bevel_mixed_corner_ends;
pub mod bevel_pair_corner_crease;
pub mod bevel_pair_corner_reach;
mod bevel_partner_tube;
pub mod bevel_partnered_bead;
pub mod bevel_profile_polygon;
mod bevel_rebuild_advisories;
mod bevel_round_descriptor;
pub mod bevel_round_entry;
mod bevel_round_faces;
pub mod bevel_seam_ring;
pub mod bevel_shadow_clip;
pub mod bevel_shared_edges;
pub mod bevel_source_rounds;
mod bevel_source_rounds_fit;
mod bevel_surface_labels;
mod bevel_surface_walk;
pub mod bevel_t_vertex_slivers;
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
pub use bevel_construction_mode::BevelConstructionMode;
pub use bevel_corner_face_ring::CornerFaceRing;
pub use bevel_corner_patch::{BevelCornerPatch, BevelCornerPatchSet};
pub use bevel_end_cap::BevelEndCap;
pub use bevel_face_fit::{BevelFaceFit, BevelFaceFitEndCaps};
pub use bevel_face_run_out::{BevelFaceRunOut, BevelRunOutBound};
pub use bevel_group_result::{
    bevel_group_failures, BevelGroupResult, BevelLoopFailure, BevelRebuildResults, BevelResultGroup,
};
pub use bevel_mixed_blend::{BevelMixedBlend, BevelMixedBlendSet};
pub use bevel_mixed_corner_ends::BevelMixedCornerEnds;
pub use bevel_pair_corner_crease::PairFaces;
pub use bevel_partner_tube::BevelPartnerTube;
pub use bevel_rebuild_advisories::BevelRebuildAdvisories;
pub use bevel_round_descriptor::{BevelRoundDescriptor, RoundStation};
pub use bevel_round_faces::BevelRoundFaces;
pub use bevel_surface_labels::BevelSurfaceLabels;
pub use bevel_surface_walk::{BevelSurfaceWalk, SurfaceWalkStop};
