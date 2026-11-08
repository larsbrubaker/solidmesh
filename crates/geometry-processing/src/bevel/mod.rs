//! `MatterCAD.GeometryProcessing/Bevel`: the bevel itself.
//!
//! - **Reporting.** The advisories the kernel reports refusals and warnings with
//!   ([`BevelAdvisory`], [`BevelAdvisoryCode`], [`bevel_advisory_codes`]), what a rebuild has to
//!   say about itself ([`BevelRebuildAdvisories`]) and what each group did on it
//!   ([`BevelGroupResult`], [`bevel_group_failures`]).
//! - **Sections and runs.** The profile and frame an edge's section is drawn in
//!   ([`bevel_edge_tool_builder`]), arc chording ([`bevel_arc_chording`]), a section's winding
//!   ([`bevel_profile_polygon`]), the runs a selection links into ([`BevelSweptChain`],
//!   [`bevel_chain_linker`]) and the sweep that measures them: its rulers
//!   ([`bevel_chain_sweep`]), sections ([`bevel_chain_sweep_sections`]), segments
//!   ([`bevel_chain_sweep_mesh`]), joint rings ([`bevel_chain_sweep_rings`]), the joints one
//!   sweep cannot carry ([`bevel_mitre_fit`]) and how deep its back may reach
//!   ([`bevel_sweep_back_depth`]).
//! - **Ends and corners.** The interrupted-crease readings and the steepest plane an end stops on
//!   ([`bevel_interrupted_crease`]), the equal-angle step a corner samples its ball with
//!   ([`bevel_seam_ring`]) and a corner patch's interior refinement ([`bevel_patch_refinement`]).
//! - **The working mesh.** The working mesh and its verdict cache ([`bevel_working_mesh`]), its
//!   T-vertex and thin sliver repairs ([`bevel_t_vertex_slivers`], [`bevel_thin_slivers`]), a
//!   mesh's lattice and straight-line test ([`bevel_split_chords`]), the float grid step
//!   ([`float_grid`]) and one mesh state prepared for picking ([`BevelPickData`]).
//! - **Constructions.** The level-line build ([`level_line`]), the vertex-centric sections
//!   ([`sections`]) and the surface construction ([`surface`]).
//! - **The part.** A whole part's fillet or chamfer through one entry point,
//!   [`part::bevel_part`] (a request of mesh, picks and settings in; the mesh, a coded row per
//!   group and the advisories out), with its prologue, the surface construction that builds the
//!   part in one pass, Point mode's corners, and what they are orchestrated with: the progress bar
//!   and phases, the pick memo, the settled pick, the touching loops left alone, refusal blame by
//!   place and the self-intersection gate ([`part`]).
//!
//! Stateless helpers are modules of free functions.

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;
pub mod bevel_arc_chording;
pub mod bevel_chain_linker;
pub mod bevel_chain_sweep;
pub mod bevel_chain_sweep_mesh;
pub mod bevel_chain_sweep_rings;
pub mod bevel_chain_sweep_sections;
pub mod bevel_edge_tool_builder;
mod bevel_group_result;
pub mod bevel_interrupted_crease;
pub mod bevel_mitre_fit;
pub mod bevel_patch_refinement;
pub mod bevel_pick_data;
pub mod bevel_profile_polygon;
mod bevel_rebuild_advisories;
pub mod bevel_seam_ring;
pub mod bevel_split_chords;
pub mod bevel_sweep_back_depth;
pub mod bevel_t_vertex_slivers;
pub mod bevel_thin_slivers;
pub mod bevel_working_mesh;
pub mod float_grid;
pub mod level_line;
pub mod part;
pub mod sections;
pub mod surface;

pub use bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
pub use bevel_advisory_code::BevelAdvisoryCode;
pub use bevel_chain_linker::BevelSweptChain;
pub use bevel_group_result::{
    bevel_group_failures, BevelGroupResult, BevelLoopFailure, BevelResultGroup,
};
pub use bevel_pick_data::BevelPickData;
pub use bevel_rebuild_advisories::BevelRebuildAdvisories;
