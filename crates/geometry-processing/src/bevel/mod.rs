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
//! region face grid ([`surface`]). C# `internal static class` helpers are modules of free
//! functions here.

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;
pub mod bevel_arc_chording;
pub mod bevel_blend_seam;
mod bevel_construction_mode;
pub mod bevel_corner_face_ring;
pub mod bevel_corner_seam;
pub mod bevel_crease_merge;
mod bevel_end_cap;
mod bevel_mixed_corner_ends;
pub mod bevel_pair_corner_crease;
mod bevel_partner_tube;
pub mod bevel_profile_polygon;
mod bevel_round_descriptor;
mod bevel_round_faces;
pub mod bevel_seam_ring;
pub mod bevel_shared_edges;
mod bevel_surface_labels;
mod bevel_surface_walk;
pub mod float_grid;
pub mod level_line;
pub mod sections;
pub mod surface;

pub use bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
pub use bevel_advisory_code::BevelAdvisoryCode;
pub use bevel_construction_mode::BevelConstructionMode;
pub use bevel_corner_face_ring::CornerFaceRing;
pub use bevel_end_cap::BevelEndCap;
pub use bevel_mixed_corner_ends::BevelMixedCornerEnds;
pub use bevel_pair_corner_crease::PairFaces;
pub use bevel_partner_tube::BevelPartnerTube;
pub use bevel_round_descriptor::{BevelRoundDescriptor, RoundStation};
pub use bevel_round_faces::BevelRoundFaces;
pub use bevel_surface_labels::BevelSurfaceLabels;
pub use bevel_surface_walk::{BevelSurfaceWalk, SurfaceWalkStop};
