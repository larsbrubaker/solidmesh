//! `MatterCAD.GeometryProcessing/Bevel`: the bevel itself. Ported so far: the advisories the
//! kernel reports refusals and warnings with ([`BevelAdvisory`], [`BevelAdvisoryCode`],
//! [`bevel_advisory_codes`]), and the first leaf cluster of the core: arc chording
//! ([`bevel_arc_chording`]), the construction mode ([`BevelConstructionMode`]), the corner face
//! ring ([`bevel_corner_face_ring`]), the second pass's crease merge ([`bevel_crease_merge`])
//! and the end cap plane ([`BevelEndCap`]); and the second: the cross-section ear clip
//! ([`bevel_profile_polygon`]), the selected ends at a corner ([`BevelMixedCornerEnds`]), the
//! crease between two runs ([`bevel_pair_corner_crease`]), the partner's ball tube
//! ([`BevelPartnerTube`]) and the analytic round ([`BevelRoundDescriptor`]). C# `internal static
//! class` helpers are modules of free functions here.

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;
pub mod bevel_arc_chording;
mod bevel_construction_mode;
pub mod bevel_corner_face_ring;
pub mod bevel_crease_merge;
mod bevel_end_cap;
mod bevel_mixed_corner_ends;
pub mod bevel_pair_corner_crease;
mod bevel_partner_tube;
pub mod bevel_profile_polygon;
mod bevel_round_descriptor;

pub use bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
pub use bevel_advisory_code::BevelAdvisoryCode;
pub use bevel_construction_mode::BevelConstructionMode;
pub use bevel_corner_face_ring::CornerFaceRing;
pub use bevel_end_cap::BevelEndCap;
pub use bevel_mixed_corner_ends::BevelMixedCornerEnds;
pub use bevel_pair_corner_crease::PairFaces;
pub use bevel_partner_tube::BevelPartnerTube;
pub use bevel_round_descriptor::{BevelRoundDescriptor, RoundStation};
