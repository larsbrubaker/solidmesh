//! `MatterCAD.GeometryProcessing/Bevel`: the bevel itself. Ported so far: the advisories the
//! kernel reports refusals and warnings with ([`BevelAdvisory`], [`BevelAdvisoryCode`],
//! [`bevel_advisory_codes`]), and the first leaf cluster of the core: arc chording
//! ([`bevel_arc_chording`]), the construction mode ([`BevelConstructionMode`]), the corner face
//! ring ([`bevel_corner_face_ring`]), the second pass's crease merge ([`bevel_crease_merge`])
//! and the end cap plane ([`BevelEndCap`]). C# `internal static class` helpers are modules of
//! free functions here.

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;
pub mod bevel_arc_chording;
mod bevel_construction_mode;
pub mod bevel_corner_face_ring;
pub mod bevel_crease_merge;
mod bevel_end_cap;

pub use bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
pub use bevel_advisory_code::BevelAdvisoryCode;
pub use bevel_construction_mode::BevelConstructionMode;
pub use bevel_corner_face_ring::CornerFaceRing;
pub use bevel_end_cap::BevelEndCap;
