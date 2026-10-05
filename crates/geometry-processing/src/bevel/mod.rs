//! `MatterCAD.GeometryProcessing/Bevel`: the bevel itself. Ported so far: the advisories the
//! kernel reports refusals and warnings with ([`BevelAdvisory`], [`BevelAdvisoryCode`],
//! [`bevel_advisory_codes`]).

mod bevel_advisory;
mod bevel_advisory_code;
pub mod bevel_advisory_codes;

pub use bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
pub use bevel_advisory_code::BevelAdvisoryCode;
