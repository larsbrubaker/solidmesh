//! `MatterCAD.GeometryProcessing/Bevel/Sections`: the section construction. Ported so far: its
//! append-only point pool ([`SectionPool`]). The construction itself lands with the cyclic bevel
//! core.

mod section_pool;

pub use section_pool::{NanSectionPoint, SectionPool};
