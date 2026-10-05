//! `MatterCAD.GeometryProcessing/Bevel/Sections`: the section construction. Ported so far: its
//! append-only point pool ([`SectionPool`]) and the walk of a side's fan at a chain vertex
//! ([`section_fan_walk`]). The construction itself lands with the cyclic bevel core.

pub mod section_fan_walk;
mod section_pool;

pub use section_pool::{NanSectionPoint, SectionPool};
