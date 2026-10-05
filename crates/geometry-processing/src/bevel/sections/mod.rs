//! `MatterCAD.GeometryProcessing/Bevel/Sections`: the section construction. Ported so far: its
//! append-only point pool ([`SectionPool`]) and the walk of a side's fan at a chain vertex
//! ([`section_fan_walk`]), and the type definitions of the construction: the section at a chain
//! vertex ([`BevelSection`]) and at a junction ([`JunctionSection`]), the absorbed split points
//! ([`SectionAbsorbedSplits`]), the room cache ([`SectionRoomCache`]), one chain's run of
//! sections ([`SectionRun`]) and the surface walk's result ([`SurfaceWalkResult`]). The construction itself lands with the cyclic bevel core.

pub mod bevel_section;
pub mod section_absorbed_splits;
pub mod section_builder;
pub mod section_builder_walked_side;
pub mod section_corner;
pub mod section_end;
pub mod section_fan_walk;
pub mod section_junction;
pub mod section_junction_canal;
mod section_pool;
pub mod section_room;
pub mod section_room_smooth_side;
pub mod section_run;
pub mod section_smooth_side_boundary;
pub mod section_split_blend;
pub mod section_surface_walk;
pub mod section_surface_walk_faces;
pub mod section_surface_walk_join;
pub mod section_surface_walk_try;
pub mod section_walk_vertex_turn;

pub use bevel_section::{BevelSection, SectionKind, SectionSide};
pub use section_absorbed_splits::SectionAbsorbedSplits;
pub use section_junction::{JunctionPiece, JunctionSection};
pub use section_pool::{NanSectionPoint, SectionPool};
pub use section_room::SectionRoomCache;
pub use section_run::{IsAFeatureEdge, SectionRun};
pub use section_surface_walk::{SurfaceWalkCrossing, SurfaceWalkRefusal, SurfaceWalkResult};
