//! `MatterCAD.GeometryProcessing/Bevel/Sections`: the vertex-centric section construction - its
//! append-only point pool ([`SectionPool`]), the walk of a side's fan at a chain vertex
//! ([`section_fan_walk`]), the section at a chain vertex ([`BevelSection`]), at a corner, at an
//! open end and at a junction ([`JunctionSection`]), the builder of one chain's run of sections
//! ([`SectionRun`]), the room reading ([`SectionRoomCache`]), the surface walk
//! ([`SurfaceWalkResult`]) and the router between the legacy construction and the sections
//! ([`section_routing`]), the cut of a run's smooth side ([`section_smooth_side`]), with the region re-mesh ([`section_fan_remesh`]) and the mixed
//! corner's reading ([`section_mixed_corner`]).

pub mod bevel_section;
pub mod section_absorbed_splits;
pub mod section_builder;
pub mod section_builder_offsets;
pub mod section_builder_run;
pub mod section_builder_station;
pub mod section_builder_walked_side;
pub mod section_corner;
pub mod section_end;
pub mod section_fan_remesh;
pub mod section_fan_walk;
pub mod section_junction;
pub mod section_junction_build;
pub mod section_junction_canal;
pub mod section_mixed_corner;
mod section_pool;
pub mod section_room;
pub mod section_room_smooth_side;
pub mod section_routing;
pub mod section_routing_clauses;
pub mod section_run;
pub mod section_smooth_side;
pub mod section_smooth_side_boundary;
pub mod section_smooth_side_facets;
pub mod section_smooth_side_holes;
pub mod section_smooth_side_junction_rim;
pub mod section_smooth_side_triangulate;
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
