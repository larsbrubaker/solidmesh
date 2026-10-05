//! `MatterCAD.GeometryProcessing/Bevel/Surface`: the surface bevel construction. Ported so far:
//! its leaf records - a band end's half space ([`SurfaceBandEnd`]), a partial run-off
//! ([`SurfaceBandRunsOffOverAStretch`]), which edges own a strip ([`SurfaceEdgeRoles`]) and the
//! plan's infinite line ([`SurfaceLine`]) - and the pieces every stage of it shares: the welding
//! vertex pool ([`SurfaceVertexPool`]), the closure gate the answer is refused by
//! ([`surface_mesh_closure`]), the step laid at a mitre joint ([`surface_mitre_joint_step`]) and
//! the reading of a feature standing on a region ([`surface_room_standing_feature`]), the rolling
//! ball's spine round a mixed corner ([`SurfaceMixedCornerSpine`]) and the per-region face grid
//! the wall-clearance questions are asked of ([`SurfaceRegionFaceGrid`]). The construction itself
//! lands in later slices.

mod surface_band_end;
mod surface_band_runs_off_over_a_stretch;
mod surface_edge_roles;
mod surface_line;
pub mod surface_mesh_closure;
pub mod surface_mitre_joint_step;
mod surface_mixed_corner_spine;
mod surface_region_face_grid;
pub mod surface_room_standing_feature;
mod surface_vertex_pool;

pub use surface_band_end::SurfaceBandEnd;
pub use surface_band_runs_off_over_a_stretch::SurfaceBandRunsOffOverAStretch;
pub use surface_edge_roles::SurfaceEdgeRoles;
pub use surface_line::SurfaceLine;
pub use surface_mixed_corner_spine::{ContactArms, SurfaceMixedCornerSpine};
pub use surface_region_face_grid::{nearest_on_the_triangle, SurfaceRegionFaceGrid};
pub use surface_vertex_pool::SurfaceVertexPool;
