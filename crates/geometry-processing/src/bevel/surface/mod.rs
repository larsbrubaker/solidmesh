//! `MatterCAD.GeometryProcessing/Bevel/Surface`: the surface bevel construction. Ported so far:
//! its leaf records - a band end's half space ([`SurfaceBandEnd`]), a partial run-off
//! ([`SurfaceBandRunsOffOverAStretch`]), which edges own a strip ([`SurfaceEdgeRoles`]) and the
//! plan's infinite line ([`SurfaceLine`]). The construction itself lands in later slices.

mod surface_band_end;
mod surface_band_runs_off_over_a_stretch;
mod surface_edge_roles;
mod surface_line;

pub use surface_band_end::SurfaceBandEnd;
pub use surface_band_runs_off_over_a_stretch::SurfaceBandRunsOffOverAStretch;
pub use surface_edge_roles::SurfaceEdgeRoles;
pub use surface_line::SurfaceLine;
