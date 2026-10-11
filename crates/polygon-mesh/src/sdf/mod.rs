//! Signed distance fields (agg-sharp `PolygonMesh/Sdf`): `GridMarchingCubes`, marching cubes
//! straight over a sampled field, with g3's `DenseGrid3f` it reads, and `MeshSdfGrid`, a solid's
//! narrow-band signed distance field and the offset surfaces pulled from it, with the
//! `DistanceGridBuilder` hook a GPU fill is injected through.

pub mod distance_grid_builder;
pub mod grid_marching_cubes;
pub(crate) mod marching_cubes_tables;
pub mod mesh_sdf_grid;

pub use distance_grid_builder::{DistanceBuildError, DistanceGridBuilder, DistanceGridRequest};
pub use grid_marching_cubes::{extract, DenseGrid3f};
pub use mesh_sdf_grid::{FloodProgress, MeshSdfGrid, SdfGridError};
