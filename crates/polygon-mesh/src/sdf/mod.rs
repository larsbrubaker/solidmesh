//! Signed distance fields: `grid_marching_cubes`, marching cubes straight over a sampled field,
//! with the `DenseGrid3f` it reads, and `MeshSdfGrid`, a solid's narrow-band signed distance field
//! and the offset surfaces pulled from it, with the `DistanceGridBuilder` and
//! `AsyncDistanceGridBuilder` hooks a GPU fill is injected through (the second for a host that can
//! only await the fill).

pub mod distance_grid_builder;
pub mod grid_marching_cubes;
pub(crate) mod marching_cubes_tables;
pub mod mesh_sdf_grid;

pub use distance_grid_builder::{
    AsyncDistanceGridBuilder, DistanceBuildError, DistanceFillFuture, DistanceGridBuilder,
    DistanceGridRequest, OwnedDistanceGridRequest,
};
pub use grid_marching_cubes::{extract, DenseGrid3f};
pub use mesh_sdf_grid::{FloodProgress, MeshSdfGrid, PreparedDistanceGrid, SdfGridError};
