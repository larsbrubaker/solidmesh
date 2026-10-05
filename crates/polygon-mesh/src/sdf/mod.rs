//! Signed distance fields (agg-sharp `PolygonMesh/Sdf`). This slice ports `GridMarchingCubes`,
//! marching cubes straight over a sampled field, with g3's `DenseGrid3f` it reads. `MeshSdfGrid`
//! needs g3's narrow-band `MeshSignedDistanceGrid` and the Minkowski shell orientation first.

pub mod grid_marching_cubes;
mod marching_cubes_tables;

pub use grid_marching_cubes::{extract, DenseGrid3f};
