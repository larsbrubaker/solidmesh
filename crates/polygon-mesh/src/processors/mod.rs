//! Mesh processors (agg-sharp `PolygonMesh/Processors`). This slice ports STL reading and
//! writing (`StlProcessing.cs`, split into `stl_processing` for the reader and `stl_writer`)
//! and `MeshOutputSettings`, plus the face BSP tree (`FaceBspTree.cs`) and the convex hull (`MeshConvexHull.cs`, on a port of
//! MIConvexHull in `mi_convex_hull`) and `planar_region_simplifier`
//! (`PlanarRegionSimplifier.cs`, on manifold-rust's ear clipper); the rest of the folder joins in
//! later Phase 2 slices.

pub mod face_bsp_tree;
pub mod mesh_convex_hull;
pub mod mesh_output_settings;
mod mi_convex_hull;
pub mod planar_region_simplifier;
pub mod stl_processing;
pub mod stl_writer;
pub mod vertex_source_to_mesh;

pub use face_bsp_tree::BspNode;
pub use mesh_convex_hull::ConvexHull;
pub use mesh_output_settings::{CsgOption, MeshOutputSettings, OutputType};
pub use stl_processing::{Progress, StlChunk, StlError, StlParse, FACES_PER_YIELD};
