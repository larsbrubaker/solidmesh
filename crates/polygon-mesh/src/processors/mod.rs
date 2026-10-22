//! Mesh processors: STL reading (`stl_processing`) and writing (`stl_writer`) with
//! `mesh_output_settings`, the face BSP tree (`face_bsp_tree`), the convex hull
//! (`mesh_convex_hull`, built by `mi_convex_hull`, derived from MIConvexHull) and the vertices
//! on or outside it that transformed bounds are measured over (`hull_bounds_points`),
//! `planar_region_simplifier` (on manifold-rust's ear clipper), `mesh_helper`'s plane, and
//! turning 2D paths into meshes (`vertex_source_to_mesh`), whose solids are built on the loops
//! `ring_loops` prepares, placed and capped by `ring_vertices` over `cap_fill`.

pub mod cap_fill;
pub mod face_bsp_tree;
pub(crate) mod hull_bounds_points;
pub mod mesh_convex_hull;
pub mod mesh_helper;
pub mod mesh_output_settings;
mod mi_convex_hull;
pub mod planar_region_simplifier;
pub mod ring_loops;
pub mod ring_vertices;
pub mod stl_processing;
pub mod stl_writer;
pub mod vertex_source_to_mesh;

pub use face_bsp_tree::BspNode;
pub use mesh_convex_hull::ConvexHull;
pub use mesh_output_settings::{CsgOption, MeshOutputSettings, OutputType};
pub use stl_processing::{Progress, StlChunk, StlError, StlParse, FACES_PER_YIELD};
