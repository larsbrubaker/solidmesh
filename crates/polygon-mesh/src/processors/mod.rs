//! Mesh processors (agg-sharp `PolygonMesh/Processors`). This slice ports STL reading and
//! writing (`StlProcessing.cs`, split into `stl_processing` for the reader and `stl_writer`)
//! and `MeshOutputSettings`, plus the face BSP tree (`FaceBspTree.cs`); the rest of the folder joins in later Phase 2 slices.

pub mod face_bsp_tree;
pub mod mesh_output_settings;
pub mod stl_processing;
pub mod stl_writer;
pub mod vertex_source_to_mesh;

pub use face_bsp_tree::BspNode;
pub use mesh_output_settings::{CsgOption, MeshOutputSettings, OutputType};
pub use stl_processing::{Progress, StlChunk, StlError, StlParse, FACES_PER_YIELD};
