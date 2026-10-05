//! Mesh processors (agg-sharp `PolygonMesh/Processors`). This slice ports STL reading and
//! writing (`StlProcessing.cs`, split into `stl_processing` for the reader and `stl_writer`)
//! and `MeshOutputSettings`; the rest of the folder joins in later Phase 2 slices.

pub mod mesh_output_settings;
pub mod stl_processing;
pub mod stl_writer;

pub use mesh_output_settings::{CsgOption, MeshOutputSettings, OutputType};
pub use stl_processing::{Progress, StlChunk, StlError, StlParse, FACES_PER_YIELD};
