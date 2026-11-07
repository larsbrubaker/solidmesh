//! The test parts' solids, drawn as agg-path outlines and built by the kernel's extrude and revolve
//! on the integer polygons agg-path converts them to (`create_polygons(DEFAULT_SCALING)`), the
//! same polygons the app's paths become, so a part here is the part a path in the app makes.
//! Shared by the bevel and topology binaries and by the model's and app's bevel tests that
//! build these parts (alongside `topology_test_meshes`).

use agg_path::clipper_library::ClipperError;
use agg_path::data_converters_2d::{VertexSourceToClipperPolygons, DEFAULT_SCALING};
use agg_path::vertex_source::VertexSource;
use polygon_mesh::processors::outline_to_mesh::{self, RevolveError};
use polygon_mesh::Mesh;

/// The outline extruded from z = 0 up to `height`, square-ended.
pub fn extrude(outline: &dyn VertexSource, height: f64) -> Result<Mesh, ClipperError> {
    outline_to_mesh::extrude(&outline.create_polygons(DEFAULT_SCALING), height)
}

/// The outline (x out from the axis, y along it) swept from `angle_start` to `angle_end`.
pub fn revolve(
    outline: &dyn VertexSource,
    angle_steps: i32,
    angle_start: f64,
    angle_end: f64,
    revolve_around_z: bool,
) -> Result<Mesh, RevolveError> {
    outline_to_mesh::revolve(
        &outline.create_polygons(DEFAULT_SCALING),
        angle_steps,
        angle_start,
        angle_end,
        revolve_around_z,
    )
}
