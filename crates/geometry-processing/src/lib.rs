//! geometry-processing (package `solidmesh`): the bevel/fillet kernel. Pure geometry over
//! `polygon-mesh`, `vector-math` and `clipper-library`; nothing here may depend on a scene, a UI,
//! an application or a 2D path model (MatterCAD's app crates and agg-path, its consumer), and
//! `tests/kernel_boundary_tests.rs` reads the crate's `Cargo.toml` to guard that split. `path`
//! and the path corners work on plain contours, already flattened; MatterCAD converts its paths
//! in its mattercad-geometry crate.
//!
//! `topology` holds feature edges, chains, face groups, the region cache, edge picking, the bevel
//! edge/corner group records, the edge and corner group resolvers, path corners and the readings
//! of a pick or stored group; `path` the path bevel and segment runs; and `bevel` the whole
//! kernel: advisories, the level-line, section and surface builders, the tool, bead and clip
//! builders, and a whole part's fillet or chamfer through its one entry point,
//! `bevel::part::bevel_part`.
//! The live size preview, which drives the model's rebuild, lives in MatterCAD (mattercad-model).

pub mod bevel;
pub mod path;
pub mod topology;
