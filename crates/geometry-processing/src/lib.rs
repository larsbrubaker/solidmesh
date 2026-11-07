//! The bevel/fillet kernel. Pure geometry over `polygon-mesh` and `vector-math`; nothing here may
//! depend on the scene, the UI or the app, and the crate's `Cargo.toml` dependency list is what
//! guards that split. `path` works on `agg-path`'s vertex sources.
//!
//! `topology` holds feature edges, chains, face groups, the region cache, edge picking, the bevel
//! edge/corner group records, the edge and corner group resolvers, path corners and the readings
//! of a pick or stored group; `path` the path bevel and segment runs; and `bevel` the whole
//! kernel: advisories, the level-line, section and surface builders and the tool, bead and clip
//! builders.
//! The live size preview, which drives the model's rebuild, lives in mattercad-model.

pub mod bevel;
pub mod path;
pub mod topology;
