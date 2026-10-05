//! MatterCAD.GeometryProcessing: the bevel/fillet kernel. Ports
//! `R/MatterCAD.GeometryProcessing` folder by folder (CLAUDE.md: Rust modules follow the C#
//! folders). Pure geometry over `polygon-mesh` and `vector-math`; nothing here may depend on the
//! scene, the UI or the app (C# `KernelBoundaryTests` guards the same split; here the crate's
//! `Cargo.toml` dependency list is that guard). `Path/` works on `agg-path`'s vertex sources.
//!
//! Ported so far: `Topology/` (feature edges, chains, face groups, the region cache, edge
//! picking, the bevel edge/corner group records, the edge and corner group resolvers, path
//! corners and the readings of a pick or stored group); `Path/` (the path bevel and segment
//! runs); `Bevel/` advisories and the leaf clusters of the bevel core below its cyclic centre
//! (slices 1-7 of `docs/design/bevel-slices.md`, less `BevelLiveSizePreview`). The rest of
//! `Bevel/` lands in later Phase 5C slices.

pub mod bevel;
pub mod path;
pub mod topology;
