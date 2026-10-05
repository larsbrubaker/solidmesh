//! MatterCAD.GeometryProcessing: the bevel/fillet kernel. Ports
//! `R/MatterCAD.GeometryProcessing` folder by folder (CLAUDE.md: Rust modules follow the C#
//! folders). Pure geometry over `polygon-mesh` and `vector-math`; nothing here may depend on the
//! scene, the UI or the app (C# `KernelBoundaryTests` guards the same split; here the crate's
//! `Cargo.toml` dependency list is that guard).
//!
//! Ported so far: `Topology/` feature edges, chains, face groups, the region cache, edge
//! picking, the bevel edge/corner group records, the edge group resolver and the readings of a
//! pick or stored group; `Bevel/` advisories. Topology's path corners and corner group resolver,
//! `Path/` and the rest of `Bevel/` land in later Phase 5C slices.

pub mod bevel;
pub mod topology;
