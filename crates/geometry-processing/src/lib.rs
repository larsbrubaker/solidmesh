//! MatterCAD.GeometryProcessing: the bevel/fillet kernel. Ports
//! `R/MatterCAD.GeometryProcessing` folder by folder (CLAUDE.md: Rust modules follow the C#
//! folders). Pure geometry over `polygon-mesh` and `vector-math`; nothing here may depend on the
//! scene, the UI or the app (C# `KernelBoundaryTests` guards the same split; here the crate's
//! `Cargo.toml` dependency list is that guard).
//!
//! Ported so far: `Topology/` feature edges, chains, face groups, the region cache and edge
//! picking. The rest of `Topology/` (the bevel group records and their resolvers, pick
//! interpretations, path corners), `Path/` and `Bevel/` land in later Phase 5C slices.

pub mod bevel;
pub mod topology;
