//! polygon-mesh: a port of agg-sharp's `PolygonMesh` (`R/Submodules/agg-sharp/PolygonMesh`).
//!
//! Phase 2a, the Mesh core: [`Mesh`] (f32 vertices, indexed [`Face`]s with normals; welding,
//! winding, hashing, transforms, bounds), its face/edge queries ([`MeshEdgeGraph`],
//! [`MeshEdge`], [`VertexFaceList`]), volume and area (`mesh_measure`), the vertex sorters,
//! the Platonic solids and STL read/write (`processors`). Phase 2b adds the booleans (`csg`),
//! which run on manifold-rust.
//!
//! Conventions follow vector-math: C# properties are methods, C# extension methods on `Mesh`
//! and `Face` are inherent methods, indices stay `i32` as in C#, and matrices are row-major
//! with row vectors (`p * M`). Long operations take a [`CancelToken`].

pub mod cancel;
pub mod csg;
mod face;
mod mesh;
mod mesh_edge_graph;
mod mesh_extensions;
mod mesh_measure;
pub mod processors;
pub mod solids;
pub mod vector3_list;
pub mod vertex_sorters;

pub use cancel::CancelToken;
pub use face::{Face, FaceList};
pub use mesh::Mesh;
pub use mesh_edge_graph::{MeshEdge, MeshEdgeGraph, VertexFaceList};
pub use mesh_extensions::PositionNormal;
