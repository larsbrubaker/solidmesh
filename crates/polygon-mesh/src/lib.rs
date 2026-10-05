//! polygon-mesh: a port of agg-sharp's `PolygonMesh` (`R/Submodules/agg-sharp/PolygonMesh`).
//!
//! Phase 2a, the Mesh core: [`Mesh`] (f32 vertices, indexed [`Face`]s with normals; welding,
//! winding, hashing, transforms, bounds), its face/edge queries ([`MeshEdgeGraph`],
//! [`MeshEdge`], [`VertexFaceList`]), volume and area (`mesh_measure`), the vertex sorters,
//! the Platonic solids and STL read/write (`processors`). Phase 2b adds the booleans (`csg`),
//! which run on manifold-rust.
//!
//! Phase 2c, mesh editing: vertex merging over a vertex BVH (`mesh_editing`), cutting faces and
//! meshes by planes (`face_split`, `mesh_editing`), T-junction repair (`t_junctions`) and the
//! face BSP tree (`processors::face_bsp_tree`).
//!
//! Conventions follow vector-math: C# properties are methods, C# extension methods on `Mesh`
//! and `Face` are inherent methods, indices stay `i32` as in C#, and matrices are row-major
//! with row vectors (`p * M`). Long operations take a [`CancelToken`].

pub mod cancel;
pub mod csg;
mod face;
mod face_split;
mod mesh;
mod mesh_edge_graph;
mod mesh_editing;
mod mesh_extensions;
mod mesh_measure;
pub mod processors;
pub mod solids;
mod t_junctions;
pub mod vector3_list;
pub mod vertex_sorters;

pub use cancel::CancelToken;
pub use face::{Face, FaceList};
pub use face_split::{clip_edge_at_plane, SplitData};
pub use mesh::Mesh;
pub use mesh_edge_graph::{MeshEdge, MeshEdgeGraph, VertexFaceList};
pub use mesh_editing::{ProgressReporter, UnionFind};
pub use mesh_extensions::PositionNormal;
