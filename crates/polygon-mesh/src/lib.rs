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
//! face BSP tree (`processors::face_bsp_tree`), slicing a mesh into clipper2 integer polygons
//! (`csg::slice_layer`, on agg-path's `clipper_library`) and walling loops back up (`csg::path_stitcher`).
//!
//! Phase 2d, the ray tracer (`ray_tracer`): traceables, hits, the BVH and its default builder,
//! the shapes MatterCAD picks with, materials, solid box/sphere shapes and view-space CSG.
//!
//! Phase 2e, the implicit surfaces (`sdf`): marching cubes over a sampled field and `MeshSdfGrid`,
//! the offset preview, on the geometry3Sharp pieces it reaches (`g3`); the signed distance
//! functions of `dual_contouring`.
//!
//! Phase 2f, per-face colors and textures (`face_data`): `Mesh::face_colors` and
//! `Mesh::face_textures` follow the faces through every rebuild, and booleans paint their result
//! from the operands' colors.
//!
//! `mesh_distance` (MatterCADLib `MeshDistance`) measures the true gap between two meshes'
//! surfaces and whether they overlap. `rendering` (agg-sharp `PolygonMesh/Rendering`) has the
//! outline a mesh casts straight down, as clipper polygons (`OrthographicZProjection`).
//!
//! Conventions follow vector-math: C# properties are methods, C# extension methods on `Mesh`
//! and `Face` are inherent methods, indices stay `i32` as in C#, and matrices are row-major
//! with row vectors (`p * M`). Long operations take a [`CancelToken`].

pub mod cancel;
pub mod csg;
pub mod data_converters_2d;
pub mod dual_contouring;
mod face;
mod face_data;
mod face_split;
mod g3;
mod mesh;
pub mod mesh_distance;
mod mesh_edge_graph;
mod mesh_editing;
mod mesh_extensions;
mod mesh_measure;
pub mod processors;
pub mod ray_tracer;
pub mod rendering;
pub mod sdf;
pub mod solids;
mod t_junctions;
pub mod transformed_aabb_cache;
pub mod vector3_list;
pub mod vertex_sorters;

pub use cancel::CancelToken;
pub use face::{Face, FaceList};
pub use face_data::{CentroidColor, FaceTextureData, FaceTextures};
pub use face_split::{clip_edge_at_plane, SplitData};
pub use g3::advanced_repair::{AdvancedRepairOptions, RemoveModes};
pub use g3::mesh_auto_repair::MeshAutoRepairError;
pub use mesh::Mesh;
pub use mesh_edge_graph::{MeshEdge, MeshEdgeGraph, VertexFaceList};
pub use mesh_editing::{ProgressReporter, UnionFind};
pub use mesh_extensions::PositionNormal;
pub use transformed_aabb_cache::TransformedAabbCache;
