//! polygon-mesh: MatterCAD's triangle meshes and the geometry built on them.
//!
//! The Mesh core: [`Mesh`] (f32 vertices, indexed [`Face`]s with normals; welding,
//! winding, hashing, transforms, bounds), its face/edge queries ([`MeshEdgeGraph`], also built in
//! steps by [`MeshEdgeGraphSteps`],
//! [`MeshEdge`], [`VertexFaceList`]), volume and area (`mesh_measure`), the vertex sorters,
//! the Platonic solids and STL read/write (`processors`). The booleans (`csg`) run on
//! manifold-rust.
//!
//! Mesh editing: vertex merging over a vertex BVH (`mesh_editing`), cutting faces and
//! meshes by planes (`face_split`, `mesh_editing`), T-junction repair (`t_junctions`) and the
//! face BSP tree (`processors::face_bsp_tree`), slicing a mesh into clipper2 integer polygons
//! (`csg::slice_layer`, on agg-path's `clipper_library`) and walling loops back up (`csg::path_stitcher`).
//!
//! The ray tracer (`ray_tracer`): traceables, hits, the BVH and its default builder,
//! the shapes MatterCAD picks with, materials, solid box/sphere shapes and view-space CSG.
//!
//! The implicit surfaces (`sdf`): marching cubes over a sampled field and `MeshSdfGrid`, the
//! offset preview, on the pieces derived from geometry3Sharp (`g3`); the signed distance
//! functions of `dual_contouring`.
//!
//! Per-face colors and textures (`face_data`): `Mesh::face_colors` and
//! `Mesh::face_textures` follow the faces through every rebuild, and booleans paint their result
//! from the operands' colors.
//!
//! `mesh_distance` measures the true gap between two meshes' surfaces and whether they
//! overlap. `rendering` has the outline a mesh casts straight down, as clipper polygons
//! (`rendering::orthographic_z_projection`).
//!
//! Conventions follow vector-math: queries are methods on `Mesh` and `Face`, indices are
//! `i32` (the sizes saved files and the model use), and matrices are row-major with row
//! vectors (`p * M`). Long operations take a [`CancelToken`].

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
mod mesh_edge_graph_steps;
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
pub use mesh_edge_graph_steps::MeshEdgeGraphSteps;
pub use mesh_editing::{ProgressReporter, UnionFind};
pub use mesh_extensions::PositionNormal;
pub use transformed_aabb_cache::TransformedAabbCache;
