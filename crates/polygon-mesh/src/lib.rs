//! **solidmesh-mesh**: triangle meshes for solid modeling, in pure Rust: booleans that keep
//! closed solids closed, repair, signed distance fields and implicit meshing, convex hulls and
//! Minkowski sums, extrusion and revolution of 2D outlines, slicing, ray tracing and STL. It
//! builds for `wasm32-unknown-unknown`, contains no C or C++, and gives the same mesh on every
//! platform.
//!
//! The mesh is [`Mesh`]: `f32` vertices and indexed triangle [`Face`]s, with volume, area, bounds
//! and welding as methods. Where to start:
//!
//! - **Booleans**: [`csg::csg_operations`] (`union`, `subtract`, `intersect` of two meshes) and
//!   [`csg::boolean_processing::do_array`] (many operands, each with its own matrix), on
//!   [manifold-rust](https://docs.rs/manifold-rust);
//! - **Outlines**: [`processors::outline_to_mesh`] extrudes (optionally with bevelled edges) or
//!   revolves 2D contours into closed solids;
//! - **Solids**: [`solids::platonic_solids`] and [`solids::RoundedCornerBox`];
//! - **Implicit surfaces**: [`sdf`] (marching cubes over a sampled field) and [`dual_contouring`];
//! - **Minkowski sums and hulls**: [`csg::minkowski_morph`], [`processors::mesh_convex_hull`];
//! - **Repair**: [`csg::mesh_repair_rust`], and [`AdvancedRepairOptions`] for the full repair;
//! - **Remeshing for analysis**: [`remesh_for_analysis`] rebuilds a closed solid's surface with
//!   well-shaped triangles at a size given per position, keeping its sharp creases;
//! - **Files**: [`processors::stl_processing`] reads STL and [`processors::stl_writer`] writes it.
//!
//! Long operations take a [`CancelToken`]; matrices are `solidmesh-math`'s row-major
//! [`Matrix4X4`](solidmesh_math::Matrix4X4) with row vectors (`p * M`).
//!
//! ```
//! use solidmesh_mesh::csg::csg_operations::subtract;
//! use solidmesh_mesh::processors::outline_to_mesh::extrude_contours;
//! use solidmesh_mesh::solids::platonic_solids::create_cube;
//! use solidmesh_mesh::CancelToken;
//! use solidmesh_math::Vector2;
//!
//! // A 20 mm cube with a 10 mm cube cut out of its middle: a closed hollow solid.
//! let hollow = subtract(
//!     &create_cube(20.0, 20.0, 20.0),
//!     &create_cube(10.0, 10.0, 10.0),
//!     &CancelToken::new(),
//! )?;
//! assert!((hollow.get_volume() - 7000.0).abs() < 1e-6);
//!
//! // A 10 by 5 rectangle extruded 2 mm tall.
//! let rectangle = vec![
//!     Vector2::new(0.0, 0.0),
//!     Vector2::new(10.0, 0.0),
//!     Vector2::new(10.0, 5.0),
//!     Vector2::new(0.0, 5.0),
//! ];
//! let plate = extrude_contours(&[rectangle], 2.0)?;
//! assert!((plate.get_volume() - 100.0).abs() < 1e-6);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! solidmesh-mesh is one of four crates: [solidmesh-math](https://docs.rs/solidmesh-math)
//! (vectors, matrices, bounds), [solidmesh-clipper](https://docs.rs/solidmesh-clipper) (integer
//! polygon clipping), this one and [solidmesh](https://docs.rs/solidmesh) (fillets and chamfers),
//! which re-exports the other three; depend on `solidmesh` alone to get them all.
//!
//! # How the crate is laid out
//!
//! The Mesh core: [`Mesh`] (f32 vertices, indexed [`Face`]s with normals; welding,
//! winding, hashing, transforms, bounds), its face/edge queries ([`MeshEdgeGraph`], also built in
//! steps by [`MeshEdgeGraphSteps`],
//! [`MeshEdge`], [`VertexFaceList`]), volume and area (`mesh_measure`), the vertex sorters,
//! the Platonic solids, STL read/write and the split into separate volumes (`processors`). The
//! booleans (`csg`) run on manifold-rust.
//!
//! Mesh editing: vertex merging over a vertex BVH (`mesh_editing`), cutting faces and
//! meshes by planes (`face_split`, `mesh_editing`), T-junction repair (`t_junctions`) and the
//! face BSP tree (`processors::face_bsp_tree`), slicing a mesh into clipper2 integer polygons
//! (`csg::slice_layer`, on `solidmesh_clipper`) and walling loops back up (`csg::path_stitcher`).
//!
//! The ray tracer (`ray_tracer`): traceables, hits, the BVH and its default builder,
//! the shapes MatterCAD picks with, materials, solid box/sphere shapes and view-space CSG, and
//! what shading a hit needs: `MeshSurface` and the `SurfacePoint` it answers with (smooth
//! normals, color, a safe origin for a continuing ray), and `PlacedParts` for many placed parts
//! sharing a tree; and the scattering model a path tracer shades with
//! (`ray_tracer::ScatteringSurface`).
//!
//! The implicit surfaces (`sdf`): marching cubes over a sampled field and `MeshSdfGrid`, the
//! offset preview, on the pieces derived from geometry3Sharp (`g3`); the signed distance
//! functions of `dual_contouring`.
//!
//! Per-face colors and textures (`face_data`): `Mesh::face_colors` and
//! `Mesh::face_textures` follow the faces through every rebuild, and booleans paint their result
//! from the operands' colors and keep each operand face's picture and UVs
//! (`csg::manifold_uv_channel`).
//!
//! `mesh_distance` measures the true gap between two meshes' surfaces and whether they
//! overlap.
//!
//! Conventions follow vector-math: queries are methods on `Mesh` and `Face`, indices are
//! `i32` (the sizes saved files and the model use), and matrices are row-major with row
//! vectors (`p * M`). Long operations take a [`CancelToken`].

pub mod analysis_remesh;
pub mod cancel;
pub mod csg;
pub mod data_converters_2d;
pub mod dual_contouring;
mod face;
mod face_data;
mod face_split;
mod face_texture_fill;
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
pub mod sdf;
pub mod solids;
mod t_junctions;
mod transformed_aabb_cache;
pub(crate) mod vector3_list;
#[doc(hidden)]
pub mod vertex_sorters;

pub use analysis_remesh::{
    remesh_for_analysis, AnalysisRemesh, AnalysisRemeshError, AnalysisRemeshOptions,
    AnalysisRemeshReport, AnalysisSurface,
};
pub use cancel::{CancelToken, Cancelled};
pub use face::{Face, FaceList};
pub use face_data::{
    take_uniform_face_color, CentroidColor, CornerTexture, FaceTextureData, FaceTextures,
};
pub use face_split::{clip_edge_at_plane, SplitData};
pub use g3::advanced_repair::{AdvancedRepairOptions, RemoveModes};
pub use g3::mesh_auto_repair::MeshAutoRepairError;
pub use mesh::Mesh;
pub use mesh_edge_graph::{MeshEdge, MeshEdgeGraph, VertexFaceList};
pub use mesh_edge_graph_steps::MeshEdgeGraphSteps;
pub use mesh_editing::{ProgressCallback, UnionFind};
pub use mesh_extensions::PositionNormal;
pub use transformed_aabb_cache::TransformedAabbCache;
