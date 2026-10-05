//! The pieces of geometry3Sharp (https://github.com/gradientspace/geometry3Sharp, Copyright Ryan
//! Schmidt / gradientspace, Boost Software License 1.0, vendored in agg-sharp) that agg-sharp's
//! PolygonMesh reaches: an append-only `DMesh3` with its topology edits, the top-down `DMeshAABBTree3` and its
//! nearest-triangle query, `MeshSignedDistanceGrid` in its narrow-band flood mode (for
//! `sdf::MeshSdfGrid`) and its default signed narrow-band mode (`signed_distance_grid`), the
//! implicit functions and boolean operators (`implicit3d`), and `MarchingCubes` /
//! `MarchingCubesPro`, which the implicit booleans (`csg::implicit_boolean`) mesh with. Only
//! what is reached is ported; each file names the C# it follows. See `THIRD_PARTY_NOTICES.md`.
//! The dynamic `DMesh3` (`dmesh3*`, `ref_count_vector`, `small_list_set`) is the base the mesh
//! repair (`MeshAutoRepair`) builds on.

pub(crate) mod dmesh3;
pub(crate) mod dmesh3_collapse;
pub(crate) mod dmesh3_edge_ops;
pub(crate) mod dmesh3_queries;
pub(crate) mod dmesh_aabb_tree3;
pub(crate) mod implicit3d;
pub(crate) mod marching_cubes;
pub(crate) mod marching_cubes_pro;
pub(crate) mod mesh_signed_distance_grid;
pub(crate) mod ref_count_vector;
pub(crate) mod signed_distance_grid;
pub(crate) mod small_list_set;
pub(crate) mod tri_distance;

pub(crate) use dmesh3::DMesh3;
pub(crate) use dmesh_aabb_tree3::DMeshAABBTree3;

#[cfg(test)]
mod dmesh3_tests;
