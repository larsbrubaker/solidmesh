//! The pieces of geometry3Sharp (https://github.com/gradientspace/geometry3Sharp, Copyright Ryan
//! Schmidt / gradientspace, Boost Software License 1.0, vendored in agg-sharp) that agg-sharp's
//! PolygonMesh reaches: an append-only `DMesh3`, the top-down `DMeshAABBTree3` and its
//! nearest-triangle query, `MeshSignedDistanceGrid` in its narrow-band flood mode (for
//! `sdf::MeshSdfGrid`) and its default signed narrow-band mode (`signed_distance_grid`), the
//! implicit functions and boolean operators (`implicit3d`), and `MarchingCubes` /
//! `MarchingCubesPro`, which the implicit booleans (`csg::implicit_boolean`) mesh with. Only
//! what is reached is ported; each file names the C# it follows. See `THIRD_PARTY_NOTICES.md`.

pub(crate) mod dmesh3;
pub(crate) mod dmesh_aabb_tree3;
pub(crate) mod implicit3d;
pub(crate) mod marching_cubes;
pub(crate) mod marching_cubes_pro;
pub(crate) mod mesh_signed_distance_grid;
pub(crate) mod signed_distance_grid;

pub(crate) use dmesh3::DMesh3;
pub(crate) use dmesh_aabb_tree3::DMeshAABBTree3;
