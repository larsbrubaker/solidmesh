//! The pieces of geometry3Sharp (https://github.com/gradientspace/geometry3Sharp, Copyright Ryan
//! Schmidt / gradientspace, Boost Software License 1.0, vendored in agg-sharp) that
//! `sdf::MeshSdfGrid` reaches: an append-only `DMesh3`, the top-down `DMeshAABBTree3` and its
//! nearest-triangle query, and `MeshSignedDistanceGrid`'s narrow-band flood. Only what is reached
//! is ported; each file names the C# it follows. See `THIRD_PARTY_NOTICES.md`.

pub(crate) mod dmesh3;
pub(crate) mod dmesh_aabb_tree3;
pub(crate) mod mesh_signed_distance_grid;

pub(crate) use dmesh3::DMesh3;
pub(crate) use dmesh_aabb_tree3::DMeshAABBTree3;
