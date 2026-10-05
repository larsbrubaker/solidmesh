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
//! The remesher `MinimalHoleFill` runs (`remesher`, `remesher_pro`, with `mesh_constraints`,
//! `mesh_refiner_base`, `mesh_util` and `vector3d`) edits that `DMesh3`, on a submesh copy of
//! the fill that `region_operator` / `dsubmesh3` extract and splice back, sized with
//! `mesh_queries`' edge-length statistics.

pub(crate) mod dmesh3;
pub(crate) mod dmesh3_collapse;
pub(crate) mod dmesh3_edge_ops;
pub(crate) mod dmesh3_queries;
pub(crate) mod dmesh3_set_triangle;
pub(crate) mod dmesh_aabb_tree3;
pub(crate) mod dsubmesh3;
pub(crate) mod dynamic_priority_queue;
pub(crate) mod edge_loop;
pub(crate) mod implicit3d;
pub(crate) mod marching_cubes;
pub(crate) mod marching_cubes_pro;
pub(crate) mod merge_coincident_edges;
pub(crate) mod mesh_boundary_loops;
pub(crate) mod mesh_constraints;
pub(crate) mod mesh_editor;
pub(crate) mod mesh_face_selection;
pub(crate) mod mesh_iterators;
pub(crate) mod mesh_queries;
pub(crate) mod mesh_refiner_base;
pub(crate) mod mesh_repair_orientation;
pub(crate) mod mesh_signed_distance_grid;
pub(crate) mod mesh_util;
pub(crate) mod point_set_hashtable;
pub(crate) mod ray_intersection;
pub(crate) mod ref_count_vector;
pub(crate) mod region_operator;
pub(crate) mod remesher;
pub(crate) mod remesher_pro;
pub(crate) mod signed_distance_grid;
pub(crate) mod simple_hole_filler;
pub(crate) mod small_list_set;
pub(crate) mod tri_distance;
pub(crate) mod vector3d;

pub(crate) use dmesh3::DMesh3;
pub(crate) use dmesh_aabb_tree3::DMeshAABBTree3;

#[cfg(test)]
mod dmesh3_tests;
#[cfg(test)]
mod mesh_repair_ops_tests;
#[cfg(test)]
mod mesh_topology_tests;
#[cfg(test)]
mod region_operator_tests;
#[cfg(test)]
mod remesher_tests;
