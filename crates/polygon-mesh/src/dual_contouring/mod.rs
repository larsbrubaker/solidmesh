//! Dual contouring: the signed distance functions of [`density`] (the [`density::Sdf`] trait, its
//! shapes and combinators), the public-domain QEF solver (`qef`), and octree dual contouring
//! (`octree`, `octree_contour`).
//!
//! The reference octree implementation carries an LGPL 2.1 licence, which the
//! permissive-licenses-only contract does not admit, so `octree` and `octree_contour` are written
//! clean-room from the published algorithm (Ju et al., "Dual Contouring of Hermite Data", SIGGRAPH
//! 2002) behind two public calls (`build_octree`, `generate_mesh_from_octree`).

pub mod density;
pub mod octree;
mod octree_contour;
pub(crate) mod qef;

pub use density::{Box, Cylinder, Intersection, Sdf, Sphere, Subtraction, Transform, Union};
pub use octree::{build_octree, OctreeNode};
pub use octree_contour::generate_mesh_from_octree;
