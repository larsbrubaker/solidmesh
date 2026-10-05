//! Port of agg-sharp `PolygonMesh/DualContouring/`: the signed distance functions of
//! `density.cs` (`ISdf` and its shapes and combinators), the public-domain QEF solver (`qef`), and
//! octree dual contouring (`octree`, `octree_contour`).
//!
//! `Octree.cs` itself carries Tao Ju's LGPL 2.1 header, which the permissive-licenses-only
//! contract does not admit, so `octree` and `octree_contour` are written clean-room from the
//! published algorithm (Ju et al., "Dual Contouring of Hermite Data", SIGGRAPH 2002) behind the
//! same public calls (`build_octree`, `generate_mesh_from_octree`); see
//! `docs/CSHARP_DIVERGENCES.md`. `PythonCode.cs` is `#if false` dead code and `Perlin.cs` is
//! unused by MatterCAD; neither is ported.

pub mod density;
pub mod octree;
mod octree_contour;
pub mod qef;

pub use density::{Box, Cylinder, Intersection, Sdf, Sphere, Subtraction, Transform, Union};
pub use octree::{build_octree, OctreeNode};
pub use octree_contour::generate_mesh_from_octree;
