//! Port of agg-sharp `PolygonMesh/DualContouring/`, the part that is ours to port: the signed
//! distance functions of `density.cs` (`ISdf` and its shapes and combinators).
//!
//! Not ported here: `Octree.cs` (the dual contouring octree itself) carries Tao Ju's LGPL 2.1
//! header, which the permissive-licenses-only contract does not admit, so it and the
//! public-domain QEF solver that only it calls (`QefSolver.cs`, `QefData.cs`, `SVD.cs`,
//! `Givens.cs`, `Schur2.cs`, `Mat3.cs`, `SMat3.cs`, `MatUtils.cs`) and its node types wait on
//! that decision (see `docs/PORTING_PLAN.md`). `PythonCode.cs` is `#if false` dead code and
//! `Perlin.cs` is unused by MatterCAD; neither is ported.

pub mod density;

pub use density::{Box, Cylinder, Intersection, Sdf, Sphere, Subtraction, Transform, Union};
