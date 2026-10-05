//! Ported agg-sharp `Tests/Agg.Tests/Agg.PolygonMesh` boolean test classes, one file each
//! (CLAUDE.md net 1). Cases that need code not ported yet are listed as skipped in each file's
//! header.

#[path = "csg/csg_tests.rs"]
mod csg_tests;
#[path = "csg/implicit_boolean_tests.rs"]
mod implicit_boolean_tests;
#[path = "csg/manifold_rust_backend_cancel_tests.rs"]
mod manifold_rust_backend_cancel_tests;
#[path = "csg/manifold_rust_backend_tests.rs"]
mod manifold_rust_backend_tests;
#[path = "csg/minkowski_progress_model_tests.rs"]
mod minkowski_progress_model_tests;
