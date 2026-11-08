# solidmesh-mesh

Triangle meshes for solid modeling: booleans that keep closed solids closed, repair, signed distance fields and implicit meshing, convex hulls and Minkowski sums, extrusion and revolution of 2D outlines, slicing, ray tracing and STL.

Most users depend on [`solidmesh`](https://crates.io/crates/solidmesh), which re-exports this crate as `solidmesh::mesh`.

It is one of the four crates of [solidmesh](https://github.com/larsbrubaker/solidmesh): pure,
deterministic Rust that builds for `wasm32-unknown-unknown`. The
[repository README](https://github.com/larsbrubaker/solidmesh#readme) says what the library does,
shows the live demo and lists the crates; the API is documented on
[docs.rs](https://docs.rs/solidmesh-mesh).

License: BSD-2-Clause.
