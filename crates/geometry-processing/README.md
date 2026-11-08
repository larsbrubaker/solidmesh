# solidmesh

Solid modeling on triangle meshes in pure Rust: fillets and chamfers on mesh edges, corners and 2D outlines, over a mesh kernel with booleans, repair and implicit (SDF) operations that keep closed solids closed.

This is the one crate to depend on: it re-exports `solidmesh-math`, `solidmesh-clipper` and
`solidmesh-mesh` as `solidmesh::math`, `solidmesh::clipper` and `solidmesh::mesh`.

```toml
[dependencies]
solidmesh = "0.1"
```

A part's fillet or chamfer is one call, `solidmesh::bevel_part`, whose outcome reports every
pick that did not fit as a code with the largest size that would have.

It is one of the four crates of [solidmesh](https://github.com/larsbrubaker/solidmesh): pure,
deterministic Rust that builds for `wasm32-unknown-unknown`. The
[repository README](https://github.com/larsbrubaker/solidmesh#readme) says what the library does,
shows the live demo and lists the crates; the API is documented on
[docs.rs](https://docs.rs/solidmesh).

License: BSD-2-Clause.
