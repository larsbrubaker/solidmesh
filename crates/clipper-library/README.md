# solidmesh-clipper

Integer polygon booleans, offsets and simplification (a pure-Rust port of Clipper 6.4.2) with polygon helpers on clipper2-rust's integer types, the 2D clipper under the solidmesh mesh kernel.

Most users depend on [`solidmesh`](https://crates.io/crates/solidmesh), which re-exports this crate as `solidmesh::clipper`.

It is one of the four crates of [solidmesh](https://github.com/larsbrubaker/solidmesh): pure,
deterministic Rust that builds for `wasm32-unknown-unknown`. The
[repository README](https://github.com/larsbrubaker/solidmesh#readme) says what the library does,
shows the live demo and lists the crates; the API is documented on
[docs.rs](https://docs.rs/solidmesh-clipper).

License: BSD-2-Clause.
