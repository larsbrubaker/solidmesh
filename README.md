# solidmesh

**Solid modeling on triangle meshes, in pure Rust: booleans, fillets and chamfers, repair, and
implicit (SDF) operations that keep closed solids closed.**

solidmesh is the geometry kernel behind MatterCAD, being moved out
of the application into its own library so that others can use it and build on it. It works on
the meshes real pipelines contain (STL and 3MF downloads, scans, triangle soup) as well as on
clean modeled parts, builds for `wasm32-unknown-unknown`, contains no C or C++, and gives the
same answer on every platform.

## Status

**In extraction: no code has moved here yet.** The kernel still lives in the
[mattercad-rust](https://github.com/MatterHackers/mattercad-rust) workspace
(`crates/vector-math`, `crates/polygon-mesh`, `crates/geometry-processing`) while it is cut free
of MatterCAD's 2D path and image crate (`agg-path`). The plan, with its decisions and the steps
still open, is the "solidmesh" item in mattercad-rust's
[`docs/roadmap/geometry-libraries.md`](https://github.com/MatterHackers/mattercad-rust/blob/main/docs/roadmap/geometry-libraries.md);
each step lands on mattercad-rust's `main` as it is done.

As soon as the kernel no longer needs `agg-path`, the crates move here with their history and
mattercad-rust depends on them by path. From then on the work happens here: results as codes
only, the fillet orchestration moving in from the app, the public API, and publishing to
crates.io.

What is here already is the workspace the crates move into: the repository gates
(`crates/workspace-gates`), the pinned toolchain, lint and test settings, and CI.

## Planned crates

| Crate | What it holds | Comes from (mattercad-rust) |
|---|---|---|
| `solidmesh-math` | Vectors, matrices, planes, rays and bounds in `f64` and `f32` | `crates/vector-math` |
| `solidmesh-mesh` | The triangle mesh; welding, winding, measures; booleans on [manifold-rust](https://github.com/larsbrubaker/manifold-rust); repair; SDF and implicit meshing; convex hull and Minkowski sums; extrusion and revolution of 2D outlines; STL | `crates/polygon-mesh` |
| `solidmesh` | Fillets and chamfers on mesh edges, corners and outlines, behind one public API; re-exports the other two | `crates/geometry-processing` |

## Design rules

- **Pure, safe Rust.** No C or C++, no `-sys` crates, no native build scripts; every library
  crate builds for `wasm32-unknown-unknown`.
- **Deterministic.** `f64` geometry in a stable operation order, never `mul_add`; explicit
  tie-breaks wherever order reaches a result.
- **Closed in, closed out.** An operation on a closed solid returns a closed solid or a typed
  refusal that says why, with the largest size that would have worked where there is one.
- **Errors are values.** Library code returns `Result`s and never panics on input.

## Building

The crates depend on three sibling libraries by path, so check them out beside this repository:

```
<any folder>/
  solidmesh/       github.com/larsbrubaker/solidmesh
  manifold-rust/   github.com/larsbrubaker/manifold-rust
  clipper2-rust/   github.com/larsbrubaker/clipper2-rust
  tess2-rust/      github.com/larsbrubaker/tess2-rust
```

`rust-toolchain.toml` pins the Rust version (rustup installs it on first use). From
`solidmesh/`:

```
cargo fmt --check
cargo clippy --workspace --all-targets -- -D warnings
cargo nextest run --workspace     # or: cargo test --workspace
```

The tests include the repository gates (file headers and size, Markdown placement, permissive
licenses and pure-Rust dependencies, the browser-safe clock); CI runs the same commands plus a
`wasm32-unknown-unknown` build of every library crate.

## License

BSD-2-Clause ([LICENSE](LICENSE)). Third-party code the crates adapt is listed, by the crate
that carries it, in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
