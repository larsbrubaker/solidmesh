# solidmesh

**Solid modeling on triangle meshes, in pure Rust: booleans, fillets and chamfers, repair, and
implicit (SDF) operations that keep closed solids closed.**

[![Live WASM demo](README_HERO.png)](https://larsbrubaker.github.io/solidmesh/)

<p align="center"><a href="https://larsbrubaker.github.io/solidmesh/"><b>▶ Try the live demo</b></a></p>

solidmesh is the geometry kernel behind MatterCAD, moved out of the application into its own
library so that others can use it and build on it. It works on the meshes real pipelines contain
(STL and 3MF downloads, scans, triangle soup) as well as on clean modeled parts, builds for
`wasm32-unknown-unknown`, contains no C or C++, and gives the same answer on every platform.

## Status

**The kernel lives here; it is not on crates.io yet.** The four crates moved in from the
[mattercad-rust](https://github.com/MatterHackers/mattercad-rust) workspace with their history,
and MatterCAD now depends on them by path, from a solidmesh checkout beside its own. The work
still open happens here: results as codes only (the application words them), the fillet
orchestration moving in from the app behind one entry point, the public API (everything else
crate-private), and then publishing to crates.io. Until then the crates are `publish = false`
and their API changes with MatterCAD's needs.

## Crates

| Crate | Folder | What it holds |
|---|---|---|
| `solidmesh-math` | `crates/vector-math` | Vectors, matrices, quaternions, planes, rays and bounds in `f64` and `f32`; a BVH; colours, rectangles and the progress reporter the kernel shares with its callers |
| `solidmesh-clipper` | `crates/clipper-library` | A Clipper 6.4.2 port (integer polygon booleans, offsets, simplification) and the polygon helpers on clipper2-rust's types that the kernel slices, extrudes and revolves with |
| `solidmesh-mesh` | `crates/polygon-mesh` | The triangle mesh; welding, winding, measures; booleans on [manifold-rust](https://github.com/larsbrubaker/manifold-rust); repair; SDF and implicit meshing; convex hull and Minkowski sums; extrusion and revolution of 2D outlines; slicing; ray tracing; STL |
| `solidmesh` | `crates/geometry-processing` | Fillets and chamfers on mesh edges, corners and outlines |

Each folder and library name is the one MatterCAD has always used (`use vector_math::...`,
`use polygon_mesh::...`), so code reads the same in both repositories; the package name is the
one the crate will publish under.

## Demo

The [live demo](https://larsbrubaker.github.io/solidmesh/) runs the library itself in the browser,
compiled to WebAssembly: fillets and chamfers through their one entry point,
`bevel::part::bevel_part` (pick edge chains, single edges, face rims or corners on a part, set
one radius for them all, and see the part rounded in one call; a pick that does not fit is refused
with the size that would), booleans (including inside-out and self-intersecting operands), outlines
extruded with bevels or revolved, smooth implicit blends meshed by marching cubes or dual
contouring, Minkowski sums and convex hulls, mesh repair, and a part to measure and download as
STL. Each page shows the operation's time and the result's triangle count, volume and area.

It lives in `demo/`: `demo/wasm` is the bindings crate (a workspace member, never published), the
pages are TypeScript built with [Bun](https://bun.sh), and `.github/workflows/deploy-demo.yml`
publishes it to GitHub Pages on every push to `main`. To run it locally, from `demo/`:

```
bun install
bun run build:wasm     # wasm-pack build into demo/public/pkg
bun run dev            # http://localhost:3030
```

With the dev server running, `bun run capture:thumbs` renders the page thumbnails and
`bun run capture:hero` screenshots the home page into `README_HERO.png`, both through a headless
Edge or Chrome (`demo/capture-thumbs.ts` says how).

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
