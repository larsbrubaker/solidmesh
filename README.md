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
[mattercad-rust](https://github.com/MatterHackers/mattercad-rust) workspace while its public API
is drawn and its MatterCAD-only parts are separated out. Then the crates move here with their
history and are published to crates.io.

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

## License

BSD-2-Clause ([LICENSE](LICENSE)). Third-party code is listed in `THIRD_PARTY_NOTICES.md` as it
arrives.
