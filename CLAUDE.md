# solidmesh

Solid modeling on triangle meshes in pure Rust: booleans, fillets and chamfers, repair and
implicit (SDF) operations that keep closed solids closed. It is MatterCAD's geometry kernel,
moved out of the app so anyone can use it. `README.md` says what it does and the root
`Cargo.toml` lists the crates; how any part works is in the header of its crate's `lib.rs` and
of each file.

## Never
- **Code that isn't Rust**: no C/C++, no `-sys` crates, no native build scripts. Every library
  crate builds for `wasm32-unknown-unknown`. The `dependencies` gate refuses the rest.
- **A license that isn't permissive** (MIT/BSD/Apache/zlib-compatible). Third-party code goes in
  `THIRD_PARTY_NOTICES.md`, under the crate that carries it, in the same change.
- **Stubs**: no `todo!()`, placeholders or partial implementations. Build a missing dependency
  first.
- **Weakening a test to make it pass.** Every failure is a real bug: find the root cause. An
  expected value changes only when the behaviour is deliberately changed, and the commit says why.
- **Breaking outputs callers rely on without saying so.** A change to the geometry an operation
  returns, a public signature or an error code is a break for someone; the commit names it.

## Exactness
Callers compare outputs bit for bit (MatterCAD's saved designs, golden scenes, boolean inputs).
- `f64` for geometry, in a stable operation order, never `mul_add` (`clippy.toml` refuses it).
- Where map order reaches an output, use `IndexMap`; where sort ties do, give an explicit
  tie-break.
- `Matrix4X4` is row-major with row vectors (`v * M`); never mix in a column convention.

## Working
- **Bug fixes are test-first:** a failing test, the root-cause fix, the test passing.
- **Tests exercise production code**, never copies. Tests live in `crates/<crate>/tests/<area>/`,
  one binary per area (`tests/<area>.rs`), and read files only inside their own crate.
- **Errors are `Result`s**, never swallowed; `unwrap` stays out of library code. A refusal says
  why, with the largest size that would have worked where there is one.
- **Siblings are ours** (manifold-rust, clipper2-rust, tess2-rust, checked out beside this repo):
  fix a gap upstream first, never work around it here. A change solidmesh relies on bumps the
  sibling's version and raises its floor in each `Cargo.toml` that names it.

## Code and docs
- Every file starts with a header saying what it is and how it fits its neighbours. That header
  is the file's documentation: keep it true in the same commit.
- Comments explain why, in this code's own terms, never by citing another implementation.
- A file over 800 lines is split by responsibility, never trimmed.
- There are no design docs: Markdown lives only where `markdown_placement` allows.

## Consumers
MatterCAD (mattercad-rust, checked out beside this repo) depends on these crates by path. A
change that moves output MatterCAD ships says so in its commit, and the matching change
(updated goldens, a raised version floor) lands in mattercad-rust with it.

## Done
`cargo fmt --check`, `cargo clippy --workspace --all-targets -- -D warnings` and
`cargo nextest run --workspace` are green (the workspace-gates crate runs with them), and CI is
green. Each change is one commit whose message says what a library user can now do.
