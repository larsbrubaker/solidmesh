# Third-party notices: solidmesh

solidmesh is BSD-2-Clause licensed (see `LICENSE`). Third-party code it ports or adapts is
listed below with its license, which requires these notices to stay with the source and with
binary redistributions; a dependency used as a crate (not copied) is listed for provenance. This
is the `solidmesh` section of the solidmesh repository's `THIRD_PARTY_NOTICES.md`, which the
workspace-gates `notices` test keeps identical to it.

# solidmesh (`crates/geometry-processing`)

## indexmap (MIT OR Apache-2.0)

Carried by: solidmesh (`crates/geometry-processing`)

Source: https://github.com/indexmap-rs/indexmap — a crates.io dependency (not copied) of
`crates/geometry-processing`, which iterates maps and sets in insertion order with it where that
order reaches a result (CLAUDE.md, "Exactness").

## libm (MIT)

Carried by: solidmesh (`crates/geometry-processing`)

Source: https://github.com/rust-lang/compiler-builtins (its `libm` crate) — a crates.io dependency
(not copied) of `crates/geometry-processing`, whose 2D path bevel and topology readings take
`sin`, `cos`, `tan`, `acos` and `atan2` from it in pure Rust, so a design gives the same curves on
every platform. Its license text is in that repository.

## libc, windows-sys, sha2 and zip (test-only)

Carried by: solidmesh (`crates/geometry-processing`), as dev-dependencies only

Sources: https://github.com/rust-lang/libc and https://github.com/microsoft/windows-rs (MIT OR
Apache-2.0), read by the bevel cost tests for the calling thread's CPU time (`clock_gettime` on
Unix, `GetThreadTimes` on Windows); https://github.com/RustCrypto/hashes (`sha2`, MIT OR
Apache-2.0), the cost tests' mesh digests; https://github.com/zip-rs/zip2 (`zip`, MIT), which the
Thingi10K sweep reads the corpus' meshes out of their zip files with. Used as crates (not copied),
by the tests alone: none of them reaches the library. License texts are in each crate's
repository.
