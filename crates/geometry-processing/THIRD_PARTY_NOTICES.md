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

## solidcore fillet and chamfer cases (Apache-2.0, test data only)

Carried by: solidmesh (`crates/geometry-processing`), in its tests alone

Source: https://github.com/tangency-labs/solidcore, `fixtures/solidcraft/` at commit
c8f07fd3ae6a5b6d838d26b6f72ea9a438633d92 (2026-11-28). The case files that hold a fillet or
chamfer are copied unchanged into `crates/geometry-processing/tests/solidcore_cases/fixtures/`
(`pin.json` beside them records the commit and each file's SHA-256), and the `solidcore_cases`
test binary replays them against solidmesh's own bevel. No solidcore code is copied or run; the
files are excluded from the published crate (`exclude = ["tests/"]`). They are used under the
Apache License, Version 2.0, whose full text, as solidcore ships it, is copied beside them at
`crates/geometry-processing/tests/solidcore_cases/LICENSE-APACHE`. solidcore's NOTICE reads, in
the part that covers its own work:

    solidcore
    Copyright (c) 2026 The solidcore contributors

    This product is a Rust port (a derivative work) of the SMLib and SM_API
    components of USD BRep, https://github.com/NVIDIA-Omniverse/usd-brep
    (v1.0.1, commit 3fd5b95), licensed under the Apache License, Version 2.0.

    USD BRep is Copyright (c) 2020-2026, NVIDIA CORPORATION & AFFILIATES. All
    rights reserved. NVIDIA-authored portions of this product are licensed under
    the Apache License, Version 2.0, except where otherwise noted.
