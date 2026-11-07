//! workspace-gates: a test-only crate. Its gates live in `tests/`: `file_compliance.rs` (the
//! 800-non-empty-line limit, header comments and merge-conflict markers across the whole
//! repository), `markdown_placement.rs` (Markdown only where it isn't documentation of code;
//! both walk the repository through `tests/support/repo_walk.rs`), `workspace_members.rs` (every
//! crate under `crates/` is a member and takes the workspace lints), `web_clock.rs` (library code
//! reads the clock through `web_time`, never std's, which panics on wasm32), `scope_reads.rs`
//! (a crate's tests read files only inside that crate, through `tests/support/source_reads.rs`)
//! and `dependencies.rs` (every dependency is permissively licensed and compiles no native code).
//! Each gate walks the workspace rather than naming crates, so a crate that moves in is covered
//! at once, and each gate's failure message teaches the rule it holds. There is deliberately no
//! library code here.
