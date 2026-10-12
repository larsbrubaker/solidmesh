//! Ported `R/Tests/GeometryProcessingTests` test classes for `Bevel/`, one file each (CLAUDE.md
//! net 1), with the C# fixture builders they need (`Fixtures/*.cs`) as Rust modules.
//!
//! The member modules are listed in group modules under `bevel/`, by C# folder and class prefix, so
//! no list outgrows the file-size gate: `level_line` (`LevelLine/*.cs` and its helpers),
//! `sections` (`Section*`), `surface` (`Surface*`), `kernel` (`Bevel*` and the remaining top-level
//! classes), `rust_only` (`rust_only_bevel_*`) and `shared_fixtures` (`Fixtures/*.cs` and helpers
//! shared across groups). Each group is re-exported here, so every member resolves as
//! `crate::<module>` from any test file; a test's path carries its group
//! (`level_line::chain_field_tests::...`), which `.config/nextest.toml`'s filters match.

#[path = "bevel/kernel.rs"]
mod kernel;
#[path = "bevel/level_line.rs"]
mod level_line;
#[path = "bevel/rust_only.rs"]
mod rust_only;
#[path = "bevel/sections.rs"]
mod sections;
#[path = "bevel/shared_fixtures.rs"]
mod shared_fixtures;
#[path = "bevel/surface.rs"]
mod surface;

use kernel::*;
use level_line::*;
use rust_only::*;
use sections::*;
use shared_fixtures::*;
use surface::*;
