//! The bevel kernel's test binary: one file per test class, with the fixture
//! builders they need as modules.
//!
//! The member modules are listed in group modules under `bevel/`, by subject and class prefix, so
//! no list outgrows the file-size gate: `level_line` (the level-line classes and their helpers),
//! `part` (`part_*`), `sections` (`section_*`), `surface` (`surface_*`), `kernel` (`bevel_*` and
//! the remaining top-level classes), `rust_only` (`rust_only_bevel_*`) and `shared_fixtures`
//! (the fixture builders and helpers shared across groups). Each group but `part`, whose files
//! share nothing, is re-exported here, so every member resolves as `crate::<module>` from any
//! test file; a test's path carries its group (`level_line::chain_field_tests::...`), which
//! `.config/nextest.toml`'s filters match.

#[path = "bevel/kernel.rs"]
mod kernel;
#[path = "bevel/level_line.rs"]
mod level_line;
#[path = "bevel/part.rs"]
mod part;
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
