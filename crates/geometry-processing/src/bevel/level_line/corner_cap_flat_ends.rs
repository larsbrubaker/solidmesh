//! The flat-ends switch of the corner cap: an END FACE end closed flat by the flat hole filler
//! instead of the zips and fans, on by default. Ports the switch of
//! `Bevel/LevelLine/CornerCap.FlatEnds.cs` (a partial of the C# static class `CornerCap`; its
//! hole-fill switch is in `corner_cap_hole_fill_switch.rs`); the flat end's closing lands with
//! later packages.

use std::cell::Cell;
use std::sync::OnceLock;

use super::level_line_routing::reads_as_off;

/// C# `CornerCap.UseFlatEndsVariable`: the environment variable that turns [`use_flat_ends`] off
/// ("0" or "false"); unset or anything else leaves it on.
pub const USE_FLAT_ENDS_VARIABLE: &str = "MATTERCAD_BEVEL_FLATENDS";

/// C# `CornerCap.FlatEndsFromTheEnvironment`: read once, when first asked - the developer opt-out.
/// On by default since the owner files built no worse with it and the kernel suite is green on
/// it. The browser has no process environment, so there it is always on.
fn flat_ends_from_the_environment() -> bool {
    static FROM_THE_ENVIRONMENT: OnceLock<bool> = OnceLock::new();
    *FROM_THE_ENVIRONMENT
        .get_or_init(|| !reads_as_off(std::env::var(USE_FLAT_ENDS_VARIABLE).ok().as_deref()))
}

thread_local! {
    /// C# `[ThreadStatic] CornerCap.UseFlatEndsOverride`: a per-thread override of the
    /// environment's answer - `None` leaves it to the variable. Per thread so a test that turns
    /// flat ends off never changes what a parallel test builds.
    static USE_FLAT_ENDS_OVERRIDE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// Sets (or, with `None`, clears) this thread's override of [`use_flat_ends`] (C#'s
/// `UseFlatEndsOverride` setter).
pub fn set_use_flat_ends_override(value: Option<bool>) {
    USE_FLAT_ENDS_OVERRIDE.with(|o| o.set(value));
}

/// This thread's override of [`use_flat_ends`] (C#'s `UseFlatEndsOverride` getter).
pub fn use_flat_ends_override() -> Option<bool> {
    USE_FLAT_ENDS_OVERRIDE.with(|o| o.get())
}

/// C# `CornerCap.UseFlatEnds`: whether an end-face hole is closed by the flat hole filler (and its
/// end ray bounded at the end plane).
pub fn use_flat_ends() -> bool {
    use_flat_ends_override().unwrap_or_else(flat_ends_from_the_environment)
}
