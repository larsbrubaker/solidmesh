//! The hole-fill switch of the corner cap: the corner holes closed by the one hole filler instead
//! of the fans and rings, on by default. Ports the switch of `Bevel/LevelLine/CornerCap.HoleFill.cs`
//! (a partial of the C# static class `CornerCap`; its geometry readings are in
//! `corner_cap_hole_fill.rs`, its flat-ends switch in `corner_cap_flat_ends.rs`).

use std::cell::Cell;
use std::sync::OnceLock;

use super::level_line_routing::reads_as_off;

/// C# `CornerCap.UseHoleFillVariable`: the environment variable that turns [`use_hole_fill`] off
/// ("0" or "false"); unset or anything else leaves it on.
pub const USE_HOLE_FILL_VARIABLE: &str = "MATTERCAD_BEVEL_HOLEFILL";

/// C# `CornerCap.HoleFillFromTheEnvironment`: read once, when first asked - the developer opt-out.
/// On by default since stage 3's setbacks leave every corner row flat at radius r, so the
/// filler's patch is small at every size. The browser has no process environment, so there it is
/// always on.
fn hole_fill_from_the_environment() -> bool {
    static FROM_THE_ENVIRONMENT: OnceLock<bool> = OnceLock::new();
    *FROM_THE_ENVIRONMENT
        .get_or_init(|| !reads_as_off(std::env::var(USE_HOLE_FILL_VARIABLE).ok().as_deref()))
}

thread_local! {
    /// C# `[ThreadStatic] CornerCap.UseHoleFillOverride`: a per-thread override of the
    /// environment's answer - `None` leaves it to the variable. Per thread so a test that turns
    /// the filler on never changes what a parallel test builds.
    static USE_HOLE_FILL_OVERRIDE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// Sets (or, with `None`, clears) this thread's override of [`use_hole_fill`] (C#'s
/// `UseHoleFillOverride` setter).
pub fn set_use_hole_fill_override(value: Option<bool>) {
    USE_HOLE_FILL_OVERRIDE.with(|o| o.set(value));
}

/// C# `CornerCap.UseHoleFill`: whether the corner cap hands each corner hole to the hole filler.
pub fn use_hole_fill() -> bool {
    USE_HOLE_FILL_OVERRIDE
        .with(|o| o.get())
        .unwrap_or_else(hole_fill_from_the_environment)
}
