//! The corner cap's two switches as one build holds them: the hole fill ([`use_hole_fill`], the
//! corner holes closed by the one hole filler instead of the fans and rings; its geometry readings
//! are in `corner_cap_hole_fill.rs`) and flat ends (`corner_cap_flat_ends::use_flat_ends`), both on
//! unless the build says otherwise.
//!
//! The choice is the request's (`BevelRouting::corner_hole_fill` and `BevelRouting::flat_ends`):
//! the surface part builder holds them for its build ([`hold_corner_cap_switches`]), so every
//! corner and end the level line closes under that build reads them without each layer between
//! the builder and the corner cap carrying two more arguments - the same way the build's work
//! budget reaches them (`LevelLineWork`). Per thread, because a build runs on one, and so a test
//! that turns a switch off never changes what a parallel test builds. A test that calls the level
//! line directly, with no builder, sets them itself through the same per-thread values
//! ([`set_use_hole_fill_override`], `corner_cap_flat_ends::set_use_flat_ends_override`).

use std::cell::Cell;

use super::corner_cap_flat_ends::{set_use_flat_ends_override, use_flat_ends_override};

thread_local! {
    /// This thread's hole-fill setting - `None` leaves it on.
    static USE_HOLE_FILL_OVERRIDE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// Sets (or, with `None`, clears back to on) this thread's [`use_hole_fill`].
pub fn set_use_hole_fill_override(value: Option<bool>) {
    USE_HOLE_FILL_OVERRIDE.with(|o| o.set(value));
}

/// Whether the corner cap hands each corner hole to the hole filler.
pub fn use_hole_fill() -> bool {
    USE_HOLE_FILL_OVERRIDE.with(|o| o.get()).unwrap_or(true)
}

/// Sets both corner cap switches on this thread until the returned scope drops, which puts back
/// what was there before.
pub fn hold_corner_cap_switches(flat_ends: bool, hole_fill: bool) -> CornerCapSwitchesScope {
    let scope = CornerCapSwitchesScope {
        flat_ends_was: use_flat_ends_override(),
        hole_fill_was: USE_HOLE_FILL_OVERRIDE.with(|o| o.get()),
        _not_send: std::marker::PhantomData,
    };
    set_use_flat_ends_override(Some(flat_ends));
    set_use_hole_fill_override(Some(hole_fill));
    scope
}

/// Holds one build's corner cap switches ([`hold_corner_cap_switches`]); dropping it puts back the
/// thread's own. It belongs to the thread that made it.
#[derive(Debug)]
#[must_use = "the switches are held only while the scope is alive"]
pub struct CornerCapSwitchesScope {
    flat_ends_was: Option<bool>,
    hole_fill_was: Option<bool>,
    // A scope moved to another thread would restore that thread's switches wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl Drop for CornerCapSwitchesScope {
    fn drop(&mut self) {
        set_use_flat_ends_override(self.flat_ends_was);
        set_use_hole_fill_override(self.hole_fill_was);
    }
}
