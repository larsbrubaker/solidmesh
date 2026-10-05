//! The process-wide lock every test that switches or reads `FieldCounters`' global meter takes:
//! the meter is one static for the whole test process (as C#'s), so two such tests running at
//! once would see each other's switch.

use std::sync::{Mutex, MutexGuard};

static METER: Mutex<()> = Mutex::new(());

/// Holds the meter for the caller until the guard drops. A test that panicked while holding it
/// leaves the meter as it was, so a poisoned lock is simply taken.
pub fn meter_lock() -> MutexGuard<'static, ()> {
    METER
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}
