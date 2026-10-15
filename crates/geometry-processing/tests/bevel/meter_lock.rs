//! The process-wide lock the metering tests take: the needle and split meters keep one state for
//! the whole test process, so two such tests running at once would read each other's counts.
//! (`FieldCounters`' own switches and tallies are per thread and need no lock.)

use std::sync::{Mutex, MutexGuard};

static METER: Mutex<()> = Mutex::new(());

/// Holds the meter for the caller until the guard drops. A test that panicked while holding it
/// leaves the meter as it was, so a poisoned lock is simply taken.
pub fn meter_lock() -> MutexGuard<'static, ()> {
    METER
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}
