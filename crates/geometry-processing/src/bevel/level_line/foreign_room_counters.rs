//! The foreign-room reading's meter. Ports `Bevel/LevelLine/ForeignRoomCounters.cs` (a C# static
//! class): so far its clock read ([`clock_now`]); the per-thread tallies and the report land with
//! later packages.

use super::stopwatch;
use super::FieldCounters;

/// A clock read for the room meter - zero, and no clock read, with the meter off.
pub fn clock_now() -> i64 {
    if FieldCounters::meter_on() {
        stopwatch::get_timestamp()
    } else {
        0
    }
}
