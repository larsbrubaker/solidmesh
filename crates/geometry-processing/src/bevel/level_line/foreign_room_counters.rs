//! The foreign-room reading's meter: its per-thread tallies ([`ForeignRoomTally`], one
//! `thread_local!`, as `field_counters.rs` keeps `FieldCounters`'), its clock read
//! ([`clock_now`]), the edge charge ([`read`]), the ticks since a reading began ([`since`]), the
//! zeroing ([`reset`]) and the one-line report ([`report`]).

use std::cell::RefCell;

use super::stopwatch;
use super::FieldCounters;

thread_local! {
    static TALLY: RefCell<ForeignRoomTally> = RefCell::new(ForeignRoomTally::default());
}

/// One thread's tally of what the FOREIGN reading of a pick costs,
/// split off the whole reading's work total - instrumentation for the many-chain cost work,
/// zeroed by a test.
///
/// Counted PER THREAD, as `FieldCounters` is and for the same reason: the kernel never forks a
/// reading, so a thread's tally is exactly one caller's, where a process-wide one would add in
/// whatever test is running beside it. Being per call and per edge rather than per second, these
/// read the same on a loaded machine as on an idle one.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ForeignRoomTally {
    /// How many times `ForeignEdgeRoom.RoomFor` was entered with edges to read.
    pub calls: i64,
    /// How many foreign edges those calls handed the field, summed over the calls.
    pub edges_offered: i64,
    /// How many of those edges were actually asked of the field (a closest approach each).
    pub edges_read: i64,
    /// Of `calls`, the ones reading ANOTHER CHAIN's edges rather than the part's.
    pub pair_calls: i64,
    /// Of `edges_read`, the ones read for a chain-to-chain pair.
    pub pair_edges_read: i64,
    /// Set by `BandRoom` round its chain-to-chain call so the two readings can be told apart.
    pub in_a_pair: bool,
    /// Work units spent in the foreign reading a pick takes.
    pub foreign_units: i64,
    /// Work units spent in the chain-to-chain reading.
    pub pair_units: i64,
    /// Work units spent in the self reading.
    pub self_units: i64,
    /// How many foreign edges each PER-CHAIN reading was handed, one entry per chain (`None` until
    /// the first [`reset`]).
    pub edges_per_chain: Option<Vec<i32>>,
    /// Of `edges_read`, the ones read by the nearest-first SEARCH over the index rather than by a
    /// trial size of the bisection that follows it.
    pub search_edges_read: i64,
    /// How many edges those searches left for the bisection, summed over the per-chain calls.
    pub kept_edges: i64,
    /// How many readings fell back off the one bare pass to the doubling-and-bisection path.
    pub bracket_fallbacks: i64,
    /// Of `bracket_fallbacks`, the ones because a foreign edge already STANDS on the chain.
    pub touching_fallbacks: i64,
    /// Stopwatch ticks of the foreign reading, kept only under `FieldCounters.MeterOn`.
    pub foreign_ticks: i64,
    /// Stopwatch ticks of the chain-to-chain reading.
    pub pair_ticks: i64,
    /// Stopwatch ticks of the self reading.
    pub self_ticks: i64,
}

/// Runs `f` on this thread's tally. Never call another function of this module from inside `f`.
pub fn with<R>(f: impl FnOnce(&mut ForeignRoomTally) -> R) -> R {
    TALLY.with(|tally| f(&mut tally.borrow_mut()))
}

/// One foreign edge asked of the field, charged to whichever
/// reading is running.
pub fn read() {
    with(|tally| {
        tally.edges_read += 1;
        if tally.in_a_pair {
            tally.pair_edges_read += 1;
        }
    });
}

/// A clock read for the room meter - zero, and no clock read, with the meter off.
pub fn clock_now() -> i64 {
    if FieldCounters::meter_on() {
        stopwatch::get_timestamp()
    } else {
        0
    }
}

/// The ticks since `started_at`, or zero with the meter off (a
/// start of zero).
pub fn since(started_at: i64) -> i64 {
    if started_at == 0 {
        0
    } else {
        stopwatch::get_timestamp() - started_at
    }
}

/// Zeroes this thread's tally, with an empty per-chain list.
pub fn reset() {
    with(|tally| {
        *tally = ForeignRoomTally {
            edges_per_chain: Some(Vec::new()),
            ..ForeignRoomTally::default()
        };
    });
}

/// The calls and edges in one line.
pub fn report() -> String {
    with(|tally| {
        format!(
            "calls {} (pairs {}), edges offered {}, edges read {} (pairs {})",
            tally.calls,
            tally.pair_calls,
            tally.edges_offered,
            tally.edges_read,
            tally.pair_edges_read
        )
    })
}
