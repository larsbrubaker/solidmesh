//! A measurement only: what the per-split needle repair does on each call - how many repairs one
//! call makes, what made the triangles it repairs (the input, a refinement split, or an earlier
//! repair), and how thin they were - so a cascade can be read for what it is made of. Ports
//! `Bevel/LevelLine/NeedleMeter.cs` (a C# static class): what made a triangle
//! ([`NeedleMeterMadeBy`]) and the meter itself ([`NeedleMeter`]).
//!
//! Off unless a caller calls [`NeedleMeter::begin`] AND `FieldCounters` meter is on; the hooks on
//! the mesh read the meter switch first, so a build pays one static bool read for them. Not per
//! thread, like the split meter: one caller turns it on round one build. C#'s plain statics are one
//! process-wide `Mutex`.

use std::collections::{HashMap, HashSet};
use std::sync::{Mutex, MutexGuard};

use super::stopwatch::{g4, n0};
use super::FieldCounters;

/// C# `NeedleMeter.MadeBy` (a `byte` enum): what put a triangle at an index of the mesh, as the
/// meter last saw it.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum NeedleMeterMadeBy {
    /// An input triangle no split or repair has rewritten.
    #[default]
    Input = 0,

    /// Added or rewritten by anything other than the needle repair - a refinement split.
    Split,

    /// One of the two triangles a needle repair left behind.
    Repair,
}

/// The first this many repairs are recorded in full; the rest are only counted.
pub const NEEDLE_METER_CAP: usize = 20;

const HEIGHT_BANDS: [&str; 5] = ["< 1x", "1-10x", "10-100x", "100-1000x", "> 1000x"];

const CALL_BANDS: [&str; 6] = ["0", "1", "2-10", "11-100", "101-1000", "> 1000"];

const MADE_BY: [NeedleMeterMadeBy; 3] = [
    NeedleMeterMadeBy::Input,
    NeedleMeterMadeBy::Split,
    NeedleMeterMadeBy::Repair,
];

/// The meter's process-wide state (C#'s private statics).
#[derive(Debug, Default)]
struct NeedleMeterState {
    begun: bool,
    input_count: i32,
    size: f64,
    made: HashMap<i32, NeedleMeterMadeBy>,
    calls_by: Vec<i64>,
    /// Per [`NeedleMeterMadeBy`], per height band (C# `long[3, HeightBands.Length]`).
    repaired_by: Vec<[i64; 5]>,
    calls_with_no_needle: i64,
    whole_pass_repairs: i64,
    calls: i64,
    repairs: i64,
    longest: i64,
    longest_call: i64,
    longest_distinct_apexes: i32,
    children_under_the_floor: i64,
    in_this_call: i64,
    apexes_this_call: HashSet<i32>,
    first: Vec<String>,
    opening_this_call: Vec<String>,
    opening_of_the_longest: Vec<String>,
}

static STATE: Mutex<Option<NeedleMeterState>> = Mutex::new(None);

fn state() -> MutexGuard<'static, Option<NeedleMeterState>> {
    // A panic while metering leaves numbers, not invariants, behind: read on.
    STATE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn with<R>(f: impl FnOnce(&mut NeedleMeterState) -> R) -> R {
    f(state().get_or_insert_with(NeedleMeterState::default))
}

/// C# `NeedleMeter`: the needle repair's measurement (see the module docs).
pub struct NeedleMeter;

impl NeedleMeter {
    /// Whether the meter is counting: begun, and the field counters' meter on.
    pub fn on() -> bool {
        with(|s| s.begun) && FieldCounters::meter_on()
    }

    /// Starts the meter for a mesh built from `input_triangles` input triangles.
    pub fn begin(input_triangles: i32, pick_size: f64) {
        with(|s| {
            *s = NeedleMeterState {
                begun: true,
                input_count: input_triangles,
                size: pick_size,
                calls_by: vec![0; CALL_BANDS.len()],
                repaired_by: vec![[0; 5]; MADE_BY.len()],
                ..NeedleMeterState::default()
            };
        });
    }

    /// Stops the meter; the tallies stay for the report.
    pub fn end() {
        with(|s| {
            s.begun = false;
            s.made = HashMap::new();
            s.apexes_this_call = HashSet::new();
        });
    }

    /// A triangle was ADDED at this index by anything but a repair. An index under the input count
    /// is the mesh being built from the input - the build adds every input triangle through the
    /// same call a split does - so it stays Input until something actually rewrites it; stamping
    /// it Split here is what left the Input column empty.
    pub fn note_added(index: i32) {
        if Self::on() {
            with(|s| {
                if index >= s.input_count {
                    s.made.insert(index, NeedleMeterMadeBy::Split);
                }
            });
        }
    }

    /// A triangle was rewritten at this index by anything but a repair.
    pub fn note_written(index: i32) {
        if Self::on() {
            with(|s| {
                s.made.insert(index, NeedleMeterMadeBy::Split);
            });
        }
    }

    /// A per-split call found no triangle under the floor among those handed in.
    pub fn note_no_needle() {
        if Self::on() {
            with(|s| {
                s.calls += 1;
                s.calls_with_no_needle += 1;
            });
        }
    }

    /// A repair pass is starting.
    pub fn start_call() {
        if !Self::on() {
            return;
        }

        with(|s| {
            s.in_this_call = 0;
            s.apexes_this_call.clear();
            s.opening_this_call.clear();
        });
    }

    /// A repair pass ended; `around` false is the whole-mesh pass, kept apart.
    pub fn end_call(around: bool) {
        if !Self::on() {
            return;
        }

        with(|s| {
            if !around {
                s.whole_pass_repairs += s.in_this_call;
                return;
            }

            s.calls += 1;
            let n = s.in_this_call;
            let band = if n == 0 {
                0
            } else if n == 1 {
                1
            } else if n <= 10 {
                2
            } else if n <= 100 {
                3
            } else if n <= 1000 {
                4
            } else {
                5
            };
            s.calls_by[band] += 1;
            if n > s.longest {
                s.longest = n;
                s.longest_call = s.calls;
                s.longest_distinct_apexes = s.apexes_this_call.len() as i32;
                s.opening_of_the_longest = s.opening_this_call.clone();
            }
        });
    }

    /// One needle about to be repaired, read BEFORE the mesh changes: its index, apex, height and
    /// longest edge.
    #[allow(clippy::too_many_arguments)] // C#'s signature: one reading per argument.
    pub fn note_repair(
        needle: i32,
        apex: i32,
        long_from: i32,
        long_to: i32,
        across: i32,
        height: f64,
        longest_edge: f64,
        floor: f64,
    ) {
        if !Self::on() {
            return;
        }

        with(|s| {
            s.repairs += 1;
            s.in_this_call += 1;
            s.apexes_this_call.insert(apex);
            let by = match s.made.get(&needle) {
                Some(&kind) => kind,
                None if needle < s.input_count => NeedleMeterMadeBy::Input,
                None => NeedleMeterMadeBy::Split,
            };
            let ratio = if floor > 0.0 {
                height / floor
            } else {
                f64::INFINITY
            };
            s.repaired_by[by as usize][Self::band(ratio) as usize] += 1;
            let said = format!(
                "call {} triangle {needle} made by {by:?}, apex {apex} on long edge \
                 ({long_from},{long_to}) re-tiling {across}, height/floor {}, longest edge/size {}",
                s.calls + 1,
                g4(ratio),
                g4(longest_edge / s.size)
            );
            if s.first.len() < NEEDLE_METER_CAP {
                s.first.push(format!("  #{} {said}", s.first.len() + 1));
            }

            if s.opening_this_call.len() < NEEDLE_METER_CAP {
                let at = s.opening_this_call.len() + 1;
                s.opening_this_call.push(format!("  #{at} {said}"));
            }
        });
    }

    /// One of the two triangles a repair left: marked as the repair's, and counted where under the
    /// floor.
    pub fn note_repair_left(triangle: i32, under_the_floor: bool) {
        if !Self::on() {
            return;
        }

        with(|s| {
            s.made.insert(triangle, NeedleMeterMadeBy::Repair);
            if under_the_floor {
                s.children_under_the_floor += 1;
            }
        });
    }

    /// The band of a height in floor units: under 1, 1-10, 10-100, 100-1000, over 1000.
    pub fn band(ratio: f64) -> i32 {
        if ratio < 1.0 {
            0
        } else if ratio < 10.0 {
            1
        } else if ratio < 100.0 {
            2
        } else if ratio < 1000.0 {
            3
        } else {
            4
        }
    }

    /// The meter's report. Panics where C# throws: asked before any [`NeedleMeter::begin`].
    pub fn report() -> String {
        with(|s| {
            let mut lines = vec![
                format!(
                    "needle meter: {} repairs; {} per-split calls, {} of them with no needle among \
                     the triangles handed in; whole-mesh pass repaired {}",
                    n0(s.repairs),
                    n0(s.calls),
                    n0(s.calls_with_no_needle),
                    n0(s.whole_pass_repairs)
                ),
                format!(
                    "  repairs per per-split call that found a needle: {}",
                    indices(CALL_BANDS.len(), |i| format!(
                        "{}: {}",
                        CALL_BANDS[i],
                        n0(s.calls_by[i])
                    ))
                    .join(", ")
                ),
                format!(
                    "  longest cascade {} repairs in call {}, through {} distinct apexes",
                    n0(s.longest),
                    n0(s.longest_call),
                    n0(s.longest_distinct_apexes as i64)
                ),
                format!(
                    "  repair children still under the floor: {} of {}",
                    n0(s.children_under_the_floor),
                    n0(2 * s.repairs)
                ),
            ];

            for by in MADE_BY {
                let row = s.repaired_by[by as usize];
                let total: i64 = row.iter().sum();
                lines.push(format!(
                    "  repaired needles made by {by:?}: {}; height/floor {}",
                    n0(total),
                    indices(HEIGHT_BANDS.len(), |i| format!(
                        "{}: {}",
                        HEIGHT_BANDS[i],
                        n0(row[i])
                    ))
                    .join(", ")
                ));
            }

            lines.push("  first repairs:".to_string());
            lines.extend(s.first.iter().cloned());
            lines.push("  first repairs of the longest cascade:".to_string());
            lines.extend(s.opening_of_the_longest.iter().cloned());
            lines.join("\n")
        })
    }
}

/// `say(i)` for every `i` below `count`.
fn indices(count: usize, say: impl Fn(usize) -> String) -> Vec<String> {
    (0..count).map(say).collect()
}
