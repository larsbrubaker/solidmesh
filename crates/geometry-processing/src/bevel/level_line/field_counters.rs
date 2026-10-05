//! How many of each query a level-line build asked, by the refinement rule that asked it. Ports
//! `Bevel/LevelLine/FieldCounters.cs`: the rules ([`LevelLineRule`]), the per-thread tally
//! ([`FieldCounterTally`], C#'s `[ThreadStatic]` fields in one `thread_local!`), the two
//! process-wide switches and the counting methods of the static class ([`FieldCounters`]). The
//! rule, bucket and reach reports, the rule tallies the refinement charges (`Fired`, `NoteStep`,
//! `AskedBy`, ...) and `Reset` land with later packages as further `impl` blocks.
//!
//! Counted PER THREAD. The kernel never forks a build, so a thread's tally is exactly one caller's
//! build - where one process-wide tally would count whatever build is running BESIDE it, which is
//! every other test in a parallel run and made the same build read a different number of queries
//! from one run to the next.

use std::cell::RefCell;
use std::sync::atomic::{AtomicBool, Ordering};

use indexmap::IndexMap;

use super::stopwatch;
use super::{LevelLineWork, LevelLineWorkPhase};

/// C# `LevelLineRule`: the refinement rules a search can be charged to, by the names the rules
/// carry in `BandRefinement`'s own remarks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LevelLineRule {
    /// Nothing named this search - it was not asked from inside a rule.
    #[default]
    None = 0,

    /// (i) an edge with both ends outside whose least value dips below the line.
    EdgeDip,

    /// (i, mirrored) an edge with both ends inside whose greatest value rises above it.
    EdgeBump,

    /// (i, third face) an edge whose two ends read DIFFERENT sides, asked whether it crosses the
    /// line more than the once its ends admit to - the rule that makes "one root per crossed edge"
    /// true.
    MixedEdge,

    /// (ii) a triangle wholly outside hiding a closed loop in its middle.
    HiddenLoop,

    /// A triangle wholly inside hiding an island of kept surface.
    Island,

    /// A ridge of kept surface crossing a triangle whose corners are all outside.
    Ridge,

    /// (iii) the chord of an already-crossed triangle, held to the chord tolerance.
    ChordAccuracy,

    /// The sliver guard: a split some other rule asked for that would have thinned a child past
    /// `RefinementSplits.SliverAspectLimit`, taken instead as a longest-edge bisection. Last, so
    /// adding it moved no other rule's row.
    SliverGuard,
}

/// Every rule in discriminant order (C# `Enum.GetValues(typeof(LevelLineRule))`).
pub const ALL_RULES: [LevelLineRule; 9] = [
    LevelLineRule::None,
    LevelLineRule::EdgeDip,
    LevelLineRule::EdgeBump,
    LevelLineRule::MixedEdge,
    LevelLineRule::HiddenLoop,
    LevelLineRule::Island,
    LevelLineRule::Ridge,
    LevelLineRule::ChordAccuracy,
    LevelLineRule::SliverGuard,
];

/// The sliver-aspect limits the would-thin meter reads side by side, so one build says how often
/// the guard would fire at each - the measurement `RefinementSplits.SliverAspectLimit` was chosen by.
pub const WOULD_THIN_LIMITS: [f64; 3] = [32.0, 64.0, 128.0];

/// The span buckets a search is filed under - the asked triangle's or edge's longest side over the
/// SIZE, which is what decides how deep a bounded search has to go. Reading the cost by bucket
/// says whether a build is paying for a few enormous searches on coarse triangles or for many
/// small ones along the line.
pub const BUCKET_EDGES: [f64; 6] = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0];

/// How far from the LINE a search's piece stands, in sizes - the field value at a corner over the
/// size. Filed twice per asking: by the NEAREST corner and by the FARTHEST, which together say
/// whether a build's search cost is spent on pieces a band could reach or on the far ends of
/// pieces that only touch it.
pub const REACH_EDGES: [f64; 4] = [1.0, 2.0, 4.0, 16.0];

/// C# `FieldCounters.MeterOn`: switches on the per-RULE meter. A plain process-wide static in the
/// C#, not a thread-static; the tallies themselves are per thread.
static METER_ON: AtomicBool = AtomicBool::new(false);

/// C# `FieldCounters.CountBySite`: switches on the per-call-site tally of `Evaluate`.
static COUNT_BY_SITE: AtomicBool = AtomicBool::new(false);

thread_local! {
    static TALLY: RefCell<FieldCounterTally> = RefCell::new(FieldCounterTally::default());
}

/// C# `FieldCounters`'s `[ThreadStatic]` fields: one thread's tally. The public counters are the
/// ones the field and the mesh charge directly (`FieldCounters.Evaluate++`); the per-rule rows
/// start empty and are sized on first use (C# `row ??= new long[...]`).
#[derive(Debug, Clone, Default)]
pub struct FieldCounterTally {
    /// Point queries the chain fields were asked.
    pub evaluate: i64,
    /// How many POINT queries the pick's field was asked - against which `evaluate`, the member
    /// chains those cost, says how many members one query has to read before it knows the least.
    pub pick_evaluate: i64,
    pub min_along: i64,
    pub min_tri: i64,
    pub max_along: i64,
    pub max_tri: i64,
    pub segment_searches: i64,
    /// How many edge roots were sought, against which the queries they cost are read.
    pub roots: i64,
    /// How many of those segment searches were answered in closed form rather than by the convex
    /// search.
    pub closed_forms: i64,
    /// How many needles the mesh mended into T-vertices.
    pub needles_repaired: i64,
    /// How many it found and could not mend: a boundary one, an apex at an end, the slivers repair
    /// leaves.
    pub needles_left_alone: i64,
    /// How many it left standing because mending would have deleted a CHAIN edge.
    pub needles_on_a_chain_edge: i64,
    /// How many repair passes stopped at their own bound instead of reaching a fixpoint.
    pub repair_hit_its_bound: i64,
    /// Which rule's search is running - set by the rule, read by the searches it costs.
    pub current_rule: LevelLineRule,
    /// Stopwatch ticks spent inside the pick field's point query, read only under the meter.
    pub pick_evaluate_ticks: i64,
    /// Member chains read inside the pick field's point query, counted only under the meter.
    pub pick_member_reads: i64,
    /// The per-rule, per-bucket and per-reach rows the reports read, sized on first use; the rule
    /// clock; the worklist history; the root-cell reading; one line per refinement call.
    pub asked: Vec<i64>,
    pub fired: Vec<i64>,
    pub steps: Vec<i64>,
    pub evaluates: Vec<i64>,
    pub deepest: Vec<i32>,
    pub convex_retired: Vec<i64>,
    pub convex_retired_root: Vec<i64>,
    pub would_thin: Vec<i64>,
    pub units: Vec<i64>,
    pub rule_ticks: Vec<i64>,
    pub rule_clock_at: i64,
    pub rule_clock_rule: LevelLineRule,
    pub current_bucket: i32,
    pub bucket_asked: Vec<i64>,
    pub bucket_fired: Vec<i64>,
    pub bucket_steps: Vec<i64>,
    pub bucket_deepest: Vec<i32>,
    pub dequeues: i64,
    pub deepest_split: i32,
    pub progress: Vec<String>,
    pub near_reach: i32,
    pub far_reach: i32,
    pub near_asked: Vec<i64>,
    pub near_fired: Vec<i64>,
    pub near_steps: Vec<i64>,
    pub far_asked: Vec<i64>,
    pub far_fired: Vec<i64>,
    pub far_steps: Vec<i64>,
    pub root_cells: Vec<i64>,
    pub root_same_chain: Vec<i64>,
    pub root_corner_max: Vec<i64>,
    pub root_both: Vec<i64>,
    pub root_both_and_searched: Vec<i64>,
    pub root_retired: Vec<i64>,
    pub refinements: Vec<String>,
    /// The per-site tally of this thread's build, in first-charged order.
    pub by_site: IndexMap<String, i64>,
}

/// C# `FieldCounters`: how many of each query a build asked - instrumentation for the speed work,
/// zeroed by a test. Its state is the thread's [`FieldCounterTally`].
pub struct FieldCounters;

impl FieldCounters {
    /// Runs `f` on this thread's tally. Never call a `FieldCounters` method from inside `f`.
    pub fn with<R>(f: impl FnOnce(&mut FieldCounterTally) -> R) -> R {
        TALLY.with(|tally| f(&mut tally.borrow_mut()))
    }

    /// Whether the per-rule meter is on (C# `FieldCounters.MeterOn`): which rule asked each
    /// search, how many found nothing, and what the bounded searches cost in subdivisions and
    /// depth. One plain static read per subdivision step when it is off, and nothing else.
    pub fn meter_on() -> bool {
        METER_ON.load(Ordering::Relaxed)
    }

    /// Switches the per-rule meter on or off for every thread.
    pub fn set_meter_on(on: bool) {
        METER_ON.store(on, Ordering::Relaxed);
    }

    /// Whether the per-call-site tally of `evaluate` is on (C# `FieldCounters.CountBySite`). Off by
    /// default: knowing HOW MANY point queries a build asks says nothing about which code asks them.
    pub fn count_by_site() -> bool {
        COUNT_BY_SITE.load(Ordering::Relaxed)
    }

    /// Switches the per-call-site tally on or off for every thread.
    pub fn set_count_by_site(on: bool) {
        COUNT_BY_SITE.store(on, Ordering::Relaxed);
    }

    /// Charges one needle the mesh looked at, mended or not. A build that repairs thousands is a
    /// build whose input tessellation is the thing to look at, which is why the two are counted
    /// apart.
    pub fn note_needle(repaired: bool) {
        if !Self::meter_on() {
            return;
        }

        Self::with(|t| {
            if repaired {
                t.needles_repaired += 1;
            } else {
                t.needles_left_alone += 1;
            }
        });
    }

    /// Charges one repair pass that ran out its bound. Never seen; metered so it cannot be silent,
    /// since a sweep reads the counters and not the mesh that carried the flag.
    pub fn note_repair_hit_its_bound() {
        if Self::meter_on() {
            Self::with(|t| t.repair_hit_its_bound += 1);
        }
    }

    /// Charges one needle left standing because mending it would have deleted a chain edge.
    pub fn note_needle_on_a_chain_edge() {
        if !Self::meter_on() {
            return;
        }

        Self::with(|t| {
            t.needles_left_alone += 1;
            t.needles_on_a_chain_edge += 1;
        });
    }

    /// How many cells the Lipschitz bound would have kept alive that the convex bound retires, and
    /// how many of those were a search's ROOT cell - what the convex cell bound is worth, per rule.
    pub fn note_convex_bound(retired: bool, depth: i32) {
        if !retired {
            return;
        }

        Self::with(|t| {
            let rule = t.current_rule as usize;
            rows(&mut t.convex_retired)[rule] += 1;
            if depth == 0 {
                rows(&mut t.convex_retired_root)[rule] += 1;
            }
        });
    }

    /// Counts a split a rule made that would mint a new sliver at the `limit`-th of
    /// [`WOULD_THIN_LIMITS`].
    pub fn note_would_thin(rule: LevelLineRule, limit: i32) {
        Self::with(|t| {
            if t.would_thin.is_empty() {
                t.would_thin = vec![0; ALL_RULES.len() * WOULD_THIN_LIMITS.len()];
            }

            t.would_thin[(rule as usize * WOULD_THIN_LIMITS.len()) + limit as usize] += 1;
        });
    }

    /// How many of a rule's splits would have thinned a child past one of [`WOULD_THIN_LIMITS`].
    /// A `limit` not in the list indexes one cell early, as C#'s `Array.IndexOf` -1 does (and
    /// panics for the first rule, where C# throws).
    pub fn would_thin(rule: LevelLineRule, limit: f64) -> i64 {
        Self::with(|t| {
            if t.would_thin.is_empty() {
                return 0;
            }

            let at = WOULD_THIN_LIMITS
                .iter()
                .position(|&l| l == limit)
                .map_or(-1, |i| i as i64);
            let cell = (rule as i64 * WOULD_THIN_LIMITS.len() as i64) + at;
            t.would_thin[usize::try_from(cell).expect("limit is one of WOULD_THIN_LIMITS")]
        })
    }

    /// One field point query, charged to whichever rule is asking.
    pub fn note_evaluate() {
        if Self::meter_on() {
            Self::with(|t| {
                let rule = t.current_rule as usize;
                rows(&mut t.evaluates)[rule] += 1;
            });
        }
    }

    /// Work units the BUDGET was charged while a rule was asking - read as the difference of the
    /// budget's spent units either side of the rule's body. Only this column says what a rule
    /// costs the build's ceiling.
    pub fn note_units(rule: LevelLineRule, amount: i64) {
        if Self::meter_on() {
            Self::with(|t| rows(&mut t.units)[rule as usize] += amount);
        }
    }

    /// Wall time per rule inside the refinement. The clock runs from one rule's ask to the next ask
    /// or the next dequeue, so a rule's seconds include the splitting its firing did; the row None
    /// is the worklist's own time between a dequeue and the first ask. It only runs while the
    /// budget is in the refinement phase.
    pub fn rule_clock(next: LevelLineRule) {
        let now = stopwatch::get_timestamp();
        let in_refinement = LevelLineWork::current()
            .is_some_and(|budget| budget.phase.get() == LevelLineWorkPhase::Refinement);
        Self::with(|t| {
            if t.rule_clock_at != 0 {
                let rule = t.rule_clock_rule as usize;
                rows(&mut t.rule_ticks)[rule] += now - t.rule_clock_at;
            }

            if in_refinement {
                t.rule_clock_at = now;
                t.rule_clock_rule = next;
            } else {
                t.rule_clock_at = 0;
            }
        });
    }

    /// The span bucket a search `span_over_size` sizes across is filed under.
    pub fn bucket_of(span_over_size: f64) -> i32 {
        for (i, &edge) in BUCKET_EDGES.iter().enumerate() {
            if span_over_size < edge {
                return i as i32;
            }
        }

        BUCKET_EDGES.len() as i32
    }

    /// How many span buckets there are.
    pub fn buckets() -> i32 {
        BUCKET_EDGES.len() as i32 + 1
    }

    /// How many reach bands there are.
    pub fn reaches() -> i32 {
        REACH_EDGES.len() as i32 + 1
    }

    /// The reach band a corner `sizes_from_the_line` sizes from the line is filed under; every band
    /// is closed below and open above.
    pub fn reach_of(sizes_from_the_line: f64) -> i32 {
        for (i, &edge) in REACH_EDGES.iter().enumerate() {
            if sizes_from_the_line < edge {
                return i as i32;
            }
        }

        REACH_EDGES.len() as i32
    }

    /// The worklist's own history - mesh size against worklist length as the build ran.
    pub fn progress_report() -> String {
        Self::with(|t| {
            format!(
                "dequeues {}, deepest split {}\n{}",
                stopwatch::n0(t.dequeues),
                t.deepest_split,
                t.progress.join("\n")
            )
        })
    }

    /// One line per refinement call, in the order they ran.
    pub fn refinement_report() -> String {
        Self::with(|t| t.refinements.join("\n"))
    }

    /// Charges one point query to the code that asked it (the caller checks [`FieldCounters::count_by_site`] first).
    /// control. C# walks the stack to the first frame outside the field; Rust has no
    /// stack reflection, so the site is the source location this call was made from, carried up
    /// through every `#[track_caller]` frame (divergence 652).
    #[track_caller]
    pub fn note_site() {
        let at = std::panic::Location::caller();
        let site = format!("{}:{}", at.file(), at.line());
        Self::with(|t| *t.by_site.entry(site).or_insert(0) += 1);
    }

    /// The headline counts, three lines.
    pub fn report() -> String {
        Self::with(|t| {
            format!(
                "Evaluate {} (pick queries {}), MinAlong {}, MinTri {}, MaxAlong {}, MaxTri {}, \
                 segment searches {} (closed form {}), roots {}\n\
                 needles repaired {} (left alone {}, of those on a chain edge {})\n\
                 repair passes that ran out their bound {}",
                t.evaluate,
                t.pick_evaluate,
                t.min_along,
                t.min_tri,
                t.max_along,
                t.max_tri,
                t.segment_searches,
                t.closed_forms,
                t.roots,
                t.needles_repaired,
                t.needles_left_alone,
                t.needles_on_a_chain_edge,
                t.repair_hit_its_bound
            )
        })
    }
}

/// A per-rule row, sized on first use (C# `Rows(ref row) => row ??= new long[AllRules.Length]`).
pub fn rows<T: Clone + Default>(row: &mut Vec<T>) -> &mut Vec<T> {
    if row.is_empty() {
        *row = vec![T::default(); ALL_RULES.len()];
    }

    row
}

/// A per-rule-and-bucket row, sized on first use (C# `Cells`).
pub fn cells<T: Clone + Default>(row: &mut Vec<T>) -> &mut Vec<T> {
    if row.is_empty() {
        *row = vec![T::default(); ALL_RULES.len() * FieldCounters::buckets() as usize];
    }

    row
}

/// A per-rule-and-reach row, sized on first use (C# `Reach`).
pub fn reach(row: &mut Vec<i64>) -> &mut Vec<i64> {
    if row.is_empty() {
        *row = vec![0; ALL_RULES.len() * FieldCounters::reaches() as usize];
    }

    row
}
