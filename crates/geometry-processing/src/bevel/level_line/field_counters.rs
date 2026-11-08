//! How many of each query a level-line build asked, by the refinement rule that asked it: the
//! rules ([`LevelLineRule`]), the per-thread tally ([`FieldCounterTally`], one `thread_local!`),
//! the two per-thread switches and the counting methods ([`FieldCounters`]), with the bounded
//! searches' step and root-cell tallies ([`FieldCounters::note_step`],
//! [`FieldCounters::note_root_cell`]), and, as further `impl` blocks, the rule, bucket and reach
//! reports, the rule tallies the refinement charges (`fired`, `asked_by`, ...) and `reset`.
//!
//! Counted PER THREAD. The kernel never forks a build, so a thread's tally is exactly one caller's
//! build - where one process-wide tally would count whatever build is running BESIDE it, which is
//! every other test in a parallel run and made the same build read a different number of queries
//! from one run to the next.

use std::cell::{Cell, RefCell};

use indexmap::IndexMap;

use super::stopwatch;
use super::{LevelLineWork, LevelLineWorkPhase};

/// The refinement rules a search can be charged to, by the names the rules
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

/// Every rule in discriminant order, the order the reports list them in.
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

thread_local! {
    // Switches on the per-RULE meter for this thread's builds. Per thread like the tallies it
    // fills: a test that meters its own build must not have another test beside it switch the
    // meter off half way, and the tally it reads only ever holds this thread's counts.
    static METER_ON: Cell<bool> = const { Cell::new(false) };

    // Switches on the per-call-site tally of `Evaluate`, per thread for the same reason.
    static COUNT_BY_SITE: Cell<bool> = const { Cell::new(false) };

    static TALLY: RefCell<FieldCounterTally> = RefCell::new(FieldCounterTally::default());
}

/// One thread's tally. The public counters are the ones the field and the mesh charge directly
/// (`evaluate += 1`); the per-rule rows start empty and are sized on first use, so a build with
/// the meter off pays nothing for them.
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

/// How many of each query a build asked - instrumentation for the speed work,
/// zeroed by a test. Its state is the thread's [`FieldCounterTally`].
pub struct FieldCounters;

impl FieldCounters {
    /// Runs `f` on this thread's tally. Never call a `FieldCounters` method from inside `f`.
    pub fn with<R>(f: impl FnOnce(&mut FieldCounterTally) -> R) -> R {
        TALLY.with(|tally| f(&mut tally.borrow_mut()))
    }

    /// Whether the per-rule meter is on: which rule asked each
    /// search, how many found nothing, and what the bounded searches cost in subdivisions and
    /// depth. One thread-local read per subdivision step when it is off, and nothing else.
    pub fn meter_on() -> bool {
        METER_ON.with(Cell::get)
    }

    /// Switches the per-rule meter on or off for this thread's builds.
    pub fn set_meter_on(on: bool) {
        METER_ON.with(|meter| meter.set(on));
    }

    /// Whether the per-call-site tally of `evaluate` is on. Off by
    /// default: knowing HOW MANY point queries a build asks says nothing about which code asks them.
    pub fn count_by_site() -> bool {
        COUNT_BY_SITE.with(Cell::get)
    }

    /// Switches the per-call-site tally on or off for this thread's builds.
    pub fn set_count_by_site(on: bool) {
        COUNT_BY_SITE.with(|count| count.set(on));
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

    /// One subdivision step of a bounded search, at the depth it
    /// stands - charged to the running rule, its span bucket and its two reach bands.
    pub fn note_step(depth: i32) {
        if !Self::meter_on() {
            return;
        }

        Self::with(|t| {
            let rule = t.current_rule as usize;
            rows(&mut t.steps)[rule] += 1;
            let deep = rows(&mut t.deepest);
            if depth > deep[rule] {
                deep[rule] = depth;
            }

            let reaches = Self::reaches() as usize;
            reach(&mut t.near_steps)[(rule * reaches) + t.near_reach as usize] += 1;
            reach(&mut t.far_steps)[(rule * reaches) + t.far_reach as usize] += 1;

            let cell = (rule * Self::buckets() as usize) + t.current_bucket as usize;
            cells(&mut t.bucket_steps)[cell] += 1;
            let deepest = cells(&mut t.bucket_deepest);
            if depth > deepest[cell] {
                deepest[cell] = depth;
            }
        });
    }

    /// The root cell of one bounded triangle search, read for the
    /// "no search needed" question: whether the three corners all take their value from the SAME
    /// chain, whether the convex bound alone proves the greatest value over the triangle stands at
    /// a corner, and whether the search's own first step retired the triangle anyway - which is
    /// what says how much a skip could still save.
    pub fn note_root_cell(same_chain: bool, corner_max_proven: bool, retired_at_the_root: bool) {
        Self::with(|t| {
            let rule = t.current_rule as usize;
            rows(&mut t.root_cells)[rule] += 1;
            if same_chain {
                rows(&mut t.root_same_chain)[rule] += 1;
            }

            if corner_max_proven {
                rows(&mut t.root_corner_max)[rule] += 1;
                if same_chain {
                    rows(&mut t.root_both)[rule] += 1;
                    if !retired_at_the_root {
                        rows(&mut t.root_both_and_searched)[rule] += 1;
                    }
                }
            }

            if retired_at_the_root {
                rows(&mut t.root_retired)[rule] += 1;
            }
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
    /// A `limit` not in the list was never tallied, so it reads 0 rather than borrowing a
    /// neighbouring rule's cell.
    pub fn would_thin(rule: LevelLineRule, limit: f64) -> i64 {
        Self::with(|t| {
            let Some(at) = WOULD_THIN_LIMITS.iter().position(|&l| l == limit) else {
                return 0;
            };

            t.would_thin
                .get((rule as usize * WOULD_THIN_LIMITS.len()) + at)
                .copied()
                .unwrap_or(0)
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
                solidmesh_math::number_format::format_i64_number(t.dequeues),
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
    /// The site is the source location this call was made from, carried up through every
    /// `#[track_caller]` frame: Rust has no stack reflection that names methods in a release or
    /// wasm build, and a caller location costs nothing while the tally is off.
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

/// A per-rule row, sized to every rule on first use.
pub fn rows<T: Clone + Default>(row: &mut Vec<T>) -> &mut Vec<T> {
    if row.is_empty() {
        *row = vec![T::default(); ALL_RULES.len()];
    }

    row
}

/// A per-rule-and-bucket row, sized on first use.
pub fn cells<T: Clone + Default>(row: &mut Vec<T>) -> &mut Vec<T> {
    if row.is_empty() {
        *row = vec![T::default(); ALL_RULES.len() * FieldCounters::buckets() as usize];
    }

    row
}

/// A per-rule-and-reach row, sized on first use.
pub fn reach(row: &mut Vec<i64>) -> &mut Vec<i64> {
    if row.is_empty() {
        *row = vec![0; ALL_RULES.len() * FieldCounters::reaches() as usize];
    }

    row
}

/// .NET `N0`: a count grouped in threes.
fn n0(value: i64) -> String {
    solidmesh_math::number_format::format_i64_number(value)
}

impl FieldCounters {
    /// One search of `rule` found what it looked for - counted by rule, span bucket
    /// and both corners' reach bands, when the meter is on.
    pub fn fired(rule: LevelLineRule) {
        if !Self::meter_on() {
            return;
        }

        Self::with(|t| {
            let rule = rule as usize;
            rows(&mut t.fired)[rule] += 1;
            let bucket = (rule * Self::buckets() as usize) + t.current_bucket as usize;
            cells(&mut t.bucket_fired)[bucket] += 1;
            let reaches = Self::reaches() as usize;
            reach(&mut t.near_fired)[(rule * reaches) + t.near_reach as usize] += 1;
            reach(&mut t.far_fired)[(rule * reaches) + t.far_reach as usize] += 1;
        });
    }

    /// How many searches of `rule` found what they looked for.
    pub fn fired_count(rule: LevelLineRule) -> i64 {
        Self::with(|t| rows(&mut t.fired)[rule as usize])
    }

    /// Every rule's subdivision steps.
    pub fn total_steps() -> i64 {
        Self::with(|t| rows(&mut t.steps).iter().sum())
    }

    /// The running rule's time banked, and the clock stopped.
    pub fn stop_rule_clock() {
        let now = stopwatch::get_timestamp();
        Self::with(|t| {
            if t.rule_clock_at != 0 {
                let rule = t.rule_clock_rule as usize;
                rows(&mut t.rule_ticks)[rule] += now - t.rule_clock_at;
                t.rule_clock_at = 0;
            }
        });
    }

    /// Stopwatch ticks as seconds.
    pub fn seconds(ticks: i64) -> f64 {
        ticks as f64 / stopwatch::FREQUENCY as f64
    }

    /// One line per rule that was asked, stepped or timed.
    pub fn rule_report() -> String {
        Self::with(|t| {
            let mut lines = vec![format!(
                "{:<14}{:>10}{:>10}{:>12}{:>12}{:>9}{:>12}{:>12}{:>12}{:>13}{:>10}",
                "rule",
                "asked",
                "fired",
                "found none",
                "steps",
                "deepest",
                "evaluates",
                "units",
                "convex cut",
                "at the root",
                "seconds"
            )];

            for rule in ALL_RULES {
                let i = rule as usize;
                if rows(&mut t.asked)[i] == 0
                    && rows(&mut t.steps)[i] == 0
                    && rows(&mut t.rule_ticks)[i] == 0
                {
                    continue;
                }

                let asked = rows(&mut t.asked)[i];
                let fired = rows(&mut t.fired)[i];
                lines.push(format!(
                    "{:<14}{:>10}{:>10}{:>12}{:>12}{:>9}{:>12}{:>12}{:>12}{:>13}{:>10.2}",
                    format!("{rule:?}"),
                    n0(asked),
                    n0(fired),
                    n0(asked - fired),
                    n0(rows(&mut t.steps)[i]),
                    rows(&mut t.deepest)[i],
                    n0(rows(&mut t.evaluates)[i]),
                    n0(rows(&mut t.units)[i]),
                    n0(rows(&mut t.convex_retired)[i]),
                    n0(rows(&mut t.convex_retired_root)[i]),
                    Self::seconds(rows(&mut t.rule_ticks)[i])
                ));
            }

            lines.join("\n")
        })
    }

    /// A search of `rule` begins - the rule clock moved to it
    /// and it made current, and (metered) counted by its span bucket.
    pub fn asked_by(rule: LevelLineRule, span_over_size: f64) {
        if Self::meter_on() {
            Self::rule_clock(rule);
        }

        Self::with(|t| t.current_rule = rule);
        if !Self::meter_on() {
            return;
        }

        let bucket = Self::bucket_of(span_over_size);
        Self::with(|t| {
            rows(&mut t.asked)[rule as usize] += 1;
            t.current_bucket = bucket;
            cells(&mut t.bucket_asked)
                [(rule as usize * Self::buckets() as usize) + bucket as usize] += 1;
        });
    }

    /// As
    /// [`FieldCounters::asked_by`], with the corners' reach bands counted too.
    pub fn asked_by_reaching(
        rule: LevelLineRule,
        span_over_size: f64,
        nearest_over_size: f64,
        farthest_over_size: f64,
    ) {
        Self::asked_by(rule, span_over_size);
        if !Self::meter_on() {
            return;
        }

        let near = Self::reach_of(nearest_over_size);
        let far = Self::reach_of(farthest_over_size);
        Self::with(|t| {
            t.near_reach = near;
            t.far_reach = far;
            let reaches = Self::reaches() as usize;
            reach(&mut t.near_asked)[(rule as usize * reaches) + near as usize] += 1;
            reach(&mut t.far_asked)[(rule as usize * reaches) + far as usize] += 1;
        });
    }

    /// One dequeue of the refinement, with a progress line every 5000.
    pub fn note_refine_step(triangle_count: i32, pending_count: i32, split_depth: i32) {
        if !Self::meter_on() {
            return;
        }

        Self::rule_clock(LevelLineRule::None);
        Self::with(|t| {
            t.dequeues += 1;
            if split_depth > t.deepest_split {
                t.deepest_split = split_depth;
            }

            if t.dequeues % 5000 == 1 {
                let line = format!(
                    "  dequeues {:>10}  triangles {:>10}  worklist {:>10}  deepest split {:>4}",
                    n0(t.dequeues),
                    n0(i64::from(triangle_count)),
                    n0(i64::from(pending_count)),
                    t.deepest_split
                );
                t.progress.push(line);
            }
        });
    }

    /// One line per rule and span bucket that was asked.
    pub fn bucket_report() -> String {
        Self::with(|t| {
            let mut lines = vec![format!(
                "{:<14}{:>12}{:>10}{:>10}{:>12}{:>12}{:>9}",
                "rule", "span/size", "asked", "fired", "found none", "steps", "deepest"
            )];

            for rule in ALL_RULES {
                for b in 0..Self::buckets() as usize {
                    let cell = (rule as usize * Self::buckets() as usize) + b;
                    let asked = cells(&mut t.bucket_asked)[cell];
                    if asked == 0 {
                        continue;
                    }

                    let span = if b == 0 {
                        "< 0.5".to_string()
                    } else if b == BUCKET_EDGES.len() {
                        ">= 16".to_string()
                    } else {
                        format!("{} - {}", BUCKET_EDGES[b - 1], BUCKET_EDGES[b])
                    };
                    let fired = cells(&mut t.bucket_fired)[cell];
                    lines.push(format!(
                        "{:<14}{:>12}{:>10}{:>10}{:>12}{:>12}{:>9}",
                        format!("{rule:?}"),
                        span,
                        n0(asked),
                        n0(fired),
                        n0(asked - fired),
                        n0(cells(&mut t.bucket_steps)[cell]),
                        cells(&mut t.bucket_deepest)[cell]
                    ));
                }
            }

            lines.join("\n")
        })
    }

    /// One line per rule, corner and reach band that was asked.
    pub fn reach_report() -> String {
        Self::with(|t| {
            let mut lines = vec![format!(
                "{:<14}{:>8}{:>14}{:>10}{:>10}{:>12}{:>12}",
                "rule", "corner", "|value|/size", "asked", "fired", "found none", "steps"
            )];

            let reaches = Self::reaches() as usize;
            for rule in ALL_RULES {
                for (which, asked_row, fired_row, steps_row) in [
                    ("near", &t.near_asked, &t.near_fired, &t.near_steps),
                    ("far", &t.far_asked, &t.far_fired, &t.far_steps),
                ] {
                    for b in 0..reaches {
                        let cell = (rule as usize * reaches) + b;
                        // A row never sized reads as all zero.
                        let asked = asked_row.get(cell).copied().unwrap_or(0);
                        if asked == 0 {
                            continue;
                        }

                        let band = if b == 0 {
                            format!("< {}", REACH_EDGES[0])
                        } else if b == REACH_EDGES.len() {
                            format!(">= {}", REACH_EDGES[REACH_EDGES.len() - 1])
                        } else {
                            format!(">= {}, < {}", REACH_EDGES[b - 1], REACH_EDGES[b])
                        };
                        let fired = fired_row.get(cell).copied().unwrap_or(0);
                        let steps = steps_row.get(cell).copied().unwrap_or(0);
                        lines.push(format!(
                            "{:<14}{:>8}{:>14}{:>10}{:>10}{:>12}{:>12}",
                            format!("{rule:?}"),
                            which,
                            band,
                            n0(asked),
                            n0(fired),
                            n0(asked - fired),
                            n0(steps)
                        ));
                    }
                }
            }

            lines.join("\n")
        })
    }

    /// One line per rule whose root cells were read.
    pub fn no_search_needed_report() -> String {
        Self::with(|t| {
            let mut lines = vec![format!(
                "{:<14}{:>10}{:>11}{:>12}{:>8}{:>27}{:>21}",
                "rule",
                "searches",
                "one chain",
                "corner max",
                "both",
                "both, not already retired",
                "retired at the root"
            )];

            for rule in ALL_RULES {
                let i = rule as usize;
                if rows(&mut t.root_cells)[i] == 0 {
                    continue;
                }

                lines.push(format!(
                    "{:<14}{:>10}{:>11}{:>12}{:>8}{:>27}{:>21}",
                    format!("{rule:?}"),
                    n0(rows(&mut t.root_cells)[i]),
                    n0(rows(&mut t.root_same_chain)[i]),
                    n0(rows(&mut t.root_corner_max)[i]),
                    n0(rows(&mut t.root_both)[i]),
                    n0(rows(&mut t.root_both_and_searched)[i]),
                    n0(rows(&mut t.root_retired)[i])
                ));
            }

            lines.join("\n")
        })
    }

    /// One line for a group's refinement starting, with what has been
    /// spent so far.
    pub fn note_refinement_started(group_tag: i32, members: &[i32], candidates: i32) {
        if !Self::meter_on() {
            return;
        }

        let total_steps = Self::total_steps();
        Self::with(|t| {
            let members: Vec<String> = members.iter().map(i32::to_string).collect();
            let line = format!(
                "  group {group_tag}, chains {}, candidates {:>8}, spent to here: dequeues {:>10}, steps {:>12}, evaluates {:>12}",
                members.join(","),
                n0(i64::from(candidates)),
                n0(t.dequeues),
                n0(total_steps),
                n0(t.evaluate)
            );
            t.refinements.push(line);
        });
    }

    /// Every count, row and report back to nothing.
    pub fn reset() {
        Self::with(|t| *t = FieldCounterTally::default());
    }

    /// Every counted site, most-counted first (ties where the introsort leaves them).
    pub fn site_report() -> String {
        Self::with(|t| {
            let mut lines: Vec<(i64, String)> = t
                .by_site
                .iter()
                .map(|(site, &count)| (count, format!("{count:>10}  {site}")))
                .collect();
            // Two introsorts: by line descending (ordinal), then by count descending. The sort is
            // unstable, so ties among equal counts fall where `introspective_sort` puts them -
            // the same place on every run.
            solidmesh_math::fixed_tie_sort::introspective_sort(&mut lines, &mut |a, b| {
                b.1.cmp(&a.1)
            });
            solidmesh_math::fixed_tie_sort::introspective_sort(&mut lines, &mut |a, b| {
                b.0.cmp(&a.0)
            });
            lines
                .into_iter()
                .map(|(_, line)| line)
                .collect::<Vec<_>>()
                .join("\n")
        })
    }
}
