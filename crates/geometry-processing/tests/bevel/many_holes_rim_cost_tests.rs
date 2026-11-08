//! THE WITNESS - the app's own dear pick (Many Holes' 3,925-facet face at size 0.5, captured where
//! the route hands the whole pick over) read with the kernel's meters: one room reading, one
//! settling, one build of the survivors.
//!
//! Gated on `LEVELLINE_COST=1` (`level_line_cost_gate`): a reading of this pick is tens of seconds,
//! which has no place in the ordinary run. The table goes to `LEVELLINE_COST_LOG`. The measured
//! tables (2026-09-21) this file was written to produce found that the reading is the per-chain
//! FOREIGN reading (92% of the units), and the build is the Ridge rule's sheer number of askings on
//! triangles sixteen sizes across and bigger.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;
use std::time::Instant;

use solidmesh::bevel::level_line::band_gap_entry_points::{build_pick, BandGapBuildOptions};
use solidmesh::bevel::level_line::{band_gap_reading, foreign_room_counters, level_line_routing};
use solidmesh::bevel::level_line::{FieldCounters, PickThatFits, WorkBudget};

use crate::level_line_cost_gate::{self as gate, say};
use crate::level_line_fixture::LevelLineFixture;

/// The cap for this file: a cap, not a budget - at it the reading stops and says where
/// it stood, so a measurement can never run away inside the suite.
const DEFAULT_CAP: i64 = 5_000_000;

fn counters_now(spent: i64) -> String {
    foreign_room_counters::with(|t| {
        let mut per_chain = t.edges_per_chain.clone().unwrap_or_default();
        per_chain.sort();
        let spread = if per_chain.is_empty() {
            "none".to_string()
        } else {
            format!(
                "{} / {} / {}",
                per_chain[0],
                per_chain[per_chain.len() / 2],
                per_chain[per_chain.len() - 1]
            )
        };

        format!(
            "work {} (foreign {}, chain-to-chain {}, self {}) | foreign calls {}, foreign edges {}, \
             per chain min/median/max {spread} | pair calls {}, pair edges {} | bracket fallbacks \
             {} (touching {}) | ",
            gate::n0(spent),
            gate::n0(t.foreign_units),
            gate::n0(t.pair_units),
            gate::n0(t.self_units),
            gate::n0(t.calls - t.pair_calls),
            gate::n0(t.edges_read - t.pair_edges_read),
            gate::n0(t.pair_calls),
            gate::n0(t.pair_edges_read),
            t.bracket_fallbacks,
            t.touching_fallbacks
        )
    }) + &FieldCounters::report()
}

/// How many SIZES across the triangles near the pick are - the longest edge of every triangle
/// holding a chain vertex, over the size: the band's own neighbourhood the refinement subdivides.
fn span_over_size(fixture: &LevelLineFixture) -> String {
    let on_a_chain: HashSet<i32> = fixture.chains.iter().flatten().copied().collect();
    let mut spans = Vec::new();
    for &(a, b, c) in &fixture.indices {
        if !on_a_chain.contains(&a) && !on_a_chain.contains(&b) && !on_a_chain.contains(&c) {
            continue;
        }

        let pa = fixture.positions[a as usize];
        let pb = fixture.positions[b as usize];
        let pc = fixture.positions[c as usize];
        spans.push(
            (pa - pb)
                .length()
                .max((pb - pc).length().max((pc - pa).length())),
        );
    }

    spans.sort_by(f64::total_cmp);
    if spans.is_empty() {
        return "none".to_string();
    }

    format!(
        "{:.1} / {:.1} / {:.1} over {} triangles",
        spans[0] / fixture.size,
        spans[spans.len() / 2] / fixture.size,
        spans[spans.len() - 1] / fixture.size,
        gate::n0(spans.len() as i64)
    )
}

fn settle(fixture: &Rc<LevelLineFixture>, budget: Rc<WorkBudget>) -> PickThatFits {
    PickThatFits::read(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        gate::feature_test(fixture),
        gate::profile(fixture),
        fixture.size,
        1e-9,
        Some(budget),
        0.0,
        f64::INFINITY,
    )
    .expect("the settling")
}

/// ONE room reading of the whole pick, asked (the size the
/// settling names candidates under) and unasked, with the foreign / chain-to-chain split.
#[test]
fn the_rims_reading_is_measured() {
    if !gate::asked() {
        return;
    }

    let name = gate::witness_name();
    let fixture = Rc::new(LevelLineFixture::load(&name));
    say(&format!(
        "{name}: {} chains, {} triangles, size {}",
        fixture.chains.len(),
        gate::n0(fixture.indices.len() as i64),
        fixture.size
    ));
    say(&format!(
        "span over size near the pick, min / median / max: {}",
        span_over_size(&fixture)
    ));

    for asked in [fixture.size, f64::NAN] {
        foreign_room_counters::reset();
        FieldCounters::reset();
        let budget = gate::budget_of(gate::cap_or(DEFAULT_CAP));
        let started = Instant::now();
        let reading = band_gap_reading::read_room_of_pick(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            gate::feature_test(&fixture),
            gate::profile(&fixture),
            1e-9,
            Some(Rc::clone(&budget)),
            asked,
            0.0,
            f64::INFINITY,
        )
        .expect("the reading");

        let label = if asked.is_nan() {
            "unasked".to_string()
        } else {
            format!("asked {asked}")
        };
        say(&format!(
            "reading {label}: read {}, room {}, set by {:?}, candidates {}, phase {:?}, edges on two \
             triangles {}, offending edge {:?}, offending chain {}, divides {}, {:.1} s | {}",
            reading.read,
            reading.room,
            reading.room_set_by.kind,
            reading.candidates.len(),
            reading.phase,
            reading.chain_edge_stands_on_two_triangles,
            reading.offending_edge,
            reading.offending_chain,
            reading.chain_divides_sides,
            started.elapsed().as_secs_f64(),
            counters_now(budget.spent.get())
        ));
    }
}

/// ONE settling of the pick - what the app does per rebuild - with
/// the rounds it takes and what each round spends.
#[test]
fn the_settling_is_measured() {
    if !gate::asked() {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load(&gate::witness_name()));
    foreign_room_counters::reset();
    FieldCounters::reset();

    let budget = gate::budget_of(gate::cap_or(DEFAULT_CAP));
    let started = Instant::now();
    let settled = settle(&fixture, Rc::clone(&budget));

    say(&format!(
        "settling: rounds {}, survivors {}, dropped {}, unresolved {}, fits {}, {:.1} s | {}",
        settled.rounds,
        settled.chains.len(),
        settled.dropped.len(),
        settled.unresolved.len(),
        settled.fits,
        started.elapsed().as_secs_f64(),
        counters_now(budget.spent.get())
    ));
    let per_round = if settled.rounds > 0 {
        budget.spent.get() / settled.rounds as i64
    } else {
        0
    };
    say(&format!(
        "settling: units per round {}",
        gate::n0(per_round)
    ));
}

/// ONE build of the survivors with the per-rule meter, capped
/// so a build that cannot finish says where it stood rather than running away.
#[test]
fn the_survivors_build_is_measured() {
    if !gate::asked() {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load(&gate::witness_name()));
    let settled = settle(&fixture, gate::budget_of(gate::cap_or(DEFAULT_CAP)));

    let survivors: Vec<Vec<i32>> = settled
        .chains
        .iter()
        .map(|&k| fixture.chains[k as usize].clone())
        .collect();
    let closed: Vec<bool> = settled
        .chains
        .iter()
        .map(|&k| fixture.closed[k as usize])
        .collect();
    say(&format!(
        "build: {} survivors of {}",
        survivors.len(),
        fixture.chains.len()
    ));
    say(&format!(
        "build: span over size {}",
        span_over_size(&fixture)
    ));

    foreign_room_counters::reset();
    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    let budget = gate::budget_of(gate::cap_or(DEFAULT_CAP));

    // The pinch ladder searches on an allowance of its own, so the build's spend alone is not a
    // refused build's total; every allowance the kernel announces is gathered to sum the true one.
    let nested: Rc<RefCell<Vec<Rc<WorkBudget>>>> = Rc::new(RefCell::new(Vec::new()));
    let watch_was = level_line_routing::watch_budget();
    let gathered = Rc::clone(&nested);
    level_line_routing::set_watch_budget(Some(Rc::new(move |b: &Rc<WorkBudget>| {
        gathered.borrow_mut().push(Rc::clone(b))
    })));
    let total = || budget.spent.get() + nested.borrow().iter().map(|b| b.spent.get()).sum::<i64>();
    let started = Instant::now();
    let options = BandGapBuildOptions {
        // The chord tolerance, as the app passes it: 0 is size / 20. The fixture's Segments is the
        // FILL's segment count, not a length - handed in here it once cut the band with 8 mm
        // chords at a 0.5 size and the build refused by Size naming a room the pinch ladder (which
        // always cuts at size / 20) found below it.
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::clone(&budget)),
        ..BandGapBuildOptions::default()
    };
    match build_pick(
        &fixture.positions,
        &fixture.indices,
        &survivors,
        &closed,
        gate::feature_test(&fixture),
        fixture.size,
        gate::profile(&fixture),
        &options,
    ) {
        Ok(built) => say(&format!(
            "build: built {}, refusal {:?}, room {}, probes {}, {:.1} s | {} | build's own {}",
            built.built(),
            built.refusal,
            built.room,
            built.pinch_probes,
            started.elapsed().as_secs_f64(),
            counters_now(total()),
            gate::n0(budget.spent.get())
        )),
        Err(badly) => say(&format!(
            "build: stopped by {badly:?} after {:.1} s | {} | build's own {}",
            started.elapsed().as_secs_f64(),
            counters_now(total()),
            gate::n0(budget.spent.get())
        )),
    }
    FieldCounters::set_meter_on(false);
    level_line_routing::set_watch_budget(watch_was);

    say(&format!("build by phase:\n{}", budget.phase_report()));
    let (foreign_ticks, pair_ticks, self_ticks) =
        foreign_room_counters::with(|t| (t.foreign_ticks, t.pair_ticks, t.self_ticks));
    say(&format!(
        "build room seconds: foreign {:.2}, chain-to-chain {:.2}, self {:.2}",
        FieldCounters::seconds(foreign_ticks),
        FieldCounters::seconds(pair_ticks),
        FieldCounters::seconds(self_ticks)
    ));
    let (pick_evaluate, pick_ticks, member_reads, evaluate) = FieldCounters::with(|t| {
        (
            t.pick_evaluate,
            t.pick_evaluate_ticks,
            t.pick_member_reads,
            t.evaluate,
        )
    });
    let each = if pick_evaluate > 0 {
        FieldCounters::seconds(pick_ticks) * 1e9 / pick_evaluate as f64
    } else {
        0.0
    };
    let per = if pick_evaluate > 0 {
        member_reads as f64 / pick_evaluate as f64
    } else {
        0.0
    };
    say(&format!(
        "build pick evaluates: {} in {:.2} s, {each:.0} ns each; member chains read {}, {per:.2} \
         per pick evaluate; chain evaluates in all {}",
        gate::n0(pick_evaluate),
        FieldCounters::seconds(pick_ticks),
        gate::n0(member_reads),
        gate::n0(evaluate)
    ));
    say(&format!(
        "build rule meter:\n{}",
        FieldCounters::rule_report()
    ));
    say(&format!(
        "build rule meter by span:\n{}",
        FieldCounters::bucket_report()
    ));
    say(&format!(
        "build rule meter by distance from the line:\n{}",
        FieldCounters::reach_report()
    ));
    say(&format!(
        "no search needed:\n{}",
        FieldCounters::no_search_needed_report()
    ));
    say(&format!(
        "refinements:\n{}",
        FieldCounters::refinement_report()
    ));
    say(&format!(
        "refinement worklist:\n{}",
        FieldCounters::progress_report()
    ));
}
