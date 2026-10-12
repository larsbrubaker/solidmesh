//! Port of `LevelLine/PassOnePinchTests.cs`: WHY the witness pick does not stand up on its FIRST
//! pass - which picked (chain, side) the cut failed to leave one owned loop, whether those chains
//! close alone at the same size, and what room each of them reads. A measurement, not a contract.
//!
//! Gated on `LEVELLINE_COST=1` beside `many_holes_rim_cost_tests`, whose fixture and cap it
//! shares (`level_line_cost_gate`), because one pass of this pick is tens of seconds: a pick that
//! does not leave each side its loop hands the WHOLE pick to the pinch ladder, and every probe is
//! another whole pass. The numbers that define the target (2026-09-22): each chain that fails here
//! builds ALONE at 0.5, reading a room of about 1.05 on its own and about 2.12 against its nearest
//! neighbour, yet the ladder fails at every rung down to 0.0078 - a failure no smaller size cures.

use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build, build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::{
    ChainSideLoops, FieldCounters, LevelLineMesh, PickThatFits, SplitMeter, WorkBudget,
};
use vector_math::Vector3;

use crate::level_line_cost_gate::{self as gate, say};
use crate::level_line_fixture::LevelLineFixture;

/// Where on the part a chain is, so an index in the fixture can be pointed at.
fn where_it_is(fixture: &LevelLineFixture, chain: &[i32]) -> String {
    let points: Vec<Vector3> = chain
        .iter()
        .map(|&v| fixture.positions[v as usize])
        .collect();
    let centre = points.iter().fold(Vector3::ZERO, |a, &p| a + p) / points.len() as f64;
    let radius = points
        .iter()
        .map(|&p| (p - centre).length())
        .fold(f64::NEG_INFINITY, f64::max);
    format!(
        "{} vertices, centre ({:.1}, {:.1}, {:.1}), radius {radius:.2}",
        chain.len(),
        centre.x,
        centre.y,
        centre.z
    )
}

fn row(r: &ChainSideLoops) -> String {
    let met = if r.met_chains.is_empty() {
        "-".to_string()
    } else {
        join(&r.met_chains)
    };
    format!(
        "chain {} side {} (group {}): loops {} ({} closed), mixed {}, met {met}, {}",
        r.chain,
        r.side,
        r.group_tag,
        r.loops,
        r.closed_loops,
        r.mixed_loops,
        if r.owed { "owes nothing" } else { "FAILS" }
    )
}

fn join(values: &[i32]) -> String {
    values
        .iter()
        .map(|v| v.to_string())
        .collect::<Vec<_>>()
        .join(",")
}

/// What the per-rule meter and the split meter saw over one metered build.
fn say_the_roll(split_totals: &str) {
    say(&format!(
        "pass 1 rule meter:\n{}",
        FieldCounters::rule_report()
    ));
    say(&format!(
        "pass 1 refinement: {}",
        FieldCounters::progress_report()
            .split('\n')
            .next()
            .unwrap_or_default()
    ));
    say(&format!("pass 1 {split_totals}"));
}

fn settle(fixture: &Rc<LevelLineFixture>) -> PickThatFits {
    PickThatFits::read(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        gate::feature_test(fixture),
        gate::profile(fixture),
        fixture.size,
        1e-9,
        Some(gate::capped_budget()),
        0.0,
        f64::INFINITY,
    )
    .expect("the settling")
}

fn options(budget: &Rc<WorkBudget>) -> BandGapBuildOptions {
    BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::clone(budget)),
        ..BandGapBuildOptions::default()
    }
}

/// C# `Pass1SaysWhichSidesFail`: the survivors built ONCE at the size the room reading passed,
/// with the per-side reading printed for every picked side that did not leave its one owned loop.
#[test]
fn pass1_says_which_sides_fail() {
    if !gate::asked() {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load(&gate::witness_name()));
    let settled = settle(&fixture);

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
        "pass 1: {} survivors of {} at size {}, the settling read fits {}",
        survivors.len(),
        fixture.chains.len(),
        fixture.size,
        settled.fits
    ));
    for (i, survivor) in survivors.iter().enumerate() {
        say(&format!(
            "  survivor {i} = fixture chain {}, closed {}, {}",
            settled.chains[i],
            closed[i],
            where_it_is(&fixture, survivor)
        ));
    }

    let budget = gate::capped_budget();
    let started = Instant::now();

    // The per-rule meter and the split meter run round the build alone - the settling above is
    // not metered - and are read whether the build succeeds, refuses or fails, so a run that
    // exhausts still says which rule spent the work.
    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    SplitMeter::begin(fixture.size);
    let result = build_pick(
        &fixture.positions,
        &fixture.indices,
        &survivors,
        &closed,
        gate::feature_test(&fixture),
        fixture.size,
        gate::profile(&fixture),
        &options(&budget),
    );
    FieldCounters::set_meter_on(false);
    let split_totals = first_line(&SplitMeter::report());
    SplitMeter::end();

    let built = match result {
        Ok(built) => built,
        Err(badly) => {
            say(&format!(
                "pass 1: stopped after {:.1} s, spent {} of {}, exhausted in {:?} - if this is the \
                 budget, RAISE LEVELLINE_COST_CAP; the verdict was not reached\n  stopped by \
                 {badly:?}",
                started.elapsed().as_secs_f64(),
                gate::n0(budget.spent.get()),
                gate::n0(budget.limit),
                budget.exhausted_in.get()
            ));
            say_the_roll(&split_totals);
            return;
        }
    };

    // The strip and needle counters say whether the input clean-up ran and how much the
    // per-needle repair still had to do; the triangle counts are the input's and the working
    // mesh's kept ones at the END of the build.
    let mesh = built.mesh.as_ref().map(|m| m.borrow());
    let of =
        |read: &dyn Fn(&LevelLineMesh) -> String| mesh.as_ref().map_or(String::new(), |m| read(m));
    say(&format!(
        "pass 1: {:?}, room {}, probes {}, {:.1} s, spent {} of {}, exhausted in {:?}, strip work \
         {}, longest rim {}, strips removed {}, left {}, needles repaired {}, needles left alone \
         {}, triangles in {}, kept at end {}",
        built.refusal,
        built.room,
        built.pinch_probes,
        started.elapsed().as_secs_f64(),
        gate::n0(budget.spent.get()),
        gate::n0(budget.limit),
        budget.exhausted_in.get(),
        of(&|m| gate::n0(m.input_strip_work_spent)),
        of(&|m| m.longest_rim_cut.to_string()),
        of(&|m| m.input_strips_removed.to_string()),
        of(&|m| m.input_strips_left.to_string()),
        of(&|m| m.needles_repaired.to_string()),
        of(&|m| m.needles_left_alone().to_string()),
        fixture.indices.len(),
        of(&|m| (m.triangle_count() - m.removed_count()).to_string())
    ));

    // How far the build grew the mesh - on a refusal too, which carries the mesh it was working
    // on: made counts every triangle ever on the mesh, kept the ones not removed.
    say(&format!(
        "pass 1 growth: triangles kept {} of {} made (in {}), vertices {} (in {})",
        of(&|m| (m.triangle_count() - m.removed_count()).to_string()),
        of(&|m| m.triangle_count().to_string()),
        fixture.indices.len(),
        of(&|m| m.vertex_count().to_string()),
        fixture.positions.len()
    ));
    say_the_roll(&split_totals);

    if built.side_loops.is_empty() {
        say(
            "pass 1: no per-side reading - the build left before the loops were walked, so the \
             verdict was never reached at this cap",
        );
        return;
    }

    let failing: Vec<&ChainSideLoops> = built.side_loops.iter().filter(|r| !r.owed).collect();
    say(&format!(
        "pass 1 verdict: {} picked sides, {} fail",
        built.side_loops.len(),
        failing.len()
    ));
    let mut rows: Vec<&ChainSideLoops> = built.side_loops.iter().collect();
    rows.sort_by_key(|r| (r.chain, r.side));
    for r in rows {
        say(&format!("  {}", row(r)));
    }

    say(&format!(
        "pass 1 loops: {} in all, {} mixed, {} open",
        built.loops.len(),
        built.loops.iter().filter(|l| l.has_mixed_owners).count(),
        built.loops.iter().filter(|l| !l.is_closed).count()
    ));
    for l in built
        .loops
        .iter()
        .filter(|l| l.has_mixed_owners || !l.is_closed)
    {
        say(&format!(
            "  loop side {}, closed {}, {} vertices, owners {}",
            l.side,
            l.is_closed,
            l.vertices.len(),
            join(&l.owners)
        ));
    }

    let mut offending: Vec<i32> = Vec::new();
    for r in &failing {
        if !offending.contains(&r.chain) {
            offending.push(r.chain);
        }
    }
    for chain in offending {
        say(&format!(
            "  offending survivor {chain} = fixture chain {}, {}",
            settled.chains[chain as usize],
            where_it_is(&fixture, &survivors[chain as usize])
        ));
    }
}

fn first_line(text: &str) -> String {
    text.split('\n').next().unwrap_or_default().to_string()
}

/// C# `EachChainAloneIsMeasured`: is a failing side a real pinch at the size asked, or only what
/// the whole pick's refinement makes of it? Every survivor is built ALONE at the same size, its
/// own room printed beside the candidates the WHOLE pick's reading names about it.
#[test]
fn each_chain_alone_is_measured() {
    if !gate::asked() {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load(&gate::witness_name()));
    let settled = settle(&fixture);

    // The whole pick's candidate list, asked well above the size, so every pair tight enough to
    // matter at 0.5 has a row - this is the very reader PickThatFits settles on.
    let whole = band_gap_reading::read_room_of_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        gate::feature_test(&fixture),
        gate::profile(&fixture),
        1e-9,
        Some(gate::capped_budget()),
        4.0 * fixture.size,
        0.0,
        f64::INFINITY,
    )
    .expect("the whole pick's reading");

    say(&format!(
        "whole pick: room {} set by {:?} chain {} other {}, {} candidates",
        whole.room,
        whole.room_set_by.kind,
        whole.room_set_by.chain,
        whole.room_set_by.other_chain,
        whole.candidates.len()
    ));

    let only = std::env::var("LEVELLINE_ALONE_CHAINS").unwrap_or_default();
    let wanted: Vec<usize> = if only.is_empty() {
        (0..settled.chains.len()).collect()
    } else {
        only.split(',')
            .map(|s| s.parse().expect("a survivor index"))
            .collect()
    };

    for i in wanted {
        let k = settled.chains[i];
        let chain = &fixture.chains[k as usize];
        let budget = gate::capped_budget();
        let started = Instant::now();
        let verdict = match build(
            &fixture.positions,
            &fixture.indices,
            chain,
            fixture.closed[k as usize],
            gate::feature_test(&fixture),
            fixture.size,
            gate::profile(&fixture),
            &options(&budget),
        ) {
            Ok(alone) => format!(
                "{:?}, room {} by {:?}, loops {} ({} closed), probes {}",
                alone.refusal,
                alone.room,
                alone.room_set_by.kind,
                alone.loops.len(),
                alone.loops.iter().filter(|l| l.is_closed).count(),
                alone.pinch_probes
            ),
            Err(badly) => format!("stopped by {badly:?}"),
        };

        let mut mine: Vec<_> = whole
            .candidates
            .iter()
            .filter(|c| {
                c.source.chain == k
                    || c.source.other_chain == k
                    || c.group_members.as_ref().is_some_and(|m| m.contains(&k))
            })
            .collect();
        // C#'s OrderBy is stable, so equal rooms keep the reading's order.
        mine.sort_by(|a, b| a.room.total_cmp(&b.room));
        let mine: Vec<String> = mine
            .iter()
            .take(3)
            .map(|c| {
                let other = if c.source.other_chain >= 0 {
                    format!(",{}", c.source.other_chain)
                } else {
                    String::new()
                };
                format!(
                    "{:?}({}{other}) {:.3}",
                    c.source.kind, c.source.chain, c.room
                )
            })
            .collect();

        say(&format!(
            "survivor {i} = fixture chain {k}: alone {verdict}, {:.1} s, spent {} | its tightest \
             whole-pick readings: {} | {}",
            started.elapsed().as_secs_f64(),
            gate::n0(budget.spent.get()),
            mine.join(" ; "),
            where_it_is(&fixture, chain)
        ));
    }
}
