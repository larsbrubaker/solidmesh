//! Rust-only checks of R2.P05's `FieldCounters` methods: firing, asking by span and reach, steps,
//! the refinement notes, the reports and the reset, and the switches holding for the thread that
//! sets them, as the tallies they fill do.

use geometry_processing::bevel::level_line::field_counters::{FieldCounters, LevelLineRule};

#[test]
fn rust_only_the_meter_counts_and_reports_the_rules() {
    let _meter = crate::meter_lock::meter_lock();
    FieldCounters::reset();
    let was_on = FieldCounters::meter_on();
    FieldCounters::set_meter_on(true);

    let rule = LevelLineRule::EdgeDip;
    FieldCounters::asked_by_reaching(rule, 1.5, 0.5, 20.0);
    FieldCounters::note_step(3);
    FieldCounters::note_step(5);
    FieldCounters::fired(rule);
    FieldCounters::asked_by(rule, 100.0);
    FieldCounters::note_refine_step(10, 4, 2);
    FieldCounters::note_refinement_started(7, &[1, 2], 1234);
    FieldCounters::stop_rule_clock();

    assert_eq!(FieldCounters::fired_count(rule), 1);
    assert_eq!(FieldCounters::total_steps(), 2);
    let (asked, near, far, bucket) = FieldCounters::with(|t| {
        (
            t.asked[rule as usize],
            (t.near_reach, t.near_asked.iter().sum::<i64>()),
            (t.far_reach, t.far_asked.iter().sum::<i64>()),
            t.current_bucket,
        )
    });
    assert_eq!(asked, 2);
    assert_eq!(
        near,
        (0, 1),
        "a corner half a size out is in the first band"
    );
    assert_eq!(far, (4, 1), "twenty sizes out is past the last edge");
    assert_eq!(bucket, 6, "the second ask's span bucket");

    let rules = FieldCounters::rule_report();
    let line = rules.lines().nth(1).expect("one rule line");
    assert!(line.starts_with("EdgeDip"), "{rules}");
    assert!(
        line.contains("         2         1           1           2        5"),
        "{line}"
    );

    let buckets = FieldCounters::bucket_report();
    assert!(buckets.contains("1 - 2"), "{buckets}");
    assert!(buckets.contains(">= 16"), "{buckets}");

    let reaches = FieldCounters::reach_report();
    assert!(
        reaches.contains("near") && reaches.contains("< 1"),
        "{reaches}"
    );
    assert!(
        reaches.contains("far") && reaches.contains(">= 16"),
        "{reaches}"
    );

    assert!(FieldCounters::progress_report().contains("dequeues 1"));
    let refinements = FieldCounters::refinement_report();
    assert!(
        refinements.contains("group 7, chains 1,2, candidates    1,234"),
        "{refinements}"
    );

    FieldCounters::note_root_cell(true, true, false);
    assert!(FieldCounters::no_search_needed_report().lines().count() == 2);

    FieldCounters::set_meter_on(was_on);
    FieldCounters::reset();
    assert_eq!(FieldCounters::total_steps(), 0);
    assert_eq!(FieldCounters::fired_count(rule), 0);
    assert_eq!(
        FieldCounters::rule_report().lines().count(),
        1,
        "only the header after a reset"
    );
    assert_eq!(FieldCounters::refinement_report(), "");
}

#[test]
fn rust_only_the_site_report_puts_the_busiest_site_first() {
    FieldCounters::reset();
    FieldCounters::with(|t| {
        t.by_site.insert("a.rs:1".to_string(), 2);
        t.by_site.insert("b.rs:9".to_string(), 7);
        t.by_site.insert("c.rs:3".to_string(), 2);
    });

    let report = FieldCounters::site_report();
    let lines: Vec<&str> = report.lines().collect();
    assert_eq!(lines[0], "         7  b.rs:9");
    assert_eq!(
        lines[1], "         2  c.rs:3",
        "tied counts keep the first sort's order"
    );
    assert_eq!(lines[2], "         2  a.rs:1");
    FieldCounters::reset();
}

#[test]
fn rust_only_entering_a_phase_resets_what_the_phase_owns() {
    use std::rc::Rc;

    use geometry_processing::bevel::level_line::{LevelLineWork, LevelLineWorkPhase, WorkBudget};

    let _meter = crate::meter_lock::meter_lock();
    let budget = Rc::new(WorkBudget::new(1_000_000).unwrap());
    let _scope = LevelLineWork::begin(Some(budget.clone()));

    LevelLineWork::charge(5).unwrap();
    budget.enter_side_group(1, 3);
    budget.note_refinement_queue(7, 9);

    LevelLineWork::enter(LevelLineWorkPhase::Refinement);
    assert_eq!(budget.phase.get(), LevelLineWorkPhase::Refinement);
    assert_eq!(budget.phase_started_at.get(), 5);
    assert_eq!(
        (budget.refinement_taken.get(), budget.refinement_left.get()),
        (0, 0)
    );
    assert_eq!(
        budget.side_group_count.get(),
        3,
        "still inside the per-group loop"
    );

    // Entering the phase it is already in changes nothing.
    LevelLineWork::charge(2).unwrap();
    budget.note_refinement_queue(1, 1);
    LevelLineWork::enter(LevelLineWorkPhase::Refinement);
    assert_eq!(budget.phase_started_at.get(), 5);
    assert_eq!(budget.refinement_taken.get(), 1);

    LevelLineWork::enter(LevelLineWorkPhase::EdgeRoots);
    assert_eq!(
        budget.side_group_count.get(),
        3,
        "edge roots are part of the per-group loop"
    );
    assert_eq!(budget.phase_started_at.get(), 7);

    LevelLineWork::enter(LevelLineWorkPhase::Fill);
    assert_eq!(
        budget.side_group_count.get(),
        0,
        "out of the per-group loop"
    );
}

/// Two tests metering side by side each keep the switches they set: one turning the meter off
/// cannot stop another's build being metered half way.
#[test]
fn the_meter_switches_hold_for_the_thread_that_sets_them() {
    let set = std::sync::Arc::new(std::sync::Barrier::new(2));
    let threads: Vec<_> = [true, false]
        .into_iter()
        .map(|on| {
            let set = std::sync::Arc::clone(&set);
            let thread = std::thread::spawn(move || {
                FieldCounters::set_meter_on(on);
                FieldCounters::set_count_by_site(on);
                set.wait();
                (FieldCounters::meter_on(), FieldCounters::count_by_site())
            });
            (on, thread)
        })
        .collect();
    for (on, thread) in threads {
        let read = thread.join().expect("the metering thread runs");
        assert_eq!(
            read,
            (on, on),
            "the thread that switched the meter {on} reads it so"
        );
    }
}
