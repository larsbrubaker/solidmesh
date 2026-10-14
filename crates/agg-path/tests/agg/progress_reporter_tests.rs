// ProgressReporter's reports and its throttled yield to the UI. The host's yield is handed to
// each `yield_to_ui` call rather than set process-wide, so each test passes its own counting hook
// and nothing has to be restored.

use std::cell::Cell;
use std::sync::{Arc, Mutex};

use agg_path::progress_reporter::{ProgressReporter, YIELD_THROTTLE_MS};
use web_time::{Duration, Instant};

fn no_op() -> ProgressReporter {
    ProgressReporter::from_fn(|_ratio, _message| {})
}

/// Returns once the throttle's own clock says a full window has passed since this was called,
/// re-reading it rather than trusting one sleep.
fn wait_out_the_window() {
    let start = Instant::now();
    std::thread::sleep(Duration::from_millis(YIELD_THROTTLE_MS));
    while start.elapsed() < Duration::from_millis(YIELD_THROTTLE_MS) {
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn throttle_collapses_rapid_yields_into_one() {
    let yield_count = Cell::new(0);
    let ui_yield = || yield_count.set(yield_count.get() + 1);
    let reporter = no_op();

    // A job reports far more often than a frame lasts; without the throttle each of these would
    // be a two idle turn hop.
    for _report in 0..20 {
        reporter.yield_to_ui(Some(&ui_yield));
    }

    assert_eq!(
        yield_count.get(),
        1,
        "only the first yield may get through until the throttle window has passed"
    );
}

#[test]
fn yields_again_once_the_throttle_window_has_passed() {
    let yield_count = Cell::new(0);
    let ui_yield = || yield_count.set(yield_count.get() + 1);
    let reporter = no_op();

    reporter.yield_to_ui(Some(&ui_yield));
    reporter.yield_to_ui(Some(&ui_yield));

    // The throttle reads a real clock, so proving it reopens costs one wait of the window itself.
    wait_out_the_window();

    reporter.yield_to_ui(Some(&ui_yield));

    assert_eq!(
        yield_count.get(),
        2,
        "the window reopening must let the next report yield again"
    );
}

#[test]
fn null_reporter_swallows_reports_and_never_yields() {
    let yield_count = Cell::new(0);
    let ui_yield = || yield_count.set(yield_count.get() + 1);

    ProgressReporter::null().report(0.5, "swallowed");
    ProgressReporter::null().report_and_yield(0.5, "swallowed", Some(&ui_yield));

    assert_eq!(
        yield_count.get(),
        0,
        "nothing is showing this job's progress, so there is no frame worth painting"
    );
}

#[test]
fn report_reaches_the_wrapped_action() {
    let reported = Arc::new(Mutex::new(Vec::<(f64, String)>::new()));
    let sink = Arc::clone(&reported);
    let reporter = ProgressReporter::from_fn(move |ratio, message| {
        sink.lock().unwrap().push((ratio, message.to_string()))
    });

    reporter.report(0.25, "quarter");
    reporter.report_and_yield::<()>(0.75, "three quarters", None);

    let reported = reported.lock().unwrap();
    assert_eq!(reported.len(), 2);
    assert_eq!(reported[0], (0.25, "quarter".to_string()));
    assert_eq!(reported[1], (0.75, "three quarters".to_string()));
}

#[test]
fn a_scaled_reporter_maps_its_range_into_the_parents_window() {
    let reports = Arc::new(Mutex::new(Vec::<(f64, String)>::new()));
    let sink = Arc::clone(&reports);
    let parent = ProgressReporter::from_fn(move |ratio, message| {
        sink.lock().unwrap().push((ratio, message.to_string()))
    });

    let second_half = parent.scaled(0.5, 0.5);

    second_half.report(0.0, "start");
    second_half.report(0.5, "middle");
    second_half.report(1.0, "end");

    let reports = reports.lock().unwrap();
    let ratios: Vec<f64> = reports.iter().map(|report| report.0).collect();
    assert_eq!(ratios, vec![0.5, 0.75, 1.0]);

    // The message is the child's, untouched: the parent supplies where in the job this is, the
    // child supplies what it is doing.
    let messages: Vec<&str> = reports.iter().map(|report| report.1.as_str()).collect();
    assert_eq!(messages, vec!["start", "middle", "end"]);
}

#[test]
fn scaled_children_share_one_throttle_window_with_their_parent() {
    // The bug this pins: a hand written wrapper per phase starts never having yielded, so its
    // first yield always hops. A job that makes one per item then pays a hop per item for work
    // that takes no time at all, and the throttle stops meaning anything.
    let yield_count = Cell::new(0);
    let ui_yield = || yield_count.set(yield_count.get() + 1);
    let parent = no_op();

    parent.scaled(0.5, 0.0).yield_to_ui(Some(&ui_yield));
    parent.scaled(0.5, 0.5).yield_to_ui(Some(&ui_yield));
    parent.yield_to_ui(Some(&ui_yield));

    assert_eq!(
        yield_count.get(),
        1,
        "the throttle is one budget for the job, however many slices the job carves it into"
    );
}

#[test]
fn scaling_a_reporter_nobody_is_watching_gives_back_the_same_reporter() {
    // Nothing to report to and nothing to paint, so a wrapper would be pure cost - and, worse, a
    // live-looking object that a caller testing for "is anyone watching" cannot see through.
    assert!(ProgressReporter::null()
        .scaled(0.5, 0.0)
        .ptr_eq(&ProgressReporter::null()));
}

#[test]
fn has_target_sees_every_shape_of_nobody_watching() {
    // Not just the null singleton: a reporter built around no target is exactly as unwatched, and
    // a caller comparing against the null reporter by identity would take it for a live one.
    assert!(!ProgressReporter::null().has_target());
    assert!(!ProgressReporter::new(None).has_target());
    assert!(no_op().has_target());
    assert!(no_op().scaled(0.5, 0.0).has_target());
}
