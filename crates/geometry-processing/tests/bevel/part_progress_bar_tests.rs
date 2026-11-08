//! The one bar a bevel rebuild shows (`part::progress_bar`): kernel calls report into slices of
//! it, what the caller sees only moves forward, and every report carries the phase it was made in
//! with what the attempt open now adds (the loops it left out, the size-search rung).

use std::sync::{Arc, Mutex};

use solidmesh::bevel::level_line::work_budget::{
    LevelLineProgress, LevelLineProgressReport, LevelLineWorkPhase,
};
use solidmesh::bevel::part::{
    BevelProgress, BevelProgressBar, BevelProgressPhase, BevelSizeSearchRung,
};

type Told = Arc<Mutex<Vec<(f64, BevelProgress)>>>;

fn recording_bar() -> (BevelProgressBar, Told) {
    let told: Told = Arc::default();
    let sink = Arc::clone(&told);
    let bar = BevelProgressBar::new(Some(Arc::new(move |ratio, progress: &BevelProgress| {
        sink.lock().unwrap().push((ratio, *progress))
    })));
    (bar, told)
}

fn ratios(told: &Told) -> Vec<f64> {
    told.lock()
        .unwrap()
        .iter()
        .map(|(ratio, _)| *ratio)
        .collect()
}

fn building(phase: LevelLineWorkPhase) -> BevelProgressPhase {
    BevelProgressPhase::Building { phase, loops: 0 }
}

/// A rebuild's worth of calls (two pass attempts of room, build and fill, then size-search
/// rungs), each reporting its own 0..1 from the start: the caller sees one run that never goes
/// back and ends at 1.
#[test]
fn every_call_of_a_rebuild_moves_one_bar_forward() {
    let (bar, told) = recording_bar();

    fn run_a_call(tell: LevelLineProgressReport) {
        for i in 0..=10 {
            tell(LevelLineProgress {
                phase: LevelLineWorkPhase::Refinement,
                work_spent: i,
                fraction: i as f64 / 10.0,
            });
        }
    }

    for _attempt in 0..2 {
        bar.open_a_share_of(0.9, 0.9);
        run_a_call(bar.slice(0.0, 0.2, building));
        run_a_call(bar.slice(0.2, 0.75, building));
        run_a_call(bar.slice(0.75, 1.0, building));
        bar.finish_the_window(&BevelProgress::plain(BevelProgressPhase::CheckingTheResult));
    }

    for _rung in 0..8 {
        bar.open_a_share_of(0.97, 0.25);
        run_a_call(bar.slice(0.0, 1.0, building));
    }

    bar.tell(
        1.0,
        &BevelProgress::plain(BevelProgressPhase::CheckingTheResult),
    );

    let ratios = ratios(&told);
    assert!(ratios[0] <= 0.01);

    for i in 1..ratios.len() {
        assert!(
            ratios[i] >= ratios[i - 1],
            "report {i} went back from {} to {}",
            ratios[i - 1],
            ratios[i]
        );
    }

    // Every call moved it: no call's own run of reports sat still at the high-water mark.
    let mut distinct: Vec<u64> = ratios.iter().map(|ratio| ratio.to_bits()).collect();
    distinct.sort_unstable();
    distinct.dedup();
    let distinct = distinct.len();
    assert!(
        distinct > ratios.len() * 3 / 4,
        "{distinct} distinct of {} reports",
        ratios.len()
    );

    assert_eq!(*ratios.last().unwrap(), 1.0);
}

/// A window that opens behind the mark - a retry after the size search already moved the bar -
/// holds the bar rather than dragging it back.
#[test]
fn a_window_behind_the_mark_holds_the_bar() {
    let (bar, told) = recording_bar();

    bar.tell(
        0.95,
        &BevelProgress::plain(BevelProgressPhase::CheckingTheResult),
    );
    bar.open_a_share_of(0.9, 0.9);
    bar.slice(0.0, 1.0, building)(LevelLineProgress {
        phase: LevelLineWorkPhase::Room,
        work_spent: 0,
        fraction: 0.5,
    });

    let min = ratios(&told).into_iter().fold(f64::INFINITY, f64::min);
    assert_eq!(min, 0.95);
    assert_eq!(bar.high_water(), 0.95);
}

/// A call's reports carry its phase, the loops the attempt open now left out, and the rung open
/// now, read when each report comes; a new attempt closes the rung.
#[test]
fn a_report_carries_its_phase_and_what_the_attempt_adds() {
    let (bar, told) = recording_bar();
    let tell = bar.slice(0.0, 1.0, |phase| BevelProgressPhase::Building {
        phase,
        loops: 3,
    });
    let room = LevelLineProgress {
        phase: LevelLineWorkPhase::Room,
        work_spent: 0,
        fraction: 0.5,
    };

    tell(room);
    bar.open_an_attempt(0.97, 0.5, 0.7, 2);
    tell(room);
    let rung = BevelSizeSearchRung {
        loops: 2,
        attempt: 1,
        attempts: 8,
    };
    bar.open_a_rung(0.25, Some(rung));
    tell(room);
    bar.open_an_attempt(0.97, 0.5, 0.7, 0);
    tell(room);

    let doing = BevelProgressPhase::Building {
        phase: LevelLineWorkPhase::Room,
        loops: 3,
    };
    let said: Vec<BevelProgress> = told.lock().unwrap().iter().map(|(_, p)| *p).collect();
    assert_eq!(
        said,
        vec![
            BevelProgress::plain(doing),
            BevelProgress {
                doing,
                leaving_out: 2,
                smaller_size: None
            },
            BevelProgress {
                doing,
                leaving_out: 2,
                smaller_size: Some(rung)
            },
            BevelProgress::plain(doing),
        ]
    );
    assert_eq!(bar.leaving_out(), 0);
    assert_eq!(bar.smaller_size(), None);
}

/// A nested rebuild's bar runs 0..1 into what is left of this one up to the limit, its reports
/// passed through as they are.
#[test]
fn a_nested_bar_reports_into_the_rest_of_this_one() {
    let (bar, told) = recording_bar();
    let done = BevelProgress::plain(BevelProgressPhase::CheckingTheResult);
    bar.tell(0.5, &done);

    let nested = BevelProgressBar::new(Some(bar.the_rest_up_to(1.0)));
    let loop_two = BevelProgress {
        doing: BevelProgressPhase::ShapingLoop {
            loop_number: 2,
            loops: 4,
        },
        leaving_out: 1,
        smaller_size: None,
    };
    nested.tell(0.5, &loop_two);
    nested.tell(1.0, &done);

    let told = told.lock().unwrap().clone();
    assert_eq!(told, vec![(0.5, done), (0.75, loop_two), (1.0, done)]);
}

/// A bar nobody listens to still keeps its mark.
#[test]
fn a_bar_with_nobody_listening_still_moves() {
    let bar = BevelProgressBar::new(None);
    bar.tell(
        0.4,
        &BevelProgress::plain(BevelProgressPhase::CheckingTheResult),
    );
    assert_eq!(bar.high_water(), 0.4);
}
