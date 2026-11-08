//! Work budget tests: the requirement a hang breaks - no build may run away. Every build, room
//! reading and fill spends counted work, and when the count runs out it refuses BY NAME, saying
//! what was running - it never spins, never panics and never hands back a mesh it did not finish.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::time::{Duration, Instant};

use solidmesh::bevel::level_line::band_gap::BandGapBuildError;
use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::hole_fill;
use solidmesh::bevel::level_line::pinch_ladder_search;
use solidmesh::bevel::level_line::work_budget::{
    LevelLineProgress, LevelLineWork, DEFAULT_FLOOR, DEFAULT_PER_TRIANGLE,
};
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, LevelLineWorkPhase, WorkBudget, WorkError,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::CancelToken;

use crate::grid_hole_plate_fixture;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::hourglass_solid;
use crate::level_line_test_geometry::cylinder_solid;

const FOLD_DEGREES: f64 = 45.0;

type Rim = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>);

/// The module's own speed fixture - a 32-sided cylinder's top rim, filleted.
fn cylinder_rim() -> Rim {
    let (positions, indices) = cylinder_solid(32, 1.0, 2.0);
    (positions, indices, (32..64).collect())
}

fn budget(limit: i64) -> Rc<WorkBudget> {
    Rc::new(WorkBudget::new(limit).expect("a positive limit"))
}

fn watched(
    limit: i64,
    token: CancelToken,
    progress: impl Fn(LevelLineProgress) + 'static,
    throttle_ms: Option<i64>,
) -> Rc<WorkBudget> {
    let mut budget = WorkBudget::with_cancellation(limit, token, Some(Rc::new(progress)))
        .expect("a positive limit");
    if let Some(ms) = throttle_ms {
        budget.report_throttle_ms = ms;
    }
    Rc::new(budget)
}

/// One band-gap build of a closed chain, with the part's feature edges at the fold angle and the
/// given budget to spend.
#[allow(clippy::too_many_arguments)]
fn build_one(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    size: f64,
    profile: BandProfile,
    chord_tolerance: f64,
    snap: f64,
    budget: Rc<WorkBudget>,
) -> Result<BandGapResult, BandGapBuildError> {
    let options = BandGapBuildOptions {
        chord_tolerance,
        snap,
        budget: Some(budget),
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        true,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        profile,
        &options,
    )
}

/// The cylinder rim's build, with whatever budget the caller wants to watch or
/// stop it by.
fn build_the_rim(rim: &Rim, budget: Rc<WorkBudget>) -> Result<BandGapResult, BandGapBuildError> {
    build_one(
        &rim.0,
        &rim.1,
        &rim.2,
        0.2,
        BandProfile::Fillet,
        0.0,
        1e-9,
        budget,
    )
}

fn built_rim(rim: &Rim, budget: Rc<WorkBudget>) -> BandGapResult {
    build_the_rim(rim, budget).expect("the rim builds or refuses by name")
}

/// A tiny budget refuses by name and is not built.
#[test]
fn a_tiny_budget_refuses_by_name_and_is_not_built() {
    let rim = cylinder_rim();
    let result = built_rim(&rim, budget(100));

    assert_eq!(result.refusal, BandGapRefusal::WorkBudgetExceeded);
    assert!(!result.built());
    assert!(result.mesh.is_some());
    assert_eq!(result.loops.len(), 0);
    assert_ne!(result.phase, LevelLineWorkPhase::None);
    assert_eq!(result.work_limit, 100);

    // Spent stops within one charge of the limit: the refusal happens AT the overrun.
    assert!(result.work_spent > 100, "spent {}", result.work_spent);
    assert!(result.work_spent <= 110, "spent {}", result.work_spent);
}

/// A tiny budget refuses a room reading too.
#[test]
fn a_tiny_budget_refuses_a_room_reading_too() {
    let (positions, indices, rim) = cylinder_rim();
    let reading = band_gap_reading::read_room(
        &positions,
        &indices,
        &rim,
        true,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        BandProfile::Fillet,
        1e-9,
        Some(budget(50)),
        0.0,
        f64::INFINITY,
    )
    .expect("the reading refuses by name");

    assert!(!reading.read);
    assert!(reading.room.is_nan());
    assert_eq!(reading.phase, LevelLineWorkPhase::Room);
}

/// An ordinary build spends far under its default budget.
#[test]
fn an_ordinary_build_spends_far_under_its_default_budget() {
    let rim = cylinder_rim();
    let budget = budget(WorkBudget::default_limit(rim.1.len() as i32));
    let result = built_rim(&rim, Rc::clone(&budget));

    assert!(result.built());
    println!(
        "cylinder top rim, fillet 0.2: {} units of a {} budget",
        budget.spent.get(),
        budget.limit
    );
    assert!(budget.spent.get() * 10 < budget.limit);

    let triangles = result
        .mesh
        .as_ref()
        .expect("a mesh")
        .borrow()
        .triangle_count();
    let fill_budget = self::budget(WorkBudget::default_limit(triangles));
    let filled = gap_fill::fill_with_budget(
        &result,
        BandProfile::Fillet,
        4,
        Some(Rc::clone(&fill_budget)),
    )
    .expect("the fill");
    assert!(filled.filled());
    println!(
        "the fill: {} units of a {} budget",
        fill_budget.spent.get(),
        fill_budget.limit
    );
    assert!(fill_budget.spent.get() * 10 < fill_budget.limit);
}

/// A refinement that would not settle is stopped in the refinement.
#[test]
fn a_refinement_that_would_not_settle_is_stopped_in_the_refinement() {
    let (positions, indices, rim) = cylinder_rim();
    let result = build_one(
        &positions,
        &indices,
        &rim,
        0.2,
        BandProfile::Fillet,
        1e-12,
        1e-15,
        budget(200_000),
    )
    .expect("the build refuses by name");

    assert_eq!(result.refusal, BandGapRefusal::WorkBudgetExceeded);
    assert_eq!(result.phase, LevelLineWorkPhase::Refinement);

    // The refinement's half-grown mesh travels with the refusal, to be read and never used.
    assert!(!result.built());
    assert!(result.mesh.is_some());
}

/// A ladder too poor for its first probe says it ran out and refuses by work budget.
#[test]
fn a_ladder_too_poor_for_its_first_probe_says_it_ran_out_and_refuses_by_work_budget() {
    let (positions, indices, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
    let chains = vec![rim];
    let closed = vec![true];

    // One unit: under any probe's cost, whatever a probe comes to cost.
    let outer = budget(1);
    let ladder = {
        let _scope = LevelLineWork::begin(Some(Rc::clone(&outer)));
        pinch_ladder_search::pinch_room_below(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
            BandProfile::Chamfer,
            1e-9,
            3.0,
            f64::INFINITY,
        )
        .expect("the ladder answers")
    };

    let ran_out = ladder
        .ran_out_budget
        .as_ref()
        .expect("the probe could not be finished, so the ladder learned nothing about the part");
    assert!(ladder.probes > 0, "a rung was attempted");
    assert_eq!(ladder.room, 0.0);
    assert!(
        !ladder.pinches_everywhere,
        "no rung was answered, so no rung can be said to have pinched"
    );
    assert_eq!(
        ran_out.limit, 1,
        "the ladder's allowance is the outer build's limit handed over, not one of its own"
    );

    // The refusal BandGap makes of that outcome, made here the way BandGap makes it.
    let refused = pinch_ladder_search::ladder_ran_out(&ladder, None, None);
    assert_eq!(refused.refusal, BandGapRefusal::WorkBudgetExceeded);
    assert_ne!(
        refused.refusal,
        BandGapRefusal::Size,
        "a room of zero presented as a size reads as 'no size at all fits here'"
    );
    assert_ne!(
        refused.phase,
        LevelLineWorkPhase::None,
        "a work-budget refusal says what was running when the work ran out"
    );
    assert!(!refused.built());
    assert!(refused.mesh.is_none());
    assert_eq!(refused.work_limit, 1);
}

/// A ladder that runs out of work refuses by work budget and not by size.
#[test]
fn a_ladder_that_runs_out_of_work_refuses_by_work_budget_and_not_by_size() {
    let (positions, indices, rim) = hourglass_solid(10.0, 0.1, 4.0, 4.0);

    let build_the_hourglass = |limit: i64| {
        build_one(
            &positions,
            &indices,
            &rim,
            3.0,
            BandProfile::Chamfer,
            0.0,
            1e-9,
            budget(limit),
        )
        .expect("the hourglass builds or refuses by name")
    };

    let mut found: Option<BandGapResult> = None;
    let mut found_at = 0_i64;
    let mut last_without_probes = 0_i64;
    let mut first_with_a_room = 0_i64;
    let mut first_with_a_room_result: Option<BandGapResult> = None;

    let mut limit = 4_000_i64;
    while limit <= 100_000_000 {
        let probed = build_the_hourglass(limit);

        if probed.pinch_probes == 0 {
            last_without_probes = limit;
            // Grow the limit by 15% plus one; the cast truncates a positive value.
            limit = (limit as f64 * 1.15) as i64 + 1;
            continue;
        }

        if probed.refusal == BandGapRefusal::Size && probed.room > 0.0 {
            first_with_a_room = limit;
            first_with_a_room_result = Some(probed);
            break;
        }

        found = Some(probed);
        found_at = limit;
        break;
    }

    let found = found.unwrap_or_else(|| {
        let r = first_with_a_room_result.as_ref();
        panic!(
            "no budget leaves the pinch ladder reached and nothing proven on this fixture: the \
             largest limit the ladder was not reached at is {last_without_probes}, and the smallest \
             limit that proved a size is {first_with_a_room} (refusal {:?}, room {:?}, probes {:?}) \
             - the window has closed and this requirement needs a fixture where it is still open",
            r.map(|r| r.refusal),
            r.map(|r| r.room),
            r.map(|r| r.pinch_probes)
        )
    });

    println!(
        "pinch ladder work window: ladder unreached up to {last_without_probes}, nothing proven at \
         {found_at} (probes {}, spent {})",
        found.pinch_probes, found.work_spent
    );

    assert_ne!(
        found.refusal,
        BandGapRefusal::Size,
        "nothing was learned about which sizes fit: {:?} room {}",
        found.refusal,
        found.room
    );
    assert_eq!(found.refusal, BandGapRefusal::WorkBudgetExceeded);
    assert!(
        !found.pinches_at_every_size_tried,
        "no rung was answered, so no rung can be said to have pinched"
    );
    assert_ne!(
        found.phase,
        LevelLineWorkPhase::None,
        "a work-budget refusal says what was running when the work ran out"
    );
}

/// A default budget is proportional to the input.
#[test]
fn a_default_budget_is_proportional_to_the_input() {
    assert_eq!(WorkBudget::default_limit(0), DEFAULT_FLOOR);
    assert_eq!(
        WorkBudget::default_limit(1000),
        DEFAULT_FLOOR + (1000 * DEFAULT_PER_TRIANGLE)
    );
}

/// A token cancelled before the call stops the build at once.
#[test]
fn a_token_cancelled_before_the_call_throws_at_once() {
    let rim = cylinder_rim();
    let token = CancelToken::new();
    token.cancel();

    let budget = Rc::new(
        WorkBudget::with_cancellation(WorkBudget::default_limit(rim.1.len() as i32), token, None)
            .expect("a positive limit"),
    );

    assert!(matches!(
        build_the_rim(&rim, Rc::clone(&budget)),
        Err(BandGapBuildError::Cancelled)
    ));
    assert_eq!(budget.spent.get(), 0);
}

/// Cancelling from a progress report stops the build and leaves the input alone.
#[test]
fn cancelling_from_a_progress_report_throws_and_leaves_the_input_alone() {
    let rim = cylinder_rim();
    let positions_before = rim.0.clone();
    let indices_before = rim.1.clone();

    let token = CancelToken::new();
    let reports = Rc::new(RefCell::new(0));

    let (counted, cancel) = (Rc::clone(&reports), token.clone());
    let budget = watched(
        WorkBudget::default_limit(rim.1.len() as i32),
        token,
        move |progress| {
            *counted.borrow_mut() += 1;

            if progress.phase == LevelLineWorkPhase::Refinement {
                cancel.cancel();
            }
        },
        None,
    );

    assert!(matches!(
        build_the_rim(&rim, budget),
        Err(BandGapBuildError::Cancelled)
    ));
    assert!(*reports.borrow() > 0);
    assert_eq!(rim.0, positions_before);
    assert_eq!(rim.1, indices_before);
}

/// A cancel reaches a large hole fill within a fifth of a second (of wall-clock time).
#[test]
fn cancel_reaches_a_large_hole_fill_within_a_fifth_of_a_second() {
    const N: usize = 800;
    let mut loop_points = Vec::new();
    let mut outward = Vec::new();
    let mut lengths = Vec::new();
    for i in 0..N {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / N as f64;
        let radial = Vector3::new(angle.cos(), angle.sin(), 0.0);
        loop_points.push(radial * 50.0);
        outward.push(radial);
        lengths.push(2.0 * std::f64::consts::PI * 50.0 / N as f64);
    }

    let token = CancelToken::new();
    let theirs = token.clone();
    let fill = std::thread::spawn(move || {
        // The budget is the thread's own (it is not `Send`), so only the token crosses - which is
        // the whole of what the cancel needs.
        let budget = Rc::new(
            WorkBudget::with_cancellation(i64::MAX, theirs, None).expect("a positive limit"),
        );
        let _scope = LevelLineWork::begin(Some(budget));
        // Whether the fill left as a cancel (the error itself carries an `Rc` and stays here).
        matches!(
            hole_fill::fill(&loop_points, &outward, &lengths),
            Err(WorkError::Cancelled)
        )
    });

    std::thread::sleep(Duration::from_millis(50));
    assert!(
        !fill.is_finished(),
        "the fill has to still be running for the cancel to be tested at all"
    );

    let cancelled_at = Instant::now();
    token.cancel();

    let cancelled = fill.join().expect("the fill thread");
    let response_ms = cancelled_at.elapsed().as_secs_f64() * 1000.0;
    assert!(cancelled, "the fill leaves as a cancel");
    assert!(
        response_ms < 200.0,
        "a cancel has to land within a fifth of a second wherever the build is: {response_ms} ms"
    );
}

/// An uncancelled token changes no result.
#[test]
fn an_uncancelled_token_changes_no_result() {
    let rim = cylinder_rim();
    let limit = WorkBudget::default_limit(rim.1.len() as i32);

    let plain = built_rim(&rim, budget(limit));
    let watched = built_rim(&rim, watched(limit, CancelToken::new(), |_| {}, None));

    assert!(plain.built());
    assert!(watched.built());
    let plain = plain.mesh.as_ref().expect("a mesh").borrow();
    let watched = watched.mesh.as_ref().expect("a mesh").borrow();
    assert_eq!(watched.triangle_count(), plain.triangle_count());
    assert_eq!(watched.vertex_count(), plain.vertex_count());

    for i in 0..plain.vertex_count() as usize {
        assert_eq!(watched.vertices[i], plain.vertices[i]);
    }
}

/// Watching a build costs almost nothing (a timing test).
#[test]
fn watching_a_build_costs_almost_nothing() {
    let rim = cylinder_rim();
    let limit = WorkBudget::default_limit(rim.1.len() as i32);

    // Warm.
    built_rim(&rim, budget(limit));

    let token = CancelToken::new();
    let reports = Rc::new(RefCell::new(0_i64));
    let mut plain = f64::MAX;
    let mut watched_ms = f64::MAX;

    for _ in 0..5 {
        let plain_run = Instant::now();
        for _ in 0..10 {
            built_rim(&rim, budget(limit));
        }

        plain = plain.min(plain_run.elapsed().as_secs_f64() * 1000.0);

        let watched_run = Instant::now();
        for _ in 0..10 {
            let counted = Rc::clone(&reports);
            built_rim(
                &rim,
                watched(
                    limit,
                    token.clone(),
                    move |_| *counted.borrow_mut() += 1,
                    None,
                ),
            );
        }

        watched_ms = watched_ms.min(watched_run.elapsed().as_secs_f64() * 1000.0);
    }

    println!(
        "level line watch overhead, best of five: plain {plain:.1} ms, watched {watched_ms:.1} ms, \
         reports {}",
        reports.borrow()
    );

    assert!(watched_ms < f64::max(20.0, plain * 2.0));
}

/// Progress runs forward through the phases.
#[test]
fn progress_runs_forward_through_the_phases() {
    let rim = cylinder_rim();
    let fractions = Rc::new(RefCell::new(Vec::<f64>::new()));
    let phases = Rc::new(RefCell::new(HashSet::<LevelLineWorkPhase>::new()));

    let (f, p) = (Rc::clone(&fractions), Rc::clone(&phases));
    let gap = built_rim(
        &rim,
        watched(
            WorkBudget::default_limit(rim.1.len() as i32),
            CancelToken::default(),
            move |progress| {
                f.borrow_mut().push(progress.fraction);
                p.borrow_mut().insert(progress.phase);
            },
            None,
        ),
    );

    let fractions = fractions.borrow();
    assert!(gap.built());
    assert!(fractions.len() >= 2);
    assert!(phases.borrow().len() >= 2);

    for i in 1..fractions.len() {
        assert!(fractions[i] >= fractions[i - 1]);
    }

    assert!(fractions[0] >= 0.0);
    assert!(fractions[fractions.len() - 1] <= 1.0);
}

fn listed(fractions: &[f64]) -> String {
    fractions
        .iter()
        .map(|f| format!("{f:.3}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The distinct fractions, in first-seen order.
fn distinct(fractions: &[f64]) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    for &f in fractions {
        if !out.contains(&f) {
            out.push(f);
        }
    }
    out
}

/// Every side group moves the bar through its own slice.
#[test]
fn every_side_group_moves_the_bar_through_its_own_slice() {
    let rim = cylinder_rim();
    let reports = Rc::new(RefCell::new(Vec::<LevelLineProgress>::new()));

    let r = Rc::clone(&reports);
    let gap = built_rim(
        &rim,
        watched(
            WorkBudget::default_limit(rim.1.len() as i32),
            CancelToken::default(),
            move |progress| r.borrow_mut().push(progress),
            Some(0),
        ),
    );

    let reports = reports.borrow();
    assert!(gap.built());

    for i in 1..reports.len() {
        assert!(
            reports[i].fraction >= reports[i - 1].fraction,
            "report {i} went back from {:.3} to {:.3}",
            reports[i - 1].fraction,
            reports[i].fraction
        );
    }

    let refinement: Vec<f64> = reports
        .iter()
        .filter(|report| report.phase == LevelLineWorkPhase::Refinement)
        .map(|report| report.fraction)
        .collect();
    let refinement_distinct = distinct(&refinement);
    assert!(
        refinement_distinct.len() > 2,
        "the refinement moves the bar: {}",
        listed(&refinement_distinct)
    );
    let max = refinement.iter().copied().fold(f64::MIN, f64::max);
    let min = refinement.iter().copied().fold(f64::MAX, f64::min);
    assert!(
        max - min > 0.3,
        "the refinement carries most of the bar across its groups: {}",
        listed(&refinement_distinct)
    );

    // The LAST group's refinement moves the bar too. Find the last report that is not the
    // refinement and is followed by one that is, by its own position: reports compare by value,
    // so a search for an equal report could land on an earlier entry into the refinement.
    let last_entry = (0..reports.len().saturating_sub(1))
        .rev()
        .find(|&i| {
            reports[i].phase != LevelLineWorkPhase::Refinement
                && reports[i + 1].phase == LevelLineWorkPhase::Refinement
        })
        .map_or(0, |i| i + 1);
    let last_group: Vec<f64> = reports[last_entry..]
        .iter()
        .take_while(|report| report.phase == LevelLineWorkPhase::Refinement)
        .map(|report| report.fraction)
        .collect();
    assert!(
        last_entry > 0,
        "the rim has two side groups, so refinement is entered twice"
    );
    assert!(
        distinct(&last_group).len() > 1,
        "the last group's refinement held the bar still: {}",
        listed(&last_group)
    );

    assert!(
        reports[reports.len() - 1].fraction >= 0.85,
        "a cut that finished has passed the refinement and edge roots"
    );
}

/// The refinement is where the work goes.
#[test]
fn the_refinement_is_where_the_work_goes() {
    let rim = cylinder_rim();
    let spent_in = Rc::new(RefCell::new(HashMap::<LevelLineWorkPhase, i64>::new()));
    let last = Rc::new(RefCell::new(0_i64));
    let phase = Rc::new(RefCell::new(LevelLineWorkPhase::None));

    let budget = budget(WorkBudget::default_limit(rim.1.len() as i32));
    let (s, l, p) = (Rc::clone(&spent_in), Rc::clone(&last), Rc::clone(&phase));
    let watcher = watched(
        WorkBudget::default_limit(rim.1.len() as i32),
        CancelToken::default(),
        move |progress| {
            let had = *s.borrow().get(&*p.borrow()).unwrap_or(&0);
            s.borrow_mut()
                .insert(*p.borrow(), had + (progress.work_spent - *l.borrow()));
            *l.borrow_mut() = progress.work_spent;
            *p.borrow_mut() = progress.phase;
        },
        None,
    );

    let gap = built_rim(&rim, Rc::clone(&watcher));
    let total = watcher.spent.get();
    {
        let mut spent_in = spent_in.borrow_mut();
        let tail = *spent_in.get(&*phase.borrow()).unwrap_or(&0);
        spent_in.insert(*phase.borrow(), tail + (total - *last.borrow()));
    }

    let spent_in = spent_in.borrow();
    let mut ordered: Vec<_> = spent_in.iter().collect();
    ordered.sort_by(|a, b| b.1.cmp(a.1));
    for (key, value) in ordered {
        println!("phase {key:?} spent {value} of {total}");
    }

    assert!(gap.built());
    assert_eq!(budget.spent.get(), 0);

    let refinement = *spent_in.get(&LevelLineWorkPhase::Refinement).unwrap_or(&0);
    assert!(refinement > total / 3, "refinement {refinement} of {total}");

    for (key, value) in spent_in.iter() {
        if *key != LevelLineWorkPhase::Refinement {
            assert!(
                *value <= refinement,
                "{key:?} spent {value} over the refinement's {refinement}"
            );
        }
    }
}

/// The bar rises during the room reading.
#[test]
fn the_bar_rises_during_the_room_reading() {
    let (positions, indices, rim) = cylinder_rim();
    let room_fractions = Rc::new(RefCell::new(Vec::<f64>::new()));

    let r = Rc::clone(&room_fractions);
    let budget = watched(
        WorkBudget::default_limit(indices.len() as i32),
        CancelToken::default(),
        move |progress| {
            if progress.phase == LevelLineWorkPhase::Room {
                r.borrow_mut().push(progress.fraction);
            }
        },
        Some(0),
    );

    let reading = band_gap_reading::read_room(
        &positions,
        &indices,
        &rim,
        true,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        BandProfile::Fillet,
        1e-9,
        Some(budget),
        0.0,
        f64::INFINITY,
    )
    .expect("the reading");

    let room_fractions = room_fractions.borrow();
    assert!(reading.read);
    assert!(room_fractions.len() > 1);
    assert!(
        room_fractions[room_fractions.len() - 1] > room_fractions[0],
        "the bar rises through the reading: {}",
        listed(&room_fractions)
    );
    assert!(room_fractions[room_fractions.len() - 1] < 1.0);
}

/// A cut never reports the room phase once it has begun.
#[test]
fn a_cut_never_reports_the_room_phase_once_it_has_begun() {
    let rim = cylinder_rim();
    let rim_phases = Rc::new(RefCell::new(Vec::<LevelLineWorkPhase>::new()));
    let r = Rc::clone(&rim_phases);
    let rim_built = built_rim(
        &rim,
        watched(
            WorkBudget::default_limit(rim.1.len() as i32),
            CancelToken::default(),
            move |p| r.borrow_mut().push(p.phase),
            None,
        ),
    );

    let (plate_positions, plate_indices, plate_chains) = grid_hole_plate_fixture::pick(1);
    let plate_phases = Rc::new(RefCell::new(Vec::<LevelLineWorkPhase>::new()));
    let p = Rc::clone(&plate_phases);
    let options = BandGapBuildOptions {
        budget: Some(watched(
            WorkBudget::default_limit(plate_indices.len() as i32),
            CancelToken::default(),
            move |progress| p.borrow_mut().push(progress.phase),
            None,
        )),
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build_pick(
        &plate_positions,
        &plate_indices,
        &plate_chains,
        &vec![true; plate_chains.len()],
        Some(grid_hole_plate_fixture::feature_edges(
            &plate_positions,
            &plate_indices,
        )),
        grid_hole_plate_fixture::SIZE,
        BandProfile::Chamfer,
        &options,
    )
    .expect("the plate builds or refuses by name");

    assert!(rim_built.built());

    for phases in [rim_phases.borrow(), plate_phases.borrow()] {
        let cut_began = phases
            .iter()
            .position(|&phase| {
                phase != LevelLineWorkPhase::None && phase != LevelLineWorkPhase::Room
            })
            .expect("the cut began");
        let distinct: Vec<_> = phases.iter().fold(Vec::new(), |mut seen, p| {
            if !seen.contains(p) {
                seen.push(*p);
            }
            seen
        });
        assert!(
            !phases[cut_began..].contains(&LevelLineWorkPhase::Room),
            "the phases were {distinct:?}"
        );
    }
}
