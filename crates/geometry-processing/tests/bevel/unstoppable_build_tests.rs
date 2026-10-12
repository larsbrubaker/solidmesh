//! Ports `Tests/GeometryProcessingTests/LevelLine/UnstoppableBuildTests.cs`: a build the caller can
//! always STOP - the pinch search included. The pinch ladder once built its probes on a budget of
//! its OWN, made per probe and carrying no token, so the stop button had nothing to stop; the
//! hourglass pinches in milliseconds and runs exactly that ladder, so the requirement is stated on
//! it, with no wall clock in any assert.
//!
//! C#'s `Task.Run` plus `Task.WhenAny(build, Task.Delay(Watchdog))` is a worker thread and a
//! channel read with the watchdog as its timeout; the build's `Rc` state lives on that thread.

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::mpsc;
use std::time::Duration;

use geometry_processing::bevel::level_line::band_gap::BandGapBuildError;
use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandProfile, LevelLineWork, WorkBudget,
};
use polygon_mesh::CancelToken;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::hourglass_solid;

const FOLD_DEGREES: f64 = 45.0;

/// C# `Watchdog`: the watchdog for a build that never comes back. Never a timing rule.
const WATCHDOG: Duration = Duration::from_secs(60);

/// Whether `now` is a budget in force that is not `budget`.
fn another_budget(now: &Option<Rc<WorkBudget>>, budget: &Rc<WorkBudget>) -> bool {
    now.as_ref().is_some_and(|now| !Rc::ptr_eq(now, budget))
}

/// C# `ACancelDuringAPinchProbeEndsTheBuild`: a cancel asked for while a PINCH PROBE is running
/// ends the whole build, as cancellation and not as a refusal.
#[test]
fn a_cancel_during_a_pinch_probe_ends_the_build() {
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        let (positions, indices, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

        let source = CancelToken::new();
        let budget = Rc::new(
            WorkBudget::with_cancellation(
                WorkBudget::default_limit(indices.len() as i32),
                source.clone(),
                None,
            )
            .expect("a positive limit"),
        );
        let probed = Rc::new(Cell::new(false));

        // The stop is pressed from INSIDE the search: the build's own feature reading is called by
        // the probes too, and the budget in force there is not the caller's.
        let feature_and_stop: FeatureEdgeTest = {
            let budget = Rc::clone(&budget);
            let probed = Rc::clone(&probed);
            Rc::new(move |a, b| {
                if another_budget(&LevelLineWork::current(), &budget) {
                    probed.set(true);
                    source.cancel();
                }

                feature(a, b)
            })
        };

        // Well inside the face's room and well outside the neck, so the band closes across it and
        // the ladder runs.
        let ending = band_gap_entry_points::build(
            &positions,
            &indices,
            &rim,
            true,
            Some(feature_and_stop),
            3.0,
            BandProfile::Chamfer,
            &BandGapBuildOptions {
                budget: Some(budget),
                ..BandGapBuildOptions::default()
            },
        );
        let ended_as = match ending {
            Ok(result) => format!(
                "a result the caller no longer wanted ({:?})",
                result.refusal
            ),
            Err(BandGapBuildError::Cancelled) => "cancelled".to_string(),
            Err(other) => format!("{other:?}"),
        };
        sender
            .send((probed.get(), ended_as))
            .expect("the test is waiting");
    });

    let ended = receiver.recv_timeout(WATCHDOG);
    assert!(
        ended.is_ok(),
        "a cancelled level line build must end, and this one is still running"
    );
    let (probed, ended_as) = ended.expect("checked above");

    assert!(
        probed,
        "the pick must reach the pinch search for this to mean anything"
    );
    assert_eq!(
        ended_as, "cancelled",
        "the caller asked the build to stop, and it ended as {ended_as}"
    );
}

/// C# `ThePinchSearchSpendsOneBuildsAllowanceInAll`.
#[test]
fn the_pinch_search_spends_one_builds_allowance_in_all() {
    let (positions, indices, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit"),
    );
    let search_budgets = Rc::new(Cell::new(0));
    let last_seen: Rc<RefCell<Option<Rc<WorkBudget>>>> = Rc::new(RefCell::new(None));

    let feature_and_count: FeatureEdgeTest = {
        let budget = Rc::clone(&budget);
        let search_budgets = Rc::clone(&search_budgets);
        let last_seen = Rc::clone(&last_seen);
        Rc::new(move |a, b| {
            let now = LevelLineWork::current();
            let is_last = match (&now, &*last_seen.borrow()) {
                (Some(now), Some(last)) => Rc::ptr_eq(now, last),
                _ => false,
            };

            if another_budget(&now, &budget) && !is_last {
                // A budget that is neither the caller's nor the one the last probe ran on: a new
                // allowance has been made under this one call.
                search_budgets.set(search_budgets.get() + 1);
                *last_seen.borrow_mut() = now;
            }

            feature(a, b)
        })
    };

    let built = band_gap_entry_points::build(
        &positions,
        &indices,
        &rim,
        true,
        Some(feature_and_count),
        3.0,
        BandProfile::Chamfer,
        &BandGapBuildOptions {
            budget: Some(Rc::clone(&budget)),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick builds or refuses by name");

    assert_eq!(built.refusal, BandGapRefusal::Size);
    assert!(
        built.pinch_probes > 1,
        "the ladder must climb more than one rung for this to mean anything"
    );
    assert_eq!(
        search_budgets.get(),
        1,
        "one allowance for the whole ladder, not {} - a budget per probe lets a search spend a \
         build's worth over and over, which is how thirteen minutes went by",
        search_budgets.get()
    );
}
