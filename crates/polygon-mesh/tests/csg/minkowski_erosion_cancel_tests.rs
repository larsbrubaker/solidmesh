//! Cancellation and the throttled progress bar on both slow erosion paths of
//! `csg::minkowski_morph` - the kernel's union tree (`Manifold::try_erode_by_convex`), which a
//! non-convex solid takes, and the sweep, which a non-convex tool still takes.
//!
//! Each test asserts which path ran from the `ErosionPath` the erosion answers with. A cancelled
//! erosion answers only `Cancelled`, so the cancel tests first finish the same erosion and read
//! the path from that: the route depends on the operands alone. The difference with progress is
//! `minkowski_difference_with_path_and_progress`.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use solidmesh_mesh::csg::minkowski_morph::{
    minkowski_difference_with_path, minkowski_difference_with_path_and_progress, ErosionPath,
};
use solidmesh_mesh::csg::minkowski_processing::{sphere_mesh, MinkowskiError};
use solidmesh_mesh::csg::minkowski_progress_model::tree_units;
use solidmesh_mesh::csg::BooleanReporter;
use solidmesh_mesh::{CancelToken, Mesh};

use super::minkowski_tests::dented;

#[test]
fn a_cancelled_sweep_erosion_stops_within_a_hull_or_two_of_being_asked() {
    let solid = dented(sphere_mesh(5.0, 16).unwrap());

    let tool = non_convex_tool();

    assert_eq!(
        path_when_finished(&solid, &tool),
        ErosionPath::Sweep,
        "the tree declines a non-convex tool, so this has to be the sweep"
    );
    let reports = cancel_on_second_report(&solid, &tool);

    // 128 x 32 face-pair hulls + 128 faces + the closing merge is 4225 units, reported every
    // 42 - about 100 reports. Loose because the parallel hull map cannot recall iterations
    // already in flight.
    assert!(
        reports < 40,
        "cancel was ignored for {reports} of the sweep's ~100 reports"
    );
}

#[test]
fn a_cancelled_tree_erosion_stops_within_a_hull_or_two_of_being_asked() {
    let solid = dented(sphere_mesh(5.0, 16).unwrap());

    let tool = ball_tool();

    assert_eq!(
        path_when_finished(&solid, &tool),
        ErosionPath::Tree,
        "a non-convex solid and a convex ball are what the tree takes"
    );
    let reports = cancel_on_second_report(&solid, &tool);

    // 128 hulls + 8 leaves + 7 nodes + the subtraction + the closing pass is 145 units.
    assert!(
        reports < 40,
        "cancel was ignored for {reports} of the tree's ~145 reports"
    );
}

#[test]
fn a_throttled_sweep_erosion_still_fills_the_bar() {
    let solid = dented(sphere_mesh(5.0, 8).unwrap());
    let tool = non_convex_tool();

    assert_eq!(
        solid.faces.len() * tool.faces.len(),
        1024,
        "1024 face-pair hulls + 32 solid faces + the closing merge is 1057 units, so the throttle's step is 10"
    );

    let (ratios, path) = erode_recording_the_bar(&solid, &tool);

    assert_eq!(
        path,
        ErosionPath::Sweep,
        "the tree declines a non-convex tool, so this has to have been the sweep"
    );
    assert_throttled_and_full(&ratios, 1057);
}

#[test]
fn a_throttled_tree_erosion_still_fills_the_bar() {
    let solid = dented(sphere_mesh(5.0, 24).unwrap());

    assert_eq!(
        tree_units(solid.faces.len() as i64, true),
        325,
        "288 hulls, 18 leaves, 17 nodes, the subtraction and the closing pass: the throttle's step is 3"
    );

    let (ratios, path) = erode_recording_the_bar(&solid, &ball_tool());

    assert_eq!(
        path,
        ErosionPath::Tree,
        "a non-convex solid and a convex ball are what the tree takes"
    );
    assert_throttled_and_full(&ratios, 325);
}

fn path_when_finished(solid: &Mesh, tool: &Mesh) -> ErosionPath {
    minkowski_difference_with_path(solid, tool)
        .expect("the erosion finishes")
        .1
}

fn cancel_on_second_report(solid: &Mesh, tool: &Mesh) -> usize {
    let source = CancelToken::new();
    let reports = Arc::new(AtomicUsize::new(0));
    let reporter: BooleanReporter = {
        let source = source.clone();
        let reports = Arc::clone(&reports);
        Arc::new(move |_ratio: f64, _message: &str| {
            // Not the first: that one is the kernel opening the phase, before any hull has run,
            // so cancelling on it would only re-test the entry gate.
            if reports.fetch_add(1, Ordering::SeqCst) + 1 >= 2 {
                source.cancel();
            }
        })
    };

    let result = minkowski_difference_with_path_and_progress(solid, tool, Some(reporter), &source);

    assert!(
        matches!(result, Err(MinkowskiError::Cancelled)),
        "the kernel reports a cancelled run as a status; this layer owes the caller an error"
    );

    reports.load(Ordering::SeqCst)
}

fn erode_recording_the_bar(solid: &Mesh, tool: &Mesh) -> (Vec<f64>, ErosionPath) {
    let ratios: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&ratios);
    let reporter: BooleanReporter = Arc::new(move |ratio: f64, _message: &str| {
        sink.lock().unwrap().push(ratio);
    });

    let (eroded, path) = minkowski_difference_with_path_and_progress(
        solid,
        tool,
        Some(reporter),
        &CancelToken::new(),
    )
    .unwrap();
    assert!(!eroded.faces.is_empty());

    let ratios = ratios.lock().unwrap().clone();
    (ratios, path)
}

fn assert_throttled_and_full(ratios: &[f64], units: usize) {
    assert!(
        ratios.len() < units,
        "fewer reports than units is what makes this the throttled regime the bug lived in ({})",
        ratios.len()
    );
    assert!(
        ratios.len() > 50,
        "and more than a handful is what says a slow path ran at all ({})",
        ratios.len()
    );
    let last = ratios[ratios.len() - 1];
    assert!(
        (last - 1.0).abs() <= 1e-9,
        "a finished erosion has to leave the bar full, got {last}"
    );
}

/// The convex ball every fast path takes.
fn ball_tool() -> Mesh {
    sphere_mesh(0.5, 8).unwrap()
}

/// The same ball, dented: 32 triangles and not convex, which the tree and the closed form both
/// decline.
fn non_convex_tool() -> Mesh {
    dented(ball_tool())
}
