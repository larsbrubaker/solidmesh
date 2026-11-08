//! The yielding half of the boolean kernel (`boolean_processing::do_array_async`). An n-ary
//! union has to hand the UI its thread back between operands, or in the browser - where the job
//! and the UI share one thread - the progress bar cannot paint until the whole boolean is over.
//!
//! The UI yield hook is passed to the call, not installed in a process-wide slot, so these tests
//! run in parallel and nothing is installed or restored. `block_on` (shared with
//! `boolean_reporter_routing_tests`) drives the call to completion; every hop it is handed is
//! already complete.

use std::future::{ready, Future, Ready};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};
use std::time::{Duration, Instant};

use solidmesh_math::progress::{ProgressReporter, YIELD_THROTTLE_MS};
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::csg::boolean_processing::{do_array, do_array_async};
use solidmesh_mesh::csg::{BooleanOptions, BooleanReporter, CsgModes};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

/// Runs `future` to completion on this thread.
pub(crate) fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return output;
        }
        std::thread::yield_now();
    }
}

/// What the job did, in the order it did it: every report, and how many reports had happened at
/// each point the UI was handed the thread. The second list is what tells a yield BETWEEN two
/// operations from one before the work started or after it finished.
struct ProgressLog {
    reports: Arc<Mutex<Vec<(f64, String)>>>,
    reports_before_each_yield: Mutex<Vec<usize>>,
    /// One reporter for the whole job: the yield throttle is per-reporter state.
    reporter: ProgressReporter,
}

impl ProgressLog {
    fn new() -> Self {
        let reports: Arc<Mutex<Vec<(f64, String)>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&reports);
        Self {
            reports,
            reports_before_each_yield: Mutex::new(Vec::new()),
            reporter: ProgressReporter::from_fn(move |ratio, message| {
                sink.lock().unwrap().push((ratio, message.to_string()));
            }),
        }
    }

    /// The hook a browser host would install, plus a wait that clears `YIELD_THROTTLE_MS`.
    ///
    /// The wait is what makes the count deterministic. Without it a fixture this small finishes
    /// inside one throttle window, so only the first yield of the run would get through and
    /// "between two operands" could not be observed at all. A real browser frame costs about
    /// this much anyway.
    fn record_yield(&self) -> Ready<()> {
        let count = self.reports.lock().unwrap().len();
        self.reports_before_each_yield.lock().unwrap().push(count);
        wait_out_the_window();
        ready(())
    }
}

/// Waits at least one full throttle window, so the next yield is not throttled away.
fn wait_out_the_window() {
    let start = Instant::now();
    let window = Duration::from_millis(YIELD_THROTTLE_MS);
    std::thread::sleep(window);
    while start.elapsed() < window {
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Three cubes in a row, each overlapping the next, so every pair the kernel folds is a real
/// boolean rather than two disjoint solids.
fn three_overlapping_cubes() -> Vec<(Mesh, Matrix4X4)> {
    vec![
        (create_cube(10.0, 10.0, 10.0), Matrix4X4::IDENTITY),
        (
            create_cube(10.0, 10.0, 10.0),
            Matrix4X4::create_translation(6.0, 0.0, 0.0),
        ),
        (
            create_cube(10.0, 10.0, 10.0),
            Matrix4X4::create_translation(12.0, 0.0, 0.0),
        ),
    ]
}

fn union_async(reporter: &ProgressReporter, log: &ProgressLog) -> Mesh {
    let operands = three_overlapping_cubes();
    let items: Vec<(&Mesh, Matrix4X4)> = operands.iter().map(|(m, x)| (m, *x)).collect();
    let hook = || log.record_yield();
    block_on(do_array_async(
        &items,
        CsgModes::Union,
        reporter,
        Some(&hook),
        &CancelToken::new(),
        &BooleanOptions::default(),
    ))
    .unwrap()
}

#[test]
fn an_nary_union_hands_the_ui_its_thread_back_between_operations() {
    let log = ProgressLog::new();

    let mut result = union_async(&log.reporter, &log);

    let yields = log.reports_before_each_yield.lock().unwrap().clone();
    let reports = log.reports.lock().unwrap().clone();
    assert!(
        !yields.is_empty(),
        "a boolean that never yields is a frozen frame in the browser for as long as it runs"
    );

    // Reports on both sides of a yield is what "between two operations" means: the combine had
    // already told the bar it was starting, handed the thread over, and then ran.
    assert!(
        yields.iter().any(|&count| count > 0 && count < reports.len()),
        "the bar has to get a frame after the combine starts, not only before any report ({yields:?} of {})",
        reports.len()
    );

    assert!(reports.iter().all(|(ratio, _)| (0.0..=1.0).contains(ratio)));

    // and the union still did its job: one solid spanning all three cubes
    result.clean_and_merge();
    let x_size = result.get_axis_aligned_bounding_box().x_size();
    assert!((x_size - 22.0).abs() <= 0.001, "x size {x_size}");
}

#[test]
fn a_union_nobody_is_watching_never_yields() {
    // The hook is passed, so the only reason not to hop the event loop is that there is no
    // reporter. Getting this wrong costs every non-UI caller a UI hop per operand.
    let log = ProgressLog::new();

    union_async(&ProgressReporter::null(), &log);

    assert!(
        log.reports_before_each_yield.lock().unwrap().is_empty(),
        "a boolean with no reporter has no progress to paint"
    );
}

#[test]
fn yielding_does_not_change_the_solid() {
    let log = ProgressLog::new();

    let mut yielding = union_async(&log.reporter, &log);

    // A reporter that is watched but does nothing, so this compares the same combine with and
    // without the yields.
    let operands = three_overlapping_cubes();
    let items: Vec<(&Mesh, Matrix4X4)> = operands.iter().map(|(m, x)| (m, *x)).collect();
    let watched: BooleanReporter = Arc::new(|_ratio: f64, _message: &str| {});
    let mut plain = do_array(
        &items,
        CsgModes::Union,
        &CancelToken::new(),
        &BooleanOptions {
            reporter: Some(watched),
            ..BooleanOptions::default()
        },
    )
    .unwrap();

    yielding.clean_and_merge();
    plain.clean_and_merge();

    assert_eq!(
        yielding.faces.len(),
        plain.faces.len(),
        "handing the UI a frame in the middle of a boolean must not change the solid it builds"
    );
    assert_eq!(yielding.vertices.len(), plain.vertices.len());
}
