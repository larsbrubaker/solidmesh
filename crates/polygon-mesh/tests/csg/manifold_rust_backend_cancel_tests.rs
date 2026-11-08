//! The manifold-rust boolean backend's cancellation and progress cases: a cancel the kernel
//! observes surfaces as cancellation, a cancel from another thread stops the boolean before its
//! work finishes, and a reporter sees progress without changing the result. Also the UV sphere
//! fixture they share. The rest of the backend's tests are `manifold_rust_backend_tests.rs`;
//! split only for the 800-line limit.
//!
//! `cancel_observed_by_the_kernel_surfaces_as_cancellation` lands its cancel after the last
//! operand's import and before the kernel. `do_array` takes a slice, so the hook is the batch
//! combine's own opening report ("Boolean: combining"), which the seam publishes after every
//! import and before the kernel's CSG tree runs, so the next code to see the flag is the kernel.
//! A reporter is not a reason to fold, so this is the batch path every union takes.

use std::sync::{Arc, Barrier, Mutex};

use solidmesh_math::{Matrix4X4, Vector3, Vector3Float};
use solidmesh_mesh::csg::{
    boolean_processing, BooleanError, BooleanOptions, CsgModes, WindingRule,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Face, Mesh};

use super::manifold_rust_backend_tests::signed_volume;

fn union(
    operands: &[(&Mesh, Matrix4X4)],
    cancel: &CancelToken,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    boolean_processing::do_array(operands, CsgModes::Union, cancel, options)
}

#[test]
fn cancel_observed_by_the_kernel_surfaces_as_cancellation() {
    let mesh_a = uv_sphere(10.0, 64);
    let mesh_b = uv_sphere(10.0, 64);
    let matrix_b = Matrix4X4::create_translation(5.0, 0.0, 0.0);
    let operands = [(&mesh_a, Matrix4X4::IDENTITY), (&mesh_b, matrix_b)];

    let baseline = union(&operands, &CancelToken::new(), &BooleanOptions::default())
        .expect("the uncancelled union succeeds");
    assert!(!baseline.faces.is_empty());

    // The seam reports the combine's start after the last import and before the kernel's tree
    // runs, so cancelling there leaves the kernel as the next code to see the flag.
    let cancelling = CancelToken::new();
    let flag = cancelling.clone();
    let options = BooleanOptions {
        reporter: Some(Arc::new(move |_ratio: f64, _message: &str| flag.cancel())),
        ..BooleanOptions::default()
    };

    let thrown = union(&operands, &cancelling, &options).expect_err("a cancelled union fails");
    assert!(
        matches!(thrown, BooleanError::Cancelled),
        "a cancel must surface as a cancellation, not as: {thrown}"
    );

    // The kernel's configuration is process-global and its cancel flag is sticky, so a
    // cancelled operation could plausibly poison the next one. It must not.
    let after_cancel = union(&operands, &CancelToken::new(), &BooleanOptions::default())
        .expect("a union after a cancelled one succeeds");

    assert_eq!(after_cancel.faces.len(), baseline.faces.len());
    assert!((signed_volume(&after_cancel) - signed_volume(&baseline)).abs() <= 1e-9);
}

/// What one run of the Nonzero union reported, deduplicated by message, and how it ended.
struct ReportedRun {
    phases: Vec<String>,
    result: Result<Mesh, BooleanError>,
}

/// The pairwise (Nonzero) union of two UV spheres; `on_first_report` runs inside the kernel's
/// first progress callback.
fn run_nonzero_union(
    cancel: &CancelToken,
    on_first_report: Option<Box<dyn Fn() + Send + Sync>>,
) -> ReportedRun {
    let mesh_a = uv_sphere(10.0, 64);
    let mesh_b = uv_sphere(10.0, 64);
    let phases: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&phases);
    let options = BooleanOptions {
        reporter: Some(Arc::new(move |_ratio: f64, message: &str| {
            let first = {
                let mut phases = sink.lock().unwrap();
                let first = phases.is_empty();
                if first || phases.last().map(String::as_str) != Some(message) {
                    phases.push(message.to_string());
                }
                first
            };
            if first {
                if let Some(on_first_report) = &on_first_report {
                    on_first_report();
                }
            }
        })),
        winding_rule: WindingRule::Nonzero,
        ..BooleanOptions::default()
    };
    let result = union(
        &[
            (&mesh_a, Matrix4X4::IDENTITY),
            (&mesh_b, Matrix4X4::create_translation(5.0, 0.0, 0.0)),
        ],
        cancel,
        &options,
    );
    let phases = phases.lock().unwrap().clone();
    ReportedRun { phases, result }
}

#[test]
fn cancel_from_another_thread_during_the_boolean_stops_before_the_work_finishes() {
    let full_run = run_nonzero_union(&CancelToken::new(), None);
    let baseline = full_run.result.expect("the uncancelled union succeeds");
    assert!(!baseline.faces.is_empty());

    // The last report is the seam's own "combining", published after the kernel has returned -
    // a cancelled run never gets there whether or not the kernel honoured the cancel, so it is
    // not work a cancel can be credited with skipping. Pinned rather than assumed.
    let last = full_run.phases.last().cloned().unwrap_or_default();
    assert!(
        last.ends_with("combining"),
        "the full run's last report was expected to be the seam's combining step: {:?}",
        full_run.phases
    );
    let kernel_phases = &full_run.phases[..full_run.phases.len() - 1];
    assert!(
        kernel_phases.len() > 1,
        "the kernel must pass through more than one phase, or there is no later work a cancel \
         could be shown to skip"
    );

    let cancelling = CancelToken::new();
    let inside_the_kernel = Arc::new(Barrier::new(2));
    let cancel_sent = Arc::new(Barrier::new(2));

    let worker = {
        let cancelling = cancelling.clone();
        let inside_the_kernel = Arc::clone(&inside_the_kernel);
        let cancel_sent = Arc::clone(&cancel_sent);
        std::thread::spawn(move || {
            run_nonzero_union(
                &cancelling,
                Some(Box::new(move || {
                    // Hold the kernel inside its first phase until the other thread has
                    // cancelled, so the cancel lands mid-work on every machine.
                    inside_the_kernel.wait();
                    cancel_sent.wait();
                })),
            )
        })
    };

    // Not a delay: the boolean has reported its first phase and is parked there.
    inside_the_kernel.wait();
    cancelling.cancel();
    cancel_sent.wait();
    let cancelled_run = worker.join().expect("the worker finishes");

    match &cancelled_run.result {
        Err(BooleanError::Cancelled) => {}
        Err(other) => panic!("a cancel must surface as a cancellation, not as: {other}"),
        Ok(_) => panic!("a cancelled boolean must not report success"),
    }

    assert!(
        cancelled_run.phases.len() < kernel_phases.len(),
        "the cancelled boolean went on to report {:?}, every kernel phase of the full run {:?} - \
         the cancel is being ignored until the work finishes",
        cancelled_run.phases,
        kernel_phases
    );
}

/// A closed, manifold UV sphere: one vertex at each pole with a triangle fan, quads between the
/// interior rings. Built here rather than taken from a fixture because the only thing wanted
/// from it is bulk - a boolean slow enough to still be running a few milliseconds after it
/// starts.
fn uv_sphere(radius: f64, segments: i32) -> Mesh {
    let mut mesh = Mesh::new();
    let rings = segments / 2;

    mesh.vertices
        .push(Vector3Float::from_vector3(Vector3::new(0.0, 0.0, radius)));

    for ring in 1..rings {
        let phi = std::f64::consts::PI * ring as f64 / rings as f64;
        let z = radius * phi.cos();
        let ring_radius = radius * phi.sin();

        for segment in 0..segments {
            let theta = 2.0 * std::f64::consts::PI * segment as f64 / segments as f64;
            mesh.vertices.push(Vector3Float::from_vector3(Vector3::new(
                ring_radius * theta.cos(),
                ring_radius * theta.sin(),
                z,
            )));
        }
    }

    let south = mesh.vertices.len() as i32;
    mesh.vertices
        .push(Vector3Float::from_vector3(Vector3::new(0.0, 0.0, -radius)));

    // Ring r's segment s, as an index into the vertex list; ring 0 is the first interior ring,
    // which starts one past the north pole.
    let vertex = |ring: i32, segment: i32| 1 + (ring * segments) + (segment % segments);

    let add_face = |mesh: &mut Mesh, v0: i32, v1: i32, v2: i32| {
        let face = Face::new(v0, v1, v2, &mesh.vertices);
        mesh.faces.push(face);
    };

    for segment in 0..segments {
        add_face(&mut mesh, 0, vertex(0, segment), vertex(0, segment + 1));
        add_face(
            &mut mesh,
            south,
            vertex(rings - 2, segment + 1),
            vertex(rings - 2, segment),
        );
    }

    for ring in 0..rings - 2 {
        for segment in 0..segments {
            add_face(
                &mut mesh,
                vertex(ring, segment),
                vertex(ring + 1, segment),
                vertex(ring, segment + 1),
            );
            add_face(
                &mut mesh,
                vertex(ring, segment + 1),
                vertex(ring + 1, segment),
                vertex(ring + 1, segment + 1),
            );
        }
    }

    mesh
}

#[test]
fn reporter_sees_progress_without_changing_the_result() {
    const RATIO_COMPLETED: f64 = 0.25;
    const AMOUNT_PER_OPERATION: f64 = 0.5;

    let a = create_cube(10.0, 10.0, 10.0);
    let b = create_cube(10.0, 10.0, 10.0);
    let operands = [
        (&a, Matrix4X4::create_translation(-3.0, 0.0, 0.0)),
        (&b, Matrix4X4::create_translation(3.0, 0.0, 0.0)),
    ];

    let reported: Arc<Mutex<Vec<(f64, String)>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&reported);
    let with_reporter = union(
        &operands,
        &CancelToken::new(),
        &BooleanOptions {
            // The kernel's callback can arrive on a worker thread, so the list it lands in has
            // to be guarded even though it is never re-entered.
            reporter: Some(Arc::new(move |ratio: f64, message: &str| {
                sink.lock().unwrap().push((ratio, message.to_string()));
            })),
            amount_per_operation: AMOUNT_PER_OPERATION,
            ratio_completed: RATIO_COMPLETED,
            ..BooleanOptions::default()
        },
    )
    .expect("the union succeeds");

    let reported = reported.lock().unwrap().clone();
    assert!(
        !reported.is_empty(),
        "a boolean that reports nothing leaves the progress bar frozen"
    );

    let mut previous = RATIO_COMPLETED;
    for (ratio, message) in &reported {
        assert!(*ratio >= RATIO_COMPLETED);
        assert!(*ratio <= RATIO_COMPLETED + AMOUNT_PER_OPERATION);

        // A bar that goes backwards is worse than no bar; the kernel's fraction restarts at
        // every phase, so the adapter has to enforce this.
        assert!(*ratio >= previous);
        previous = *ratio;

        assert!(!message.is_empty());
    }

    let without_reporter = union(&operands, &CancelToken::new(), &BooleanOptions::default())
        .expect("the union succeeds");

    assert_eq!(
        with_reporter.vertices.len(),
        without_reporter.vertices.len()
    );
    assert_eq!(with_reporter.faces.len(), without_reporter.faces.len());
    assert!((signed_volume(&with_reporter) - signed_volume(&without_reporter)).abs() <= 1e-9);
}
