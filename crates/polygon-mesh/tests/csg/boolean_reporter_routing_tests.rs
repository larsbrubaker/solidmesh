//! Which n-ary route the boolean kernel takes (`boolean_processing::do_array` /
//! `do_array_async`) must not depend on the progress reporter at all - not on whether one
//! exists, and not on whether anyone is watching it.
//!
//! The two routes - the kernel's CSG tree and the explicit pairwise left fold - build the same
//! solid out of different vertices, and the fold costs a full binary boolean per operand. Only a
//! non-default winding rule may take the fold. A reporter with no target still makes a non-null
//! sink once it is wrapped as a callback; the synchronous cases pass that shape, a sink that
//! forwards to the targetless reporter.

use std::sync::{Arc, Mutex};

use solidmesh_math::progress::ProgressReporter;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::csg::boolean_processing::{do_array, do_array_async};
use solidmesh_mesh::csg::{BooleanOptions, BooleanReporter, CsgModes};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};
use std::future::Ready;

use super::boolean_kernel_yield_tests::block_on;

/// Four solids, no two of them the same size and none aligned to another, each overlapping its
/// neighbour. Asymmetric on purpose: a left fold and an n-ary tree combine different pairs
/// first, and on a symmetric chain the two orders can land on the same answer and hide the
/// divergence.
fn asymmetric_operands() -> Vec<(Mesh, Matrix4X4)> {
    vec![
        (create_cube(10.0, 10.0, 10.0), Matrix4X4::IDENTITY),
        (
            create_cube(7.0, 12.0, 9.0),
            Matrix4X4::create_rotation_z(0.3) * Matrix4X4::create_translation(5.5, 1.25, 0.0),
        ),
        (
            create_cube(9.0, 6.0, 11.0),
            Matrix4X4::create_rotation_x(0.45) * Matrix4X4::create_translation(10.75, -0.5, 1.5),
        ),
        (
            create_cube(6.0, 8.0, 7.0),
            Matrix4X4::create_rotation_y(0.2) * Matrix4X4::create_translation(15.25, 2.0, -1.0),
        ),
    ]
}

fn run(
    operands: &[(Mesh, Matrix4X4)],
    operation: CsgModes,
    reporter: Option<BooleanReporter>,
) -> Mesh {
    let items: Vec<(&Mesh, Matrix4X4)> = operands.iter().map(|(m, x)| (m, *x)).collect();
    do_array(
        &items,
        operation,
        &CancelToken::new(),
        &BooleanOptions {
            reporter,
            ..BooleanOptions::default()
        },
    )
    .unwrap()
}

fn union(reporter: Option<BooleanReporter>) -> Mesh {
    run(&asymmetric_operands(), CsgModes::Union, reporter)
}

fn union_async(reporter: &ProgressReporter) -> Mesh {
    let operands = asymmetric_operands();
    let items: Vec<(&Mesh, Matrix4X4)> = operands.iter().map(|(m, x)| (m, *x)).collect();
    block_on(do_array_async(
        &items,
        CsgModes::Union,
        reporter,
        None::<&dyn Fn() -> Ready<()>>,
        &CancelToken::new(),
        &BooleanOptions::default(),
    ))
    .unwrap()
}

/// A reporter wrapped as a boolean's progress sink: a non-null callback calling its `report`.
fn as_action(reporter: ProgressReporter) -> Option<BooleanReporter> {
    Some(Arc::new(move |ratio: f64, message: &str| {
        reporter.report(ratio, message)
    }))
}

/// Every report a recording reporter received, ratio and message.
type Reports = Arc<Mutex<Vec<(f64, String)>>>;

/// A reporter recording every report, for the sync and the async entry points alike.
fn recording() -> (Reports, ProgressReporter) {
    let reports: Reports = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&reports);
    let reporter = ProgressReporter::from_fn(move |ratio, message| {
        sink.lock().unwrap().push((ratio, message.to_string()));
    });
    (reports, reporter)
}

/// Vertex for vertex and index for index, with no tolerance: the point is that the two calls
/// ran the same arithmetic in the same order, not that they agree about the shape.
fn assert_same_mesh(expected: &Mesh, actual: &Mesh, because: &str) {
    assert_eq!(actual.vertices.len(), expected.vertices.len(), "{because}");
    assert_eq!(actual.faces.len(), expected.faces.len(), "{because}");
    for i in 0..expected.vertices.len() {
        assert_eq!(actual.vertices[i], expected.vertices[i], "{because}");
    }
    for i in 0..expected.faces.len() {
        let (a, e) = (actual.faces[i], expected.faces[i]);
        assert_eq!((a.v0, a.v1, a.v2), (e.v0, e.v1, e.v2), "{because}");
    }
}

#[test]
fn a_targetless_reporter_builds_the_same_solid_as_none() {
    let with_no_reporter = union(None);

    // ProgressReporter::null() wrapped as a non-null sink: a reporter that is present but has no
    // target must not buy the pairwise fold while the line above gets the CSG tree.
    let with_targetless_reporter = union(as_action(ProgressReporter::null()));

    assert_same_mesh(
        &with_no_reporter,
        &with_targetless_reporter,
        "a reporter nobody is watching must not change which n-ary route the kernel takes",
    );
}

#[test]
fn a_reporter_built_around_a_null_action_builds_the_same_solid_as_none() {
    // Not the null singleton: a pointer comparison would catch that one and still miss this,
    // which is just as unwatched.
    let with_no_reporter = union(None);
    let with_targetless_reporter = union(as_action(ProgressReporter::new(None)));

    assert_same_mesh(
        &with_no_reporter,
        &with_targetless_reporter,
        "nobody-is-watching arrives in more than one shape and every shape has to route alike",
    );
}

#[test]
fn a_targetless_reporter_builds_the_same_solid_as_none_on_the_async_path() {
    let with_no_reporter = union_async(&ProgressReporter::new(None));
    let with_targetless_reporter = union_async(&ProgressReporter::null());

    assert_same_mesh(
        &with_no_reporter,
        &with_targetless_reporter,
        "the async entry point routes by the same rule as the synchronous one",
    );
}

#[test]
fn a_watched_reporter_keeps_the_nary_batch_path() {
    let (reports, reporter) = recording();

    let with_no_reporter = union(None);
    let watched = union(as_action(reporter));

    // The fold and the tree build different vertices for these operands, so matching the
    // unwatched result exactly is what says the watched one took the tree.
    assert_same_mesh(
        &with_no_reporter,
        &watched,
        "progress being watched must not drop the n-ary boolean to a pairwise fold",
    );

    let reports = reports.lock().unwrap();
    assert!(
        !reports.is_empty(),
        "a watched boolean still tells the bar when it starts and finishes"
    );
    assert!(reports.iter().all(|(ratio, _)| (0.0..=1.0).contains(ratio)));
}

#[test]
fn a_watched_reporter_keeps_the_nary_batch_path_on_the_async_path() {
    let (reports, reporter) = recording();

    let with_no_reporter = union_async(&ProgressReporter::null());
    let watched = union_async(&reporter);

    assert_same_mesh(
        &with_no_reporter,
        &watched,
        "the async entry point keeps the n-ary boolean under a watched reporter too",
    );

    assert!(!reports.lock().unwrap().is_empty());
}

/// Whether any report names a kernel phase - anything past the start/finish "combining" the
/// batch path reports on its own.
fn reports_a_kernel_phase(reports: &[(f64, String)]) -> bool {
    reports
        .iter()
        .any(|(_, message)| message.starts_with("Boolean: ") && message != "Boolean: combining")
}

fn distinct_messages(reports: &[(f64, String)]) -> String {
    let mut seen: Vec<&str> = Vec::new();
    for (_, message) in reports {
        if !seen.contains(&message.as_str()) {
            seen.push(message);
        }
    }
    seen.join(", ")
}

#[test]
fn a_watched_two_operand_subtract_streams_kernel_phases() {
    // Two operands take the n-ary batch too, so before the kernel's CSG tree took a reporter the
    // most common boolean in the app showed "combining" and nothing else.
    let subtract = |reporter: Option<BooleanReporter>| {
        let operands: Vec<(Mesh, Matrix4X4)> = asymmetric_operands().into_iter().take(2).collect();
        run(&operands, CsgModes::Subtract, reporter)
    };

    let (reports, reporter) = recording();
    let with_no_reporter = subtract(None);
    let watched = subtract(as_action(reporter));

    assert_same_mesh(
        &with_no_reporter,
        &watched,
        "watching a subtract must not change its result",
    );
    let reports = reports.lock().unwrap();
    assert!(
        reports_a_kernel_phase(&reports),
        "a watched subtract streams the kernel's phases (saw {})",
        distinct_messages(&reports)
    );
    assert!(reports.iter().all(|(ratio, _)| (0.0..=1.0).contains(ratio)));
}

#[test]
fn a_watched_nary_union_streams_kernel_phases_on_the_async_path() {
    let (reports, reporter) = recording();

    let with_no_reporter = union_async(&ProgressReporter::null());
    let watched = union_async(&reporter);

    assert_same_mesh(
        &with_no_reporter,
        &watched,
        "watching an n-ary union must not change its result",
    );
    let reports = reports.lock().unwrap();
    assert!(
        reports_a_kernel_phase(&reports),
        "a watched n-ary union streams the kernel's phases (saw {})",
        distinct_messages(&reports)
    );

    // The bar never goes backwards, though each boolean in the tree restarts its phases.
    for i in 1..reports.len() {
        assert!(reports[i].0 >= reports[i - 1].0);
    }
}
