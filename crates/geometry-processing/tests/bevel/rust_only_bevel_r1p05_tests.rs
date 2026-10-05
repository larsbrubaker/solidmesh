//! Rust-only checks of bevel-core package R1.P05's first leaves: the working mesh's edge
//! adjacency, removal and ancestry, the work budget's bar arithmetic and default size, and the
//! run-out field's hull, reading and golden-section least. Their C# coverage comes through the
//! whole-build test classes that land with later packages.

use geometry_processing::bevel::level_line::run_out_field::RunOutField;
use geometry_processing::bevel::level_line::work_budget::{DEFAULT_FLOOR, PHASE_ORDER};
use geometry_processing::bevel::level_line::{
    LevelLineMesh, LevelLineTriangle, LevelLineWorkPhase, WorkBudget,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square as two triangles sharing edge 0-2, built through the mesh's own attach.
fn square() -> LevelLineMesh {
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 1),
        LevelLineTriangle::new(0, 2, 3, 1),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }
    mesh
}

#[test]
fn rust_only_the_working_mesh_keeps_its_edges_as_triangles_leave() {
    let mut mesh = square();
    assert_eq!(mesh.triangles_on(2, 0), &[0, 1]);
    assert!(mesh.has_edge(1, 0));
    assert_eq!(mesh.centroid(1), v(1.0 / 3.0, 2.0 / 3.0, 0.0));
    assert_eq!(LevelLineMesh::opposite_corner(mesh.triangles()[0], 0, 2), 1);

    mesh.remove_triangle(0);
    mesh.remove_triangle(0);
    assert_eq!(mesh.removed_count(), 1);
    assert_eq!(mesh.triangles_on(0, 2), &[1]);
    assert!(!mesh.has_edge(0, 1));

    // The polyline 0-2 walls the square in two; the half round seed 3 goes.
    let mut mesh = square();
    assert_eq!(mesh.remove_enclosed(3, &[0, 2], &|_| true), 1);
    assert!(mesh.is_removed(1) && !mesh.is_removed(0));

    let mut mesh = square();
    assert!(!mesh.is_answered_clean(0, 1));
    mesh.answer_clean(0, 1);
    assert!(!mesh.is_answered_clean(0, 1));
    mesh.forget_clean_edges();
    mesh.answer_clean(1, 0);
    assert!(mesh.is_answered_clean(0, 1));
    assert!(mesh.has_no_height(mesh.triangles()[0], 0.8));
    assert!(!mesh.has_no_height(mesh.triangles()[0], 0.5));
}

#[test]
fn rust_only_ancestries_merge_to_one_input_edge_or_none() {
    assert_eq!(LevelLineMesh::merge((3, 3), (5, 5)), (3, 5));
    assert_eq!(LevelLineMesh::merge((5, 3), (3, 5)), (3, 5));
    assert_eq!(LevelLineMesh::merge((4, 4), (4, 4)), (4, 4));
    assert_eq!(LevelLineMesh::merge((1, 2), (3, 3)), (-1, -1));
    assert_eq!(LevelLineMesh::merge((-1, -1), (3, 3)), (-1, -1));
}

#[test]
fn rust_only_the_bar_spans_one_loop_of_the_route() {
    let weights = WorkBudget::the_weights();
    let starts = WorkBudget::the_starts();
    assert!((weights.iter().sum::<f64>() - 1.0).abs() < 1e-12);
    assert_eq!(starts[PHASE_ORDER[0] as usize], 0.0);
    assert!((starts[LevelLineWorkPhase::Fill as usize] - 0.97).abs() < 1e-12);
    assert_eq!(WorkBudget::default_limit(10), DEFAULT_FLOOR + 40_000);
    assert_eq!(WorkBudget::default_limit(-5), DEFAULT_FLOOR);
}

#[test]
fn rust_only_a_run_out_wedge_reads_under_its_profile() {
    // A hull drops the dip at s = 1 and keeps the shoulder.
    let hull = RunOutField::upper_hull(&[(0.0, 1.0), (1.0, 0.5), (2.0, 1.0), (4.0, 0.0)]);
    assert_eq!(hull, vec![(0.0, 1.0), (2.0, 1.0), (4.0, 0.0)]);

    let field = RunOutField::from_profile(
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        4.0,
        &[(0.0, 1.0), (1.0, 0.5), (2.0, 1.0)],
    );
    assert_eq!(field.slope, 0.5);
    assert_eq!(field.width, 1.0);
    // On the crease inside the wedge: the nearest bound is the width cylinder, one away.
    assert!((field.read(v(1.0, 0.0, 0.0)) + 1.0).abs() < 1e-12);
    // Past P* the apex is the nearest point.
    assert!((field.read(v(7.0, 4.0, 0.0)) - 5.0).abs() < 1e-12);

    let least = RunOutField::least_of(&|x| (x - 0.3) * (x - 0.3), 0.0, 1.0);
    assert!((least - 0.3).abs() < 1e-9);
    assert_eq!(RunOutField::least_of(&|x| x, 2.0, 1.0), 2.0);
}

fn budget(limit: i64, token: polygon_mesh::CancelToken) -> std::rc::Rc<WorkBudget> {
    use std::cell::{Cell, RefCell};
    std::rc::Rc::new(WorkBudget {
        cancellation_token: token,
        progress: None,
        limit,
        input_is_float_stored: false,
        report_throttle_ms: 50,
        spent: Cell::new(0),
        phase: Cell::new(LevelLineWorkPhase::None),
        exhausted_in: Cell::new(LevelLineWorkPhase::None),
        since_cancel_check: Cell::new(0),
        last_report_ms: Cell::new(0),
        last_fraction: Cell::new(0.0),
        side_group_index: Cell::new(0),
        side_group_count: Cell::new(0),
        refinement_taken: Cell::new(0),
        refinement_left: Cell::new(0),
        phase_started_at: Cell::new(0),
        phase_estimates: RefCell::new(Default::default()),
        phase_units: RefCell::new(Default::default()),
        phase_ticks: RefCell::new(Default::default()),
        phase_tally_units_at: Cell::new(0),
        phase_tally_ticks_at: Cell::new(0),
    })
}

#[test]
fn rust_only_a_budget_runs_out_or_is_cancelled_as_an_error() {
    use geometry_processing::bevel::level_line::{LevelLineWork, WorkError};
    // No budget in force: charging is free.
    assert!(LevelLineWork::charge(1_000_000).is_ok());

    let spent = budget(10, polygon_mesh::CancelToken::new());
    let _scope = LevelLineWork::begin(Some(spent.clone()));
    assert!(LevelLineWork::charge(10).is_ok());
    match LevelLineWork::charge(1) {
        Err(WorkError::BudgetExceeded(b)) => {
            assert_eq!(b.exhausted_in.get(), LevelLineWorkPhase::Refinement);
            assert_eq!(
                WorkError::BudgetExceeded(b).to_string(),
                "the level line work budget of 10 units ran out in Refinement"
            );
        }
        other => panic!("expected the budget to run out, got {other:?}"),
    }

    let token = polygon_mesh::CancelToken::new();
    let watched = budget(1_000_000, token.clone());
    let _inner = LevelLineWork::begin(Some(watched));
    assert!(LevelLineWork::poll_cancel().is_ok());
    token.cancel();
    assert!(matches!(
        LevelLineWork::poll_cancel(),
        Err(WorkError::Cancelled)
    ));
    // The charge reads the token only every 256 charges.
    let mut answer = Ok(());
    for _ in 0..256 {
        answer = LevelLineWork::charge(1);
        if answer.is_err() {
            break;
        }
    }
    assert!(matches!(answer, Err(WorkError::Cancelled)));
}

#[test]
fn rust_only_field_queries_read_the_wedge_and_the_pair_corner_exactly() {
    use geometry_processing::bevel::level_line::{
        ChainDistanceField, PairCornerEndFaceField, ScalarField,
    };
    let wedge = RunOutField::cone(v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 4.0, 0.5, f64::INFINITY);
    assert_eq!(
        wedge.value_at(v(2.0, 3.0, 0.0)).ok(),
        Some(wedge.read(v(2.0, 3.0, 0.0)))
    );
    let least = wedge
        .min_along_segment(v(1.0, 2.0, 0.0), v(3.0, 2.0, 0.0))
        .expect("no budget");
    assert!(
        least.value <= wedge.read(v(1.0, 2.0, 0.0)) && least.value <= wedge.read(v(3.0, 2.0, 0.0))
    );
    let most = wedge
        .max_over_triangle(v(1.0, 0.0, 0.0), v(2.0, 1.0, 0.0), v(9.0, 0.0, 0.0), 0.01)
        .expect("no budget");
    assert_eq!(most.point, v(9.0, 0.0, 0.0));

    // A square corner of a 2-round: the axis stands 2 inside both faces.
    let field = PairCornerEndFaceField::of(
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 1.0),
        v(-1.0, 0.0, 0.0),
        2.0,
    );
    assert_eq!(field.axis_point, v(0.0, -2.0, -2.0));
    assert_eq!(
        PairCornerEndFaceField::quadratic_roots(1.0, 0.0, -4.0),
        vec![2.0, -2.0]
    );
    assert_eq!(
        PairCornerEndFaceField::quadratic_roots(0.0, 2.0, -4.0),
        vec![2.0]
    );
    let segment_least = field
        .min_along_segment(v(0.0, -1.0, -1.0), v(0.0, -3.0, -3.0))
        .expect("no budget");
    // The exact least is no greater than any sampled reading along the segment.
    for i in 0..=20 {
        let p = v(0.0, -1.0, -1.0) + (v(0.0, -2.0, -2.0) * (i as f64 / 20.0));
        assert!(segment_least.value <= field.read(p) + 1e-12);
    }
}
