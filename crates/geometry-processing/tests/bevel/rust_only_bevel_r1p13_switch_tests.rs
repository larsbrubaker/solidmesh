//! Rust-only checks of the bevel core's R1.P13 build-carried switches and budget notes: the work
//! budget's constructors, side-group and refinement notes and phase tally; the run-out and
//! square-crease-clip gates kept on the budget in force; the level-line routing statics.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{
    level_line_routing, run_outs_on, square_crease_clips_on, square_crease_end_clips,
    turn_run_outs_on, turn_square_crease_clips_off, LevelLineWork, LevelLineWorkPhase, WorkBudget,
};
use vector_math::Vector3;

#[test]
fn rust_only_a_work_budget_is_a_positive_count_with_the_flow_defaults() {
    assert!(WorkBudget::new(0).is_err());
    assert!(WorkBudget::new(-3).is_err());
    let budget = WorkBudget::new(10).expect("a positive limit");
    assert_eq!(budget.limit, 10);
    assert_eq!(budget.report_throttle_ms, 50);
    assert!(!budget.input_is_float_stored);
    assert!(!budget.run_outs_on.get());
    assert!(!budget.square_crease_clips_off.get());
    assert!(!budget.pair_corner_end_face_field_off.get());
    assert!(!budget.exhausted());
    budget.exhausted_in.set(LevelLineWorkPhase::Fill);
    assert!(budget.exhausted());
}

#[test]
fn rust_only_side_groups_and_the_refinement_queue_are_noted_on_the_budget_in_force() {
    let budget = Rc::new(WorkBudget::new(10).expect("a positive limit"));
    {
        let _scope = LevelLineWork::begin(Some(budget.clone()));
        LevelLineWork::enter_side_group(7, 3);
        LevelLineWork::note_refinement_queue(4, 9);
    }
    assert_eq!(budget.side_group_count.get(), 3);
    assert_eq!(budget.side_group_index.get(), 2);
    assert_eq!(budget.refinement_taken.get(), 4);
    assert_eq!(budget.refinement_left.get(), 9);
    budget.enter_side_group(-1, -2);
    assert_eq!(budget.side_group_count.get(), 0);
    assert_eq!(budget.side_group_index.get(), 0);
    // With no budget in force the notes go nowhere.
    LevelLineWork::enter_side_group(1, 2);
    assert_eq!(budget.side_group_count.get(), 0);
}

#[test]
fn rust_only_the_phase_tally_charges_the_running_phase_from_the_second_tally() {
    let budget = WorkBudget::new(100).expect("a positive limit");
    budget.phase.set(LevelLineWorkPhase::Refinement);
    budget.tally_phase();
    assert_eq!(budget.phase_units.borrow().iter().sum::<i64>(), 0);
    budget.spent.set(30);
    budget.tally_phase();
    assert_eq!(
        budget.phase_units.borrow()[LevelLineWorkPhase::Refinement as usize],
        30
    );
    assert_eq!(budget.phase_tally_units_at.get(), 30);
}

#[test]
fn rust_only_run_outs_turn_on_for_the_build_in_force_until_the_scope_drops() {
    assert!(!run_outs_on());
    {
        // No build: the scope switches nothing.
        let _on = turn_run_outs_on();
        assert!(!run_outs_on());
    }
    let budget = Rc::new(WorkBudget::new(10).expect("a positive limit"));
    let _work = LevelLineWork::begin(Some(budget.clone()));
    assert!(!run_outs_on());
    {
        let _on = turn_run_outs_on();
        assert!(run_outs_on());
        {
            let _again = turn_run_outs_on();
            assert!(run_outs_on());
        }
        assert!(run_outs_on());
    }
    assert!(!run_outs_on());
    assert!(!budget.run_outs_on.get());
}

#[test]
fn rust_only_square_crease_clips_turn_off_for_the_build_in_force_until_the_scope_drops() {
    assert!(square_crease_clips_on());
    let budget = Rc::new(WorkBudget::new(10).expect("a positive limit"));
    let _work = LevelLineWork::begin(Some(budget.clone()));
    {
        let _off = turn_square_crease_clips_off();
        assert!(!square_crease_clips_on());
        // Off, no end clips at all.
        let (positions, indices) = cube_edge();
        assert_eq!(
            square_crease_end_clips(&positions, &indices, &[0, 1], false, (false, false)),
            (None, None)
        );
    }
    assert!(square_crease_clips_on());
}

/// The unit cube's top-front edge along x ending on the x = 1 face: convex, square end, but the
/// end face meets the sides at convex creases, so no square-crease clip.
fn cube_edge() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    (positions, vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)])
}

#[test]
fn rust_only_no_square_crease_clip_where_the_end_creases_are_convex_or_rayed() {
    let (positions, indices) = cube_edge();
    assert_eq!(
        square_crease_end_clips(&positions, &indices, &[0, 1], false, (false, false)),
        (None, None)
    );
    assert_eq!(
        square_crease_end_clips(&positions, &indices, &[0, 1], true, (false, false)),
        (None, None)
    );
}

#[test]
fn rust_only_the_level_line_routing_statics() {
    assert!(level_line_routing::the_running_app_wants_it_on(None));
    level_line_routing::set_first_route(None);
    level_line_routing::set_last_route(Some("level-line".to_string()));
    level_line_routing::set_last_route(Some("existing: open".to_string()));
    assert_eq!(
        level_line_routing::last_route().as_deref(),
        Some("existing: open")
    );
    assert_eq!(
        level_line_routing::first_route().as_deref(),
        Some("level-line")
    );
    // The switch is process-wide, so the test puts back the default it found.
    level_line_routing::set_use_level_line(false);
    assert!(!level_line_routing::use_level_line());
    level_line_routing::reset_to_the_default();
    assert_eq!(
        level_line_routing::use_level_line(),
        level_line_routing::the_running_app_wants_it_on(
            std::env::var(level_line_routing::USE_LEVEL_LINE_VARIABLE)
                .ok()
                .as_deref()
        )
    );
}
