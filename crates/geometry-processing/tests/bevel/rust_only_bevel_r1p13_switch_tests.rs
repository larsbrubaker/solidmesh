//! Rust-only checks of the bevel core's R1.P13 build-carried switches and budget notes: the work
//! budget's constructors, side-group and refinement notes and phase tally; the run-out and
//! square-crease-clip gates kept on the budget in force; the level-line routing statics; the phase
//! report; the crease run-out's reading.

use std::rc::Rc;

use indexmap::IndexMap;

use geometry_processing::bevel::level_line::{
    level_line_routing, run_outs_on, square_crease_clips_on, square_crease_end_clips,
    triangle_edges, turn_run_outs_on, turn_square_crease_clips_off, CreaseRunOut, LevelLineWork,
    LevelLineWorkPhase, WorkBudget,
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

#[test]
fn rust_only_the_phase_report_lists_each_metered_phase_with_its_shares() {
    let budget = WorkBudget::new(100).expect("a positive limit");
    {
        let mut units = budget.phase_units.borrow_mut();
        units[LevelLineWorkPhase::Sides as usize] = 1500;
        units[LevelLineWorkPhase::Fill as usize] = 500;
        let mut ticks = budget.phase_ticks.borrow_mut();
        ticks[LevelLineWorkPhase::Sides as usize] = 1_000_000_000;
        ticks[LevelLineWorkPhase::Fill as usize] = 3_000_000_000;
    }
    // Before the first tally nothing is charged to the running phase.
    let report = budget.phase_report();
    let expected = [
        "phase                  units  unit share   seconds  time share",
        "Sides                  1,500      75.0 %      1.00      25.0 %",
        "Fill                     500      25.0 %      3.00      75.0 %",
        "total                  2,000                  4.00",
    ]
    .join("\n");
    assert_eq!(report, expected);
}

#[test]
fn rust_only_a_square_corner_runs_out_one_plus_root_three_radii_down_the_crease() {
    // The round runs along +x under the top face z = 0, its side the face y = 0; the wall is
    // x = 0 and the unpicked crease runs down -z from the corner.
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -5.0),
        Vector3::new(5.0, 0.0, 0.0),
    ];
    let up = Vector3::new(0.0, 0.0, 1.0);
    let front = Vector3::new(0.0, -1.0, 0.0);
    let wall = Vector3::new(-1.0, 0.0, 0.0);
    let run_out = CreaseRunOut::read(&positions, 0, 1, 2, up, front, wall, 1.0, 4)
        .expect("the ball returns to the crease");
    assert!((run_out.reach - (1.0 + 3f64.sqrt())).abs() <= 1e-9);
    assert!((run_out.end_turn_radians - (5.0 * std::f64::consts::PI / 6.0)).abs() <= 1e-9);
    assert!((run_out.canal_turn_radians - std::f64::consts::FRAC_PI_2).abs() <= 1e-15);
    assert_eq!(run_out.wall_contact.len(), 5);
    assert_eq!(run_out.tube_contact.len(), 5);
    let end = run_out.end();
    assert!(run_out.distance_to_crease_line(end) <= 1e-12);
    assert!(near(run_out.wall_contact[4], end, 1e-9));
    // At the end the ball touches the crease's sharp edge: its centre stands r from the line,
    // and always r outside the wall.
    let centre = run_out.centre_at(run_out.end_turn_radians);
    assert!((run_out.distance_to_crease_line(centre) - 1.0).abs() <= 1e-9);
    assert!(((centre - positions[0]).dot(wall) - 1.0).abs() <= 1e-12);
    // A round lying in its wall has no run-out.
    let in_the_wall = Vector3::new(0.0, 1.0, 0.0);
    assert!(CreaseRunOut::read(&positions, 0, 1, 2, up, front, in_the_wall, 1.0, 4).is_none());
}

fn near(a: Vector3, b: Vector3, tolerance: f64) -> bool {
    (a - b).length() <= tolerance
}

#[test]
fn rust_only_an_edges_sense_and_the_run_out_map() {
    // Two triangles on the edge 0-1 along x: a floor (normal +z) and a wall rising on y = 0.
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.5, 1.0, 0.0),
        Vector3::new(0.5, 0.0, 1.0),
        Vector3::new(0.5, 0.0, -1.0),
    ];
    let concave = vec![(0, 1, 2), (1, 0, 3)];
    let convex = vec![(0, 1, 2), (1, 0, 4)];
    let flat = vec![(0, 1, 2), (1, 0, 2)];
    for (indices, expected) in [(concave, 1), (convex, -1), (flat, 0)] {
        let standing = triangle_edges::standing_map(&indices);
        assert_eq!(
            CreaseRunOut::sense(&positions, &indices, &standing, (0, 1)),
            expected
        );
        assert_eq!(
            CreaseRunOut::sense(&positions, &indices, &standing, (0, 4)),
            0
        );
    }
    let mut map = IndexMap::new();
    CreaseRunOut::add(&mut map, 3, (1, 2));
    CreaseRunOut::add(&mut map, 1, (0, 1));
    CreaseRunOut::add(&mut map, 3, (2, 3));
    assert_eq!(map.keys().copied().collect::<Vec<_>>(), vec![3, 1]);
    assert_eq!(map[&3], vec![(1, 2), (2, 3)]);
}

#[test]
fn rust_only_the_foreign_room_meter_counts_per_thread() {
    use geometry_processing::bevel::level_line::foreign_room_counters;
    foreign_room_counters::reset();
    assert_eq!(
        foreign_room_counters::with(|t| t.edges_per_chain.clone()),
        Some(vec![])
    );
    foreign_room_counters::with(|t| {
        t.calls = 3;
        t.pair_calls = 1;
        t.edges_offered = 40;
    });
    foreign_room_counters::read();
    foreign_room_counters::with(|t| t.in_a_pair = true);
    foreign_room_counters::read();
    assert_eq!(
        foreign_room_counters::report(),
        "calls 3 (pairs 1), edges offered 40, edges read 2 (pairs 1)"
    );
    assert_eq!(foreign_room_counters::since(0), 0);
    assert!(foreign_room_counters::since(1) >= 0);
    // Another thread has its own tally.
    std::thread::spawn(|| assert_eq!(foreign_room_counters::with(|t| t.edges_read), 0))
        .join()
        .expect("the other thread's check");
    foreign_room_counters::reset();
    assert_eq!(foreign_room_counters::with(|t| t.edges_read), 0);
}

#[test]
fn rust_only_results_say_whether_they_came_out_and_strip_clips_note_their_outcome() {
    use geometry_processing::bevel::level_line::{
        strip_clip, BandGapRefusal, BandGapResult, GapFillRefusal, GapFillResult, LevelLineTriangle,
    };
    let filled = GapFillResult::default();
    assert!(filled.filled());
    let refused = GapFillResult {
        refusal: GapFillRefusal::RailIsNotMonotone,
        ..GapFillResult::default()
    };
    assert!(!refused.filled());
    let band = BandGapResult::default();
    assert!(band.built());
    assert!(band.chain().is_none());
    let size = BandGapResult {
        refusal: BandGapRefusal::Size,
        ..BandGapResult::default()
    };
    assert!(!size.built());

    assert_eq!(strip_clip::last_outcome(), None);
    strip_clip::note(|| "dropped 3".to_string());
    assert_eq!(strip_clip::last_outcome().as_deref(), Some("dropped 3"));
    let face = LevelLineTriangle {
        a: 4,
        b: 5,
        c: 6,
        tag: 0,
    };
    assert_eq!(strip_clip::opposite_corner(face, 4, 5), 6);
    assert_eq!(strip_clip::opposite_corner(face, 6, 5), 4);
    assert_eq!(strip_clip::opposite_corner(face, 4, 6), 5);
}
