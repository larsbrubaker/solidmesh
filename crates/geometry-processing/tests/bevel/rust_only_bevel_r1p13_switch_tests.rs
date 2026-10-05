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

#[test]
fn rust_only_the_foreign_edge_tree_splits_at_the_longest_axis_median() {
    use geometry_processing::bevel::level_line::{
        foreign_edge_room, ForeignApproach, ForeignEdgeIndex, ForeignEdgeIndexNode,
    };
    // Six unit edges standing along x at x = 5, 4, 3, 2, 1, 0 (edge i at x = 5 - i).
    let mut ends = Vec::new();
    for i in 0..6 {
        let x = 5.0 - i as f64;
        ends.push(Vector3::new(x, 0.0, 0.0));
        ends.push(Vector3::new(x, 1.0, 0.0));
    }
    let mut order: Vec<i32> = (0..6).collect();
    let mut built = Vec::new();
    let root = ForeignEdgeIndex::add(&mut built, &ends, &mut order, 0, 6);
    assert_eq!(root, 0);
    assert_eq!(built.len(), 3);
    assert_eq!(order, vec![5, 4, 3, 2, 1, 0]);
    assert_eq!(
        built[0],
        ForeignEdgeIndexNode::new(
            0,
            6,
            1,
            2,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(5.0, 1.0, 0.0)
        )
    );
    assert_eq!((built[1].start, built[1].count, built[1].left), (0, 3, -1));
    assert_eq!(built[1].max, Vector3::new(2.0, 1.0, 0.0));
    assert_eq!((built[2].start, built[2].count), (3, 3));

    let edges: Vec<(i32, i32)> = (0..6).map(|i| (2 * i, 2 * i + 1)).collect();
    let index = ForeignEdgeIndex::new(edges.clone(), ends, built);
    assert_eq!(index.count(), 6);
    assert_eq!(index.edges(), &edges[..]);
    let all = foreign_edge_room::every_edge(std::slice::from_ref(&index), None);
    assert_eq!(all.len(), 6);
    assert_eq!(all[0], (0, 1, 0.0));
    let not_the_first = |edge: (i32, i32)| edge.0 == 0;
    let some = foreign_edge_room::every_edge(&[index], Some(&not_the_first));
    assert_eq!(some.len(), 5);
    assert_eq!(some[0], (2, 3, 0.0));
    let approach = ForeignApproach::new(1.5, vec![((0, 1), 1.5)], 1);
    assert_eq!(approach.read, 1);
}

#[test]
fn rust_only_foreign_edge_regions_read_each_chains_growth() {
    use std::cell::RefCell;
    use std::collections::HashSet;

    use geometry_processing::bevel::level_line::foreign_edge_regions::{chain_ends, open_chain_of};
    use geometry_processing::bevel::level_line::{
        ForeignEdgeRegions, ForeignEdgeRegionsChainGrowth, GrowingCreaseRuns, LevelLineMesh,
    };

    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    let runs = Rc::new(GrowingCreaseRuns::new(
        mesh.clone(),
        (0, 1),
        Rc::new(|p: Vector3| p.length()),
        None,
        None,
    ));
    assert_eq!(runs.chain_edges, vec![(0, 1)]);
    assert!((runs.is_own_side)(2) && !(runs.is_own_side)(3));
    let own: Rc<dyn Fn(i32) -> bool> = Rc::new(|tag| tag == 3);
    let legs = GrowingCreaseRuns::new(
        mesh.clone(),
        (0, 1),
        Rc::new(|p: Vector3| p.length()),
        Some(vec![(0, 1), (1, 2)]),
        Some(own),
    );
    assert_eq!(legs.chain_edges, vec![(0, 1), (1, 2)]);
    assert!((legs.is_own_side)(3));
    let mut neighbours = std::collections::HashMap::new();
    GrowingCreaseRuns::add(&mut neighbours, 4, 5);
    GrowingCreaseRuns::add(&mut neighbours, 4, 6);
    assert_eq!(neighbours[&4], vec![5, 6]);

    let mut growth = ForeignEdgeRegionsChainGrowth::new(
        runs.clone(),
        HashSet::from([0, 1]),
        HashSet::new(),
        false,
        true,
    );
    growth.set_aside.insert((2, 3));
    let several = ForeignEdgeRegions::new(
        mesh.clone(),
        Default::default(),
        Some(vec![Some(growth), None]),
    );
    assert!(several.set_aside_for(0, (2, 3)));
    assert!(!several.set_aside_for(1, (2, 3)));
    assert!(several.sets_aside_any(0));
    assert!(!several.sets_aside_any(1));
    assert_eq!(several.runs_out(0), (false, true));
    assert_eq!(several.runs_out(1), (true, true));
    assert!(several
        .growing_for(0)
        .is_some_and(|r| Rc::ptr_eq(&r, &runs)));
    assert!(several.growing_for(1).is_none());

    let mut one = ForeignEdgeRegions::new(mesh, Default::default(), None);
    assert!(one.growing_for(0).is_none());
    one.growing = Some(runs.clone());
    assert!(one.growing_for(5).is_some());
    assert_eq!(one.runs_out(0), (true, true));
    assert!(!one.set_aside_for(0, (2, 3)));

    let chains = vec![vec![(0, 1)], vec![], vec![(4, 5), (5, 6)]];
    assert_eq!(
        open_chain_of(Some(&chains), Some(&[false, false, false])),
        Some(&chains[0])
    );
    assert_eq!(
        open_chain_of(Some(&chains), Some(&[true, false, false])),
        Some(&chains[2])
    );
    assert_eq!(open_chain_of(None, Some(&[false])), None);
    assert_eq!(
        chain_ends(&HashSet::from([(4, 5), (5, 6)])),
        HashSet::from([4, 6])
    );
    assert!(chain_ends(&HashSet::from([(0, 1), (1, 2), (0, 2)])).is_empty());
}

#[test]
fn rust_only_end_clips_and_a_vertex_reading() {
    use std::cell::RefCell;

    use geometry_processing::bevel::level_line::{
        ChainField, EndRayBound, LevelLineMesh, ScalarField, VertexFieldReading, VertexSide,
        WorkError,
    };

    let points = [Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)];
    assert_eq!(ChainField::find_end_clips(&points, 4.0, (None, None)), None);
    let bound = EndRayBound {
        plane_normal: Vector3::new(2.0, 0.0, 0.0),
        ..EndRayBound::default()
    };
    let clips = ChainField::find_end_clips(&points, 4.0, (None, Some(bound))).expect("one clip");
    assert_eq!(clips.len(), 1);
    assert_eq!(clips[0].vertex, points[1]);
    assert_eq!(clips[0].normal, Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(clips[0].parameter, 4.0);
    let both = ChainField::find_end_clips(&points, 4.0, (Some(bound), Some(bound))).expect("two");
    assert_eq!((both[0].parameter, both[1].parameter), (0.0, 4.0));

    struct Height;
    impl ScalarField for Height {
        fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
            Ok(point.z)
        }
    }
    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    assert!(VertexFieldReading::new(mesh.clone(), Rc::new(Height), -1.0).is_err());
    assert!(VertexFieldReading::new(mesh.clone(), Rc::new(Height), f64::NAN).is_err());
    let reading = VertexFieldReading::new(mesh, Rc::new(Height), 0.5).expect("a length");
    assert_eq!(reading.snap, 0.5);
    assert_eq!(
        reading.field().value_at(Vector3::new(0.0, 0.0, 2.0)).ok(),
        Some(2.0)
    );
    assert!(reading.values.borrow().is_empty());
    assert_eq!(VertexFieldReading::side_for(0.5, 0.5), VertexSide::On);
    assert_eq!(VertexFieldReading::side_for(-0.6, 0.5), VertexSide::Inside);
    assert_eq!(VertexFieldReading::side_for(0.6, 0.5), VertexSide::Outside);
    assert_eq!(
        VertexFieldReading::side_for(f64::NAN, 0.5),
        VertexSide::Outside
    );
}

#[test]
fn rust_only_mesh_pieces_know_whether_they_have_a_closed_form() {
    use geometry_processing::bevel::level_line::ChainFieldMeshPiece;
    let a = Vector3::new(0.0, 0.0, 0.0);
    let b = Vector3::new(2.0, 0.0, 0.0);
    let c = Vector3::new(0.0, 2.0, 0.0);
    let segment = ChainFieldMeshPiece::segment(a, b);
    assert!(segment.has_closed_form() && !segment.is_triangle);
    assert_eq!(
        segment.closest_to(Vector3::new(1.0, 5.0, 0.0)),
        Vector3::new(1.0, 0.0, 0.0)
    );
    let triangle = ChainFieldMeshPiece::triangle(a, b, c);
    assert!(triangle.has_closed_form() && triangle.is_triangle);
    let custom = ChainFieldMeshPiece::custom(Rc::new(|_| Vector3::new(7.0, 7.0, 7.0)));
    assert!(!custom.has_closed_form());
    assert_eq!(custom.closest_to(a), Vector3::new(7.0, 7.0, 7.0));
}

#[test]
fn rust_only_a_chain_cut_short_by_an_earlier_band_starts_where_its_edge_enters_the_mesh() {
    use geometry_processing::bevel::level_line::band_gap_in_turn::{
        ends_on_the_mesh, nearest_on_the_edge, takes_open_chains_in_turn,
    };
    use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, GapLoop};

    // The chain 0 - 1 - 2 along x; vertex 0 was cut away, and the earlier cut left 3 at x = 0.4
    // on the first edge (and 4 off the line).
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.4, 0.0, 0.0),
        Vector3::new(0.2, 0.5, 0.0),
    ];
    let neighbours: IndexMap<i32, Vec<i32>> =
        IndexMap::from([(4, vec![3]), (1, vec![3, 2]), (2, vec![1]), (3, vec![1, 4])]);
    assert_eq!(nearest_on_the_edge(0, 1, &positions, &neighbours), 3);
    assert_eq!(nearest_on_the_edge(0, 0, &positions, &neighbours), -1);
    assert_eq!(
        ends_on_the_mesh(&[0, 1, 2], &positions, &neighbours),
        Some(vec![3, 1, 2])
    );
    assert_eq!(
        ends_on_the_mesh(&[1, 2], &positions, &neighbours),
        Some(vec![1, 2])
    );
    // With nothing on the first edge but its far end, the chain starts at that end.
    let no_cut: IndexMap<i32, Vec<i32>> = IndexMap::from([(1, vec![2]), (2, vec![1])]);
    assert_eq!(
        ends_on_the_mesh(&[0, 1, 2], &positions, &no_cut),
        Some(vec![1, 2])
    );
    assert_eq!(ends_on_the_mesh(&[0, 1], &positions, &no_cut), None);

    let open_loop = GapLoop::new(1, false, vec![0, 1], vec![0.0, 1.0], 0, false, None);
    let gap = BandGapResult {
        loops: vec![open_loop.clone()],
        ..BandGapResult::default()
    };
    assert!(takes_open_chains_in_turn(Some(&gap), &[false, false]));
    assert!(!takes_open_chains_in_turn(Some(&gap), &[false]));
    assert!(!takes_open_chains_in_turn(Some(&gap), &[false, true]));
    assert!(!takes_open_chains_in_turn(None, &[false, false]));
    let shared = BandGapResult {
        loops: vec![GapLoop::new(
            1,
            false,
            vec![0, 1],
            vec![0.0, 1.0],
            0,
            false,
            Some(vec![0, 1]),
        )],
        ..BandGapResult::default()
    };
    assert!(!takes_open_chains_in_turn(Some(&shared), &[false, false]));
    let refused = BandGapResult {
        refusal: BandGapRefusal::Size,
        loops: vec![open_loop],
        ..BandGapResult::default()
    };
    assert!(!takes_open_chains_in_turn(Some(&refused), &[false, false]));
}

#[test]
fn rust_only_the_rail_pairing_takes_the_cheaper_opposite_senses() {
    use geometry_processing::bevel::level_line::gap_fill::try_pair;
    use geometry_processing::bevel::level_line::{GapFillRailSenses, ZipRail};
    let rail = |reversed: bool| {
        Some(Rc::new(ZipRail {
            vertices: vec![0, 1],
            parameters: vec![0.0, 1.0],
            reversed,
        }))
    };
    let both = GapFillRailSenses::new(rail(false), 0.5, rail(true), 0.1, -1);
    let forward_only = GapFillRailSenses::new(rail(false), 0.0, None, 0.0, 3);
    let backward_only = GapFillRailSenses::new(None, 0.0, rail(true), 0.0, 4);
    // Both ways round on both: left backward + right forward costs 0.6, the other 0.6 too - a tie
    // goes to left forward.
    let (left, right) = try_pair(&both, &both).expect("a pairing");
    assert!(!left.reversed && right.reversed);
    let cheap = GapFillRailSenses::new(rail(false), 0.0, rail(true), 0.0, -1);
    let (left, right) = try_pair(&both, &cheap).expect("a pairing");
    assert!(left.reversed && !right.reversed);
    assert!(try_pair(&forward_only, &forward_only).is_none());
    let (left, right) = try_pair(&forward_only, &backward_only).expect("a pairing");
    assert!(!left.reversed && right.reversed);
}

#[test]
fn rust_only_a_split_record_describes_itself_in_one_line() {
    use geometry_processing::bevel::level_line::{
        LevelLineRule, PointPlace, SplitMeterSplitKind, SplitMeterSplitRecord,
    };
    let record = SplitMeterSplitRecord {
        point: Vector3::new(1.0, -2.5, 1.0 / 3.0),
        kind: SplitMeterSplitKind::OnEdge,
        rule: LevelLineRule::EdgeDip,
        triangle: 7,
        corner_a: 1,
        corner_b: 2,
        corner_c: 3,
        edge_from: 1,
        edge_to: 2,
        triangles_on_the_edge: 2,
        distance_ab: 0.0,
        distance_bc: 1.5e-7,
        distance_ca: 2.0,
        edge_tolerance: 1e-9,
        place: PointPlace::OnAnEdge,
        judged_from: 1,
        judged_to: 2,
        ..SplitMeterSplitRecord::default()
    };
    assert_eq!(
        record.describe(),
        "on-edge split of (1,2), 2 triangle(s) on it; rule EdgeDip; triangle 7 [1,2,3]; off its edges \
         AB 0.000E+000, BC 1.500E-007, CA 2.000E+000 against 1.000E-009 - stands on (1,2); \
         at (1.000000, -2.500000, 0.333333)"
    );
    let interior = SplitMeterSplitRecord {
        kind: SplitMeterSplitKind::Free,
        place: PointPlace::AtACorner,
        judged_from: 4,
        ..record
    };
    let text = interior.describe();
    assert!(
        text.starts_with("free vertex (fill); rule EdgeDip;"),
        "{text}"
    );
    assert!(text.contains(" - stands at corner 4; "), "{text}");
}

#[test]
fn rust_only_a_trimmed_run_gets_its_end_back_whole() {
    use geometry_processing::bevel::level_line::dying_crease_fill::is_one_loop_round_an_open_chain;
    use geometry_processing::bevel::level_line::end_face_fill::with_whole_end;
    use geometry_processing::bevel::level_line::{BandGapResult, GapLoop};

    let whole = GapLoop::new(
        1,
        false,
        vec![10, 11, 12, 13, 14],
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
        2,
        false,
        None,
    );
    let trimmed = GapLoop::new(
        1,
        false,
        vec![20, 11, 12],
        vec![0.5, 1.0, 2.0],
        2,
        false,
        None,
    );
    let tail = with_whole_end(&trimmed, &whole, true);
    assert_eq!(tail.vertices, vec![20, 11, 12, 13, 14]);
    assert_eq!(tail.parameters, vec![0.5, 1.0, 2.0, 3.0, 4.0]);
    assert_eq!(tail.chain, 2);
    // Its head (20) is not on the whole run, so the run comes back as it was.
    let head = with_whole_end(&trimmed, &whole, false);
    assert_eq!(head, trimmed);
    let off_the_run = GapLoop::new(1, false, vec![30, 31], vec![0.0, 1.0], 2, false, None);
    assert_eq!(with_whole_end(&off_the_run, &whole, true), off_the_run);
    let head_on = GapLoop::new(1, false, vec![12, 40], vec![2.0, 2.5], 2, false, None);
    assert_eq!(
        with_whole_end(&head_on, &whole, false).vertices,
        vec![10, 11, 12, 40]
    );

    assert!(!is_one_loop_round_an_open_chain(None));
    assert!(!is_one_loop_round_an_open_chain(Some(
        &BandGapResult::default()
    )));
}

#[test]
fn rust_only_ray_reaches_are_their_far_ends_per_segment() {
    use geometry_processing::bevel::level_line::{ChainField, ChainFieldExtension};
    assert_eq!(ChainField::reaches_of(None), None);
    let ray = ChainFieldExtension {
        origin: Vector3::new(1.0, 0.0, 0.0),
        direction: Vector3::new(0.0, 1.0, 0.0),
        reach: 2.0,
        ..ChainFieldExtension::default()
    };
    let rays = vec![None, Some(vec![ray, ray])];
    assert_eq!(
        ChainField::reaches_of(Some(&rays)),
        Some(vec![None, Some(vec![Vector3::new(1.0, 2.0, 0.0); 2])])
    );
}

#[test]
fn rust_only_a_corner_piece_keeps_its_rails_past_a_free_end() {
    use geometry_processing::bevel::level_line::corner_fill_free_ends::{rails_past_an_end, slice};
    use geometry_processing::bevel::level_line::CornerFillPiece;
    let piece = |parameters: Vec<f64>| CornerFillPiece {
        owner: 3,
        vertices: (10..10 + parameters.len() as i32).collect(),
        parameters,
    };
    // A run at the end inside the piece: down one rail, round the end, back up the other.
    let through = piece(vec![1.0, 0.5, 0.0, 0.0, 0.0, 0.5, 1.0]);
    let rails = rails_past_an_end(&through, 0.0, 1e-9);
    assert_eq!(rails.len(), 2);
    assert_eq!(rails[0].vertices, vec![10, 11, 12]);
    assert_eq!(rails[1].vertices, vec![14, 15, 16]);
    assert_eq!(rails[1].owner, 3);
    // A run at the piece's start keeps only its inner vertex.
    let starts = piece(vec![0.0, 0.0, 0.5, 1.0]);
    assert_eq!(
        rails_past_an_end(&starts, 0.0, 1e-9)[0].vertices,
        vec![11, 12, 13]
    );
    // Wholly at the end, or nowhere near it: as it was.
    let all = piece(vec![0.0, 0.0]);
    assert_eq!(rails_past_an_end(&all, 0.0, 1e-9), vec![all.clone()]);
    let none = piece(vec![0.5, 1.0]);
    assert_eq!(rails_past_an_end(&none, 0.0, 1e-9), vec![none.clone()]);
    // A single vertex at each end keeps the piece whole.
    let tips = piece(vec![0.0, 0.5, 0.0]);
    assert_eq!(rails_past_an_end(&tips, 0.0, 1e-9), vec![tips.clone()]);
    assert_eq!(slice(&through, 2, 4).parameters, vec![0.0, 0.0, 0.0]);
}
