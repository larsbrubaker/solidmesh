// Rust-only pins for the surface part builder's first slice and how it uses the operation's pick
// memo (the memo's own rules are pinned in `part_pick_memo_tests`): the settling's mapping of a
// verdict onto loops and back into the whole pick's numbering; the rim loops; the memo key; and
// the kernel hand-off's pure readings.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::level_line::{
    BandGapResult, BandProfile, LevelLineWorkPhase, RoomReading, RoomReadingKind, RoomSource,
};
use solidmesh::bevel::part::surface_part_builder::level_line_kernel_handoff::{
    the_budget_for, the_edge_key, the_room_to_hand_the_build, the_size_refusal_for,
    the_vertex_to_blame,
};
use solidmesh::bevel::part::surface_part_builder::BevelSurfacePartBuilder;
use solidmesh::bevel::part::{BevelPickMemo, BevelProgress, BevelProgressPhase};
use solidmesh::bevel::{BevelAdvisoryCode, BevelRebuildAdvisories};
use solidmesh::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FeatureEdgeSet};
use solidmesh_math::Color;
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::CancelToken;

/// A cube with its top rim picked as group 0 and its bottom rim as group 1.
fn cube_builder(memo: Option<Arc<BevelPickMemo>>) -> BevelSurfacePartBuilder {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let features = Arc::new(FeatureEdgeSet::create(&mesh));
    let resolution = |top: bool| BevelEdgeGroupResolution {
        group: BevelEdgeGroup::default(),
        features: features.clone(),
        failure_reason: None,
        seed_edge_index: 0,
        edges: features
            .edges()
            .iter()
            .filter(|edge| (edge.start.z > 0.0 && edge.end.z > 0.0) == top)
            .filter(|edge| (edge.start.z < 0.0 && edge.end.z < 0.0) || top)
            .copied()
            .collect(),
        chain: None,
        face_group: None,
    };
    let graph = mesh.get_mesh_edge_graph();

    BevelSurfacePartBuilder::new(
        vec![BevelEdgeGroup::default(), BevelEdgeGroup::default()],
        vec![resolution(true), resolution(false)],
        vec![true, true],
        Rc::new(BevelRebuildAdvisories::new(None::<Vec<_>>)),
        Arc::new(mesh),
        graph,
        1.0,
        8,
        true,
        Matrix4X4::IDENTITY,
        Color::default(),
        None,
        None,
        memo,
    )
}

#[test]
fn rust_only_a_verdict_is_used_only_where_it_falls_on_whole_loops() {
    let loops = vec![
        HashSet::from([1, 2, 3]),
        HashSet::from([4, 5]),
        HashSet::from([6]),
    ];

    assert_eq!(
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(&loops, &[HashSet::from([4, 5])]),
        Some(vec![1])
    );
    // Half a loop.
    assert_eq!(
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(&loops, &[HashSet::from([1])]),
        None
    );
    // Nobody, and everybody.
    assert_eq!(
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(&loops, &[HashSet::from([9])]),
        None
    );
    assert_eq!(
        BevelSurfacePartBuilder::the_loops_the_settling_condemns(
            &loops,
            &[HashSet::from([1, 2, 3, 4, 5]), HashSet::from([6])]
        ),
        None
    );
}

#[test]
fn rust_only_a_reason_is_put_back_into_the_whole_picks_numbering() {
    let alive = [3, 5, 9];
    assert_eq!(
        BevelSurfacePartBuilder::in_terms_of_the_whole_pick(RoomSource::foreign(1), &alive),
        RoomSource::foreign(5)
    );
    let crease = RoomSource::growing_crease(2, Vector3::new(1.0, 2.0, 3.0));
    let moved = BevelSurfacePartBuilder::in_terms_of_the_whole_pick(crease, &alive);
    assert_eq!(moved.chain, 9);
    assert!(moved.is_a_growing_crease());
    assert_eq!(
        BevelSurfacePartBuilder::in_terms_of_the_whole_pick(RoomSource::self_room(7), &alive),
        RoomSource::self_room(-1)
    );
    assert_eq!(
        BevelSurfacePartBuilder::in_terms_of_the_whole_pick(
            RoomSource::chain_to_chain(0, 2),
            &alive
        ),
        RoomSource::chain_to_chain(3, 9)
    );
    assert_eq!(
        BevelSurfacePartBuilder::in_terms_of_the_whole_pick(
            RoomSource::new(RoomReadingKind::None, 4, 4),
            &alive
        ),
        RoomSource::nothing()
    );
}

#[test]
fn rust_only_a_pick_whose_chains_touch_is_not_the_settlings_to_judge() {
    let positions = vec![Vector3::new(0.0, 0.0, 0.0); 4];
    let indices = vec![(0, 1, 2), (0, 2, 3)];
    let answer = BevelSurfacePartBuilder::the_settled_pick(
        &positions,
        &indices,
        &[vec![0, 1], vec![1, 2]],
        &[false, false],
        None,
        BandProfile::Fillet,
        1.0,
        None,
        0.0,
        f64::INFINITY,
    );
    assert!(matches!(answer, Ok(None)));
}

#[test]
fn rust_only_the_rim_loops_are_made_once_per_group_and_scope_a_refusal() {
    let mut builder = cube_builder(None);

    let top = builder.the_rim_loops_of_the_group(0);
    assert_eq!(top.len(), 1);
    assert_eq!(top[0].edges.len(), 4);
    // The same objects on every ask.
    assert_eq!(builder.the_rim_loops_of_the_group(0), top);

    let loops = builder.the_rim_loops_of(&[1, 0]);
    assert_eq!(loops.len(), 2);
    assert_ne!(loops[0], loops[1]);
    assert_eq!(BevelSurfacePartBuilder::the_groups_of(&loops), vec![0, 1]);
    assert_eq!(BevelSurfacePartBuilder::the_edges_of(&loops).len(), 8);

    let guilty: HashSet<i32> = top[0].mesh_edges.iter().take(1).copied().collect();
    assert_eq!(
        BevelSurfacePartBuilder::the_loops_holding(&loops, &guilty),
        top
    );
    assert!(BevelSurfacePartBuilder::the_loops_holding(&loops, &HashSet::new()).is_empty());
}

#[test]
fn rust_only_the_memo_key_carries_the_stamp_profile_snap_turn_and_chains() {
    let memo = Arc::new(BevelPickMemo::new());
    let mut builder = cube_builder(Some(memo.clone()));
    assert!(Arc::ptr_eq(builder.memo(), &memo));

    let stamp = builder.the_stamp();
    assert_eq!(
        stamp,
        BevelPickMemo::the_stamp_of(&create_cube(10.0, 10.0, 10.0))
    );
    assert_eq!(
        BevelSurfacePartBuilder::the_key_of(&[vec![1, 2, 3], vec![4]], &[true, false]),
        "closed:1,2,3;open:4"
    );
    assert_eq!(
        builder.the_memo_key_of(&[vec![1, 2]], &[false], BandProfile::Chamfer, 1e-9),
        format!("{stamp}|Chamfer|1E-09|Infinity|open:1,2")
    );

    // A refusal's room is remembered as the pick's floor, with the chain its pinch named.
    let gap = BandGapResult {
        room: 0.5,
        pinch_probes: 2,
        room_set_by: RoomSource::self_room(0),
        ..BandGapResult::default()
    };
    let named = builder.the_chain_named_with_its_room_remembered(
        &gap,
        RoomSource::foreign(0),
        &[vec![1, 2]],
        &[false],
        BandProfile::Fillet,
        None,
        0.6,
    );
    assert_eq!(named, 0);
    let key = builder.the_memo_key_of(&[vec![1, 2]], &[false], BandProfile::Fillet, 1e-9);
    assert_eq!(
        memo.the_room_under_its_floor(&key, Some(2.0), Some(0.6)),
        Some(0.5)
    );
    assert_eq!(
        memo.the_pinched_chain_under_the_floor(&key, None, Some(0.6)),
        0
    );
}

#[test]
fn rust_only_a_refusal_names_its_pinch_its_floors_pinch_or_a_foreign_chain() {
    let pinched = BandGapResult {
        pinch_probes: 1,
        room_set_by: RoomSource::chain_to_chain(2, 3),
        ..BandGapResult::default()
    };
    assert_eq!(
        BevelSurfacePartBuilder::the_chain_the_pinch_named(&pinched),
        2
    );
    let foreign_pinch = BandGapResult {
        pinch_probes: 1,
        room_set_by: RoomSource::foreign(2),
        ..BandGapResult::default()
    };
    assert_eq!(
        BevelSurfacePartBuilder::the_chain_the_pinch_named(&foreign_pinch),
        -1
    );

    let handed = BandGapResult::default();
    assert_eq!(
        BevelSurfacePartBuilder::the_chain_the_refusal_names(&handed, &RoomSource::self_room(1), 5),
        5
    );
    assert_eq!(
        BevelSurfacePartBuilder::the_chain_the_refusal_names(&handed, &RoomSource::foreign(1), -1),
        1
    );
    assert_eq!(
        BevelSurfacePartBuilder::the_chain_the_refusal_names(
            &handed,
            &RoomSource::chain_to_chain(1, 2),
            -1
        ),
        -1
    );
}

#[test]
fn rust_only_the_kernel_handoff_reads() {
    let mut reading = RoomReading {
        read: true,
        room: 1.5,
        ..RoomReading::default()
    };
    assert_eq!(the_room_to_hand_the_build(Some(&reading)), Some(1.5));
    reading.chains_are_apart = false;
    assert_eq!(the_room_to_hand_the_build(Some(&reading)), None);
    assert_eq!(the_room_to_hand_the_build(None), None);

    assert_eq!(
        the_size_refusal_for(&RoomSource::chain_to_chain(0, 1)),
        BevelAdvisoryCode::SizeCollidesAcrossFace
    );
    assert_eq!(
        the_size_refusal_for(&RoomSource::growing_crease(0, Vector3::default())),
        BevelAdvisoryCode::SizeReachesPastCrease
    );
    assert_eq!(
        the_size_refusal_for(&RoomSource::foreign(0)),
        BevelAdvisoryCode::SizeExceedsFaceWidth
    );

    let chains = vec![vec![10, 11], vec![20, 21]];
    assert_eq!(the_vertex_to_blame(&chains, &RoomSource::foreign(1)), 20);
    assert_eq!(the_vertex_to_blame(&chains, &RoomSource::nothing()), 10);
    assert_eq!(the_edge_key(5, 2), (2, 5));

    let budget = the_budget_for(100, &CancelToken::default(), Rc::new(|_| {}));
    assert!(budget.input_is_float_stored);
    assert_eq!(
        budget.limit,
        solidmesh::bevel::level_line::WorkBudget::default_limit(100)
    );

    let builder = cube_builder(None);
    assert_eq!(
        builder.the_progress_for(LevelLineWorkPhase::Room, 0),
        BevelProgress::plain(BevelProgressPhase::Building {
            phase: LevelLineWorkPhase::Room,
            loops: 0
        })
    );
    let mut make = builder.the_in_turn_budgets(2, &CancelToken::default());
    assert!(make(10).input_is_float_stored);
}

#[test]
fn rust_only_the_rows_a_level_line_pass_built_carry_their_loops() {
    let mut builder = cube_builder(None);
    let loops = builder.the_rim_loops_of(&[0, 1]);
    let mut results: Vec<Option<solidmesh::bevel::BevelGroupResult>> = vec![None, None];

    builder.write_the_rows_that_built(&loops[..1], &mut results);
    let row = results[0].as_ref().expect("group 0 built");
    assert!(!row.failed());
    assert_eq!(row.edge_count, 4);
    assert_eq!(row.loop_count, 1);
    assert!(row.warnings.is_empty());
    assert!(results[1].is_none());
    // No pass handed the kernel yet, so no refusal names an edge.
    assert!(builder.the_edges_the_refusal_names().is_empty());

    // A refused loop goes under a row that built, and never under one that failed.
    let failure = solidmesh::bevel::BevelLoopFailure {
        advisory: solidmesh::bevel::BevelAdvisory::new(
            solidmesh::bevel::BevelAdvisoryCode::SizeRemovesNothing,
            Vec::new(),
        ),
        largest_size_that_fits: Some(0.5),
        edge_count: 4,
    };
    let refused = indexmap::IndexMap::from([(0, vec![failure.clone()])]);
    BevelSurfacePartBuilder::write_what_the_loops_came_to(&mut results, &refused);
    assert_eq!(
        results[0]
            .as_ref()
            .and_then(|row| row.loop_failures.clone()),
        Some(vec![failure])
    );
}
