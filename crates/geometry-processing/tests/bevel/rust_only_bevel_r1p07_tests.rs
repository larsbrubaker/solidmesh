//! Rust-only checks of the bevel core's R1.P07 package (the facet cut, the room crossing, the end
//! section readings and the run-out's column ends). The tests that read them in context (the cut
//! ruler, the end gaining the section reading, the run-out column top) need the whole build; these
//! pin the pieces alone.

use geometry_processing::bevel::surface::surface_end_gains_the_section::{area_of, kept_by};
use geometry_processing::bevel::surface::surface_run_out_column_ends::{
    try_first_row_standing, try_the_slot_the_top_takes, with_the_crossings_stationed,
};
use geometry_processing::bevel::surface::{SurfaceBandEnd, SurfaceFacetCut, SurfaceLine};
use vector_math::Vector3;

#[test]
fn rust_only_a_facet_cut_removes_only_past_its_line_and_inside_its_ends() {
    let along = Vector3::new(1.0, 0.0, 0.0);
    let inward = Vector3::new(0.0, 1.0, 0.0);
    let line = SurfaceLine::new(Vector3::new(0.0, 0.0, 0.0), along);

    let cut = SurfaceFacetCut::with_ends(
        3,
        true,
        line,
        inward,
        7,
        vec![SurfaceBandEnd::new(Vector3::new(0.0, 0.0, 0.0), along)],
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        1,
        None,
    );

    // Behind the line (y < 0) and inside the end (x > 0) is removed; past the end it is not.
    assert!(cut.removes(Vector3::new(5.0, -1.0, 0.0), 1e-9));
    assert!(!cut.removes(Vector3::new(5.0, 1.0, 0.0), 1e-9));
    assert!(!cut.removes(Vector3::new(-5.0, -1.0, 0.0), 1e-9));

    // The deepest removed point of a segment crossing the end is clipped at the end.
    let removed = cut
        .try_find_removed_point(
            Vector3::new(-5.0, -1.0, 0.0),
            Vector3::new(5.0, -3.0, 0.0),
            1e-9,
        )
        .unwrap();
    assert!((removed - Vector3::new(5.0, -3.0, 0.0)).length() < 1e-12);
    assert!(cut.stands_on_the_chain_edge(Vector3::new(4.0, 0.0, 0.0), 1e-9));

    // Drawn along another rail it keeps its ends, edge and plan.
    let drawn = cut.drawn_along(SurfaceLine::new(Vector3::new(0.0, 1.0, 0.0), along), inward);
    assert_eq!(
        (drawn.edge_index, drawn.plan_index, drawn.ends.len()),
        (3, 1, 1)
    );
}

#[test]
fn rust_only_a_square_kept_by_a_half_plane_is_its_half() {
    let square = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(2.0, 2.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
    ];
    assert!((area_of(&square) - 4.0).abs() < 1e-12);

    let half = kept_by(
        &square,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        1e-9,
    );
    assert!((area_of(&half) - 2.0).abs() < 1e-12);
}

#[test]
fn rust_only_column_ends_station_the_crossings() {
    // Base stations past the first crossing give way to the crossings themselves.
    let stationed = with_the_crossings_stationed(
        &[0.1, 0.2, 0.3, 0.4],
        &[0.25, 0.35, f64::INFINITY],
        &[false, false, false],
        0.5,
        None,
    );
    assert_eq!(stationed, vec![0.1, 0.2, 0.25, 0.35, 0.5]);

    // The rows the plane has taken are the first few; a hole in the column is refused.
    assert_eq!(try_first_row_standing(&[0.1, 0.2, 0.9], 0.5), Some(2));
    assert_eq!(try_first_row_standing(&[0.9, 0.2], 0.5), None);

    // The top swallows the rows standing further out than it, and takes the last of them.
    assert_eq!(
        try_the_slot_the_top_takes(&[3.0, 2.0, 1.0], 0, 1.5, 1e-9),
        Some(1)
    );
    assert_eq!(try_the_slot_the_top_takes(&[1.0], 0, 1.5, 1e-9), None);
}

#[test]
fn rust_only_level_line_ceilings_aspects_and_refusals() {
    use geometry_processing::bevel::level_line::{band_gap, refinement_splits, BandGapRefusal};

    // The ceiling is the widest pair; a pick on one point names none.
    let points = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(3.0, 4.0, 0.0),
    ];
    assert_eq!(band_gap::chain_ceiling(&points), 5.0);
    assert_eq!(
        band_gap::chain_ceiling(&[points[0], points[0]]),
        f64::INFINITY
    );

    // A right isosceles triangle of legs 1: longest edge squared 2, twice its area 1.
    let aspect = refinement_splits::aspect(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    );
    assert!((aspect - 2.0).abs() < 1e-12);
    assert!(refinement_splits::is_new_sliver(30.0, 10.0, 20.0));
    assert!(!refinement_splits::is_new_sliver(30.0, 40.0, 20.0));

    let refused = band_gap::refused_by_size(1.5);
    assert_eq!((refused.refusal, refused.room), (BandGapRefusal::Size, 1.5));
}

#[test]
fn rust_only_a_pick_is_checked_and_its_touching_chains_found() {
    use geometry_processing::bevel::level_line::pick_validation::{
        check_pick_arguments, every_touching_pair, PickArgumentError,
    };

    let chains = vec![vec![0, 1, 2], vec![5, 6], vec![2, 7]];
    assert_eq!(
        check_pick_arguments(&[], &[], &chains, &[false, false, false]),
        Ok(())
    );
    assert_eq!(
        check_pick_arguments(&[], &[], &[], &[]),
        Err(PickArgumentError::NoChains)
    );
    assert_eq!(
        check_pick_arguments(&[], &[], &chains, &[false]),
        Err(PickArgumentError::ClosedFlagsDoNotMatch)
    );
    assert_eq!(
        check_pick_arguments(&[], &[], &[vec![0, 1]], &[true]),
        Err(PickArgumentError::ClosedChainTooShort)
    );

    // Chains 0 and 2 share vertex 2; chain 1 touches nobody.
    assert_eq!(every_touching_pair(&chains), vec![(0, 2, 2)]);
}

#[test]
fn rust_only_a_room_reading_says_what_bounded_it() {
    use geometry_processing::bevel::level_line::RoomReading;

    let unbounded = RoomReading {
        read: true,
        room: f64::INFINITY,
        ..RoomReading::default()
    };
    assert!(unbounded.unbounded() && unbounded.is_a_size_statement());

    let touching = RoomReading {
        read: true,
        chains_are_apart: false,
        ..RoomReading::default()
    };
    assert!(!touching.unbounded() && !touching.is_a_size_statement());
}

#[test]
fn rust_only_gathering_crossings_leaves_when_cancelled() {
    use geometry_processing::bevel::level_line::WorkError;
    use geometry_processing::bevel::surface::{SurfaceRoomCrossing, SurfaceRoomCut};
    use polygon_mesh::CancelToken;

    let cut = SurfaceRoomCut::new(
        1,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        1.0,
        1.0,
        f64::INFINITY,
    );
    let cuts = [cut, cut];

    let token = CancelToken::new();
    assert!(SurfaceRoomCrossing::gather(Some(&cuts), 1.0, &token)
        .unwrap()
        .is_empty());
    assert!(SurfaceRoomCrossing::gather(None, 1.0, &token)
        .unwrap()
        .is_empty());

    token.cancel();
    assert!(matches!(
        SurfaceRoomCrossing::gather(Some(&cuts), 1.0, &token),
        Err(WorkError::Cancelled)
    ));
}

#[test]
fn rust_only_the_gap_loop_walks_verdicts() {
    use geometry_processing::bevel::level_line::gap_loop_walk::{
        each_side_leaves_one_closed_loop, side_of_a_both_sided_loop,
        without_open_members_own_pieces,
    };
    use geometry_processing::bevel::level_line::GapLoop;

    // Mostly one side reads that side; a quarter or more on the other is the group's call.
    assert_eq!(side_of_a_both_sided_loop(&[1, 1, 1, 1, 1, 2], 2), 1);
    assert_eq!(side_of_a_both_sided_loop(&[1, 1, 2, 2], 1), 1);
    assert_eq!(side_of_a_both_sided_loop(&[1, 1, 2, 2], 2), 2);

    let closed_on = |side: i32| GapLoop::new(side, true, Vec::new(), Vec::new(), 0, false, None);
    assert!(each_side_leaves_one_closed_loop(&[
        closed_on(1),
        closed_on(2)
    ]));
    assert!(!each_side_leaves_one_closed_loop(&[closed_on(1)]));

    // Two open pieces of one open member and nothing else: its one-chain build keeps the first.
    let piece = |vertex: i32| GapLoop::new(1, false, vec![vertex], vec![0.0], 0, false, None);
    let kept = without_open_members_own_pieces(vec![piece(1), piece(2)], &[0], &[false], &[]);
    assert_eq!(kept.len(), 1);
    assert_eq!(kept[0].vertices, vec![1]);
}

#[test]
fn rust_only_the_end_face_field_switch_travels_with_the_build() {
    use geometry_processing::bevel::level_line::{LevelLineWork, PairCornerEndFaceField};

    // With no build in force there is nothing to switch, and the field reads on.
    assert!(PairCornerEndFaceField::on());
    {
        let _off = PairCornerEndFaceField::turn_off();
        assert!(PairCornerEndFaceField::on());
    }

    // Inside a build the switch is its budget's: off while the scope lives, back after.
    let budget = crate::rust_only_bevel_r1p05_tests::budget_for_tests();
    let _work = LevelLineWork::begin(Some(budget.clone()));
    assert!(PairCornerEndFaceField::on());
    {
        let _off = PairCornerEndFaceField::turn_off();
        assert!(!PairCornerEndFaceField::on());
        assert!(budget.pair_corner_end_face_field_off.get());
    }
    assert!(PairCornerEndFaceField::on());
}
