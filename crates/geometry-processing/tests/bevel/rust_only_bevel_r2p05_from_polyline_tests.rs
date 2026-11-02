//! Rust-only checks of R2.P05's `ChainField::from_polyline` family: the field it builds for an
//! open plain chain is the one R2.P02's tests hand-assemble, and the readings R2.P02 could only
//! check on that one are checked here on mitred, clipped, rayed and closed fields it builds.

use std::cell::OnceCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::chain_field_from_polyline::ChainFieldArgumentError;
use geometry_processing::bevel::level_line::{
    corner_fill_free_ends, fillet_arc, segment_box_tree, swallowed_side, ChainField,
    CornerFillPiece, EndRayBound, PickField, RoomField,
};
use vector_math::Vector3;

fn l_points() -> Vec<Vector3> {
    vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(4.0, 3.0, 0.0),
    ]
}

/// R2.P02's hand assembly of an open chain with no sharp turn, ray or clip.
fn hand_assembled(points: Vec<Vector3>, setbacks: Vec<f64>) -> ChainField {
    let mut parameter_at_point = vec![0.0];
    for i in 1..points.len() {
        let previous = parameter_at_point[i - 1];
        parameter_at_point.push(previous + (points[i] - points[i - 1]).length());
    }

    let segment_count = points.len() as i32 - 1;
    let nodes = segment_box_tree::build(&points, &setbacks, segment_count, None);
    let length = parameter_at_point[points.len() - 1];
    let min_setback = setbacks.iter().copied().fold(f64::INFINITY, f64::min);
    let max_setback = setbacks.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    ChainField {
        points,
        setbacks,
        parameter_at_point,
        nodes,
        smooth_joints: None,
        sharp_turn_degrees: f64::INFINITY,
        end_rays: (false, false),
        end_ray_bounds: (None, None),
        is_closed: false,
        length,
        min_setback,
        max_setback,
        extensions: None,
        end_clips: None,
        end_plane_reader: OnceCell::new(),
        sharp_joints: OnceCell::new(),
        ball: None,
    }
}

#[test]
fn rust_only_from_polyline_builds_the_hand_assembled_open_field() {
    let built = ChainField::from_polyline(&l_points(), false, &[1.0, 2.0, 1.0]).unwrap();
    let hand = hand_assembled(l_points(), vec![1.0, 2.0, 1.0]);

    assert_eq!(built.points, hand.points);
    assert_eq!(built.setbacks, hand.setbacks);
    assert_eq!(built.parameter_at_point, hand.parameter_at_point);
    assert_eq!(built.nodes, hand.nodes);
    assert_eq!(built.length, 7.0);
    assert_eq!((built.min_setback, built.max_setback), (1.0, 2.0));
    assert!(built.extensions.is_none() && built.end_clips.is_none());
    assert!(!built.has_smooth_joints());

    for at in [
        Vector3::new(2.0, 5.0, 0.0),
        Vector3::new(2.0, -3.0, 0.0),
        Vector3::new(-1.0, 1.0, 2.0),
    ] {
        assert_eq!(built.evaluate(at).unwrap(), hand.evaluate(at).unwrap());
        assert_eq!(
            built.value_across_the_known_cut(at).unwrap(),
            hand.value_across_the_known_cut(at).unwrap()
        );
    }

    assert_eq!(built.point_on_the_ray(-3.0), hand.point_on_the_ray(-3.0));
    assert!(fillet_arc::sharp_cosine(&built).is_nan());
    assert_eq!(
        swallowed_side::tolerance(&built),
        swallowed_side::tolerance(&hand)
    );
}

#[test]
fn rust_only_from_polyline_refuses_bad_arguments() {
    let one = [Vector3::new(0.0, 0.0, 0.0)];
    assert_eq!(
        ChainField::from_polyline(&one, false, &[1.0]).err(),
        Some(ChainFieldArgumentError::TooFewPoints)
    );
    assert_eq!(
        ChainField::from_polyline(&l_points(), false, &[1.0, 1.0]).err(),
        Some(ChainFieldArgumentError::SetbackCountMismatch)
    );
    let still = [Vector3::new(1.0, 1.0, 1.0); 2];
    assert_eq!(
        ChainField::from_polyline(&still, false, &[1.0, 1.0]).err(),
        Some(ChainFieldArgumentError::ZeroLength)
    );
    assert_eq!(
        ChainFieldArgumentError::ZeroLength.to_string(),
        "A chain of zero length has no field."
    );
}

#[test]
fn rust_only_a_closed_field_wraps_and_drops_its_end_rays() {
    let square = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(2.0, 2.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
    ];
    let closed = ChainField::from_polyline_bounded(
        &square,
        true,
        &[0.5; 4],
        f64::INFINITY,
        (true, true),
        (Some(EndRayBound::default()), None),
    )
    .unwrap();

    assert_eq!(closed.length, 8.0, "the closing segment counts");
    assert_eq!(closed.parameter_at_point.len(), 5);
    assert_eq!(closed.end_rays, (false, false));
    assert_eq!(closed.end_ray_bounds, (None, None));
    assert!(closed.end_clips.is_none());
    assert_eq!(closed.wrapped(-1.0), 7.0);
    assert_eq!(closed.wrapped(10.0), 2.0);
    assert_eq!(closed.wrapped(3.0), 3.0);
}

#[test]
fn rust_only_a_mitred_field_has_a_finite_sharp_cosine_and_rays() {
    let mitred =
        ChainField::from_polyline_sharp(&l_points(), false, &[1.0, 1.0, 1.0], 45.0).unwrap();

    assert!(
        mitred.extensions.is_some(),
        "the L's right angle turns by more than 45 degrees"
    );
    assert!((fillet_arc::sharp_cosine(&mitred) - 0.5f64.sqrt()).abs() < 1e-12);

    let gentle =
        ChainField::from_polyline_sharp(&l_points(), false, &[1.0, 1.0, 1.0], 120.0).unwrap();
    assert!(
        gentle.extensions.is_none(),
        "no joint turns by more than 120 degrees"
    );
    assert!((fillet_arc::sharp_cosine(&gentle) + 0.5).abs() < 1e-12);
}

#[test]
fn rust_only_a_clipped_knife_end_reads_its_end_plane() {
    let knife = EndRayBound {
        reach_per_setback: 1.0,
        plane_normal: Vector3::new(-2.0, 0.0, 0.0),
        same_sense: false,
        ..Default::default()
    };
    let clipped = ChainField::from_polyline_bounded(
        &l_points(),
        false,
        &[1.0, 1.0, 1.0],
        f64::INFINITY,
        (true, false),
        (Some(knife), None),
    )
    .unwrap();

    let clips = clipped
        .end_clips
        .as_ref()
        .expect("a bounded end is clipped");
    assert_eq!(clips.len(), 1);
    assert_eq!(clips[0].normal, Vector3::new(-1.0, 0.0, 0.0));
    assert_eq!(clips[0].parameter, 0.0);

    // Both ends of the segment in the plane x = 0 through the first point: in the end plane.
    assert!(clipped.lies_in_an_end_plane(Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, -2.0, 5.0)));
    assert!(!clipped.lies_in_an_end_plane(Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 0.0, 0.0)));

    // A same-sense bound is not a knife: nothing lies in its plane.
    let same_sense = ChainField::from_polyline_bounded(
        &l_points(),
        false,
        &[1.0, 1.0, 1.0],
        f64::INFINITY,
        (true, false),
        (
            Some(EndRayBound {
                same_sense: true,
                ..knife
            }),
            None,
        ),
    )
    .unwrap();
    assert!(
        !same_sense.lies_in_an_end_plane(Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, -2.0, 5.0))
    );

    // The ray runs on past the start, and the clip's reading is a reading.
    let (parameter, point) = clipped.point_on_the_ray(-0.5);
    assert_eq!(parameter, -0.5);
    assert!((point - Vector3::new(-0.5, 0.0, 0.0)).length() < 1e-12);
    assert!(clipped
        .value_across_the_known_cut(Vector3::new(2.0, 1.0, 0.0))
        .is_ok());
}

#[test]
fn rust_only_smooth_joints_in_scope_are_read_when_the_field_is_built() {
    let field = {
        let _scope = ChainField::smooth_joints_while([Vector3::new(4.0, 0.0, 0.0)]);
        ChainField::from_polyline_sharp(&l_points(), false, &[1.0, 1.0, 1.0], 45.0).unwrap()
    };

    assert!(field.has_smooth_joints());
    assert!(field.is_smooth_joint(1));
    assert!(!field.is_smooth_joint(0));
    assert!(
        field.extensions.is_none(),
        "the only sharp joint is a smooth one, so nothing is mitred"
    );

    let outside = ChainField::from_polyline(&l_points(), false, &[1.0, 1.0, 1.0]).unwrap();
    assert!(!outside.has_smooth_joints());
}

#[test]
fn rust_only_tolerance_and_end_arc_read_the_built_field() {
    let field = ChainField::from_polyline(&l_points(), false, &[1.0, 3.0, 1.0]).unwrap();
    assert!((swallowed_side::tolerance(&field) - 3e-5).abs() < 1e-18);

    let mut piece = CornerFillPiece {
        owner: 0,
        vertices: vec![10, 11, 12, 13],
        parameters: vec![0.5, 0.0, 0.0, 2.0],
    };
    assert!(corner_fill_free_ends::start_inside_an_end_arc(
        &field, &mut piece
    ));
    assert_eq!(piece.vertices, vec![12, 13, 10, 11]);
    assert_eq!(piece.parameters, vec![0.0, 2.0, 0.5, 0.0]);

    let mut at_the_far_end = CornerFillPiece {
        owner: 0,
        vertices: vec![1, 2, 3],
        parameters: vec![7.0, 3.0, 7.0],
    };
    assert!(corner_fill_free_ends::start_inside_an_end_arc(
        &field,
        &mut at_the_far_end
    ));
    assert_eq!(at_the_far_end.vertices, vec![1, 2, 3]);

    let mut nowhere = CornerFillPiece {
        owner: 0,
        vertices: vec![1, 2, 3],
        parameters: vec![1.0, 2.0, 3.0],
    };
    assert!(!corner_fill_free_ends::start_inside_an_end_arc(
        &field,
        &mut nowhere
    ));
}

#[test]
fn rust_only_scaled_setbacks_rebuild_the_same_chain() {
    let bounded = ChainField::from_polyline_bounded(
        &l_points(),
        false,
        &[1.0, 2.0, 1.0],
        45.0,
        (true, false),
        (Some(EndRayBound::default()), None),
    )
    .unwrap();
    let scaled = bounded.with_setbacks_scaled(2.0).unwrap();

    assert_eq!(scaled.setbacks, vec![2.0, 4.0, 2.0]);
    assert_eq!((scaled.min_setback, scaled.max_setback), (2.0, 4.0));
    assert_eq!(scaled.points, bounded.points);
    assert_eq!(scaled.sharp_turn_degrees, 45.0);
    assert_eq!(scaled.end_rays, (true, false));
    assert_eq!(scaled.end_ray_bounds, bounded.end_ray_bounds);

    let pick = PickField::new(
        vec![
            Rc::new(ChainField::from_polyline(&l_points(), false, &[1.0; 3]).unwrap()),
            Rc::new(ChainField::from_polyline(&l_points(), false, &[3.0; 3]).unwrap()),
        ],
        &[1],
    )
    .unwrap();
    let scaled_pick = pick.with_setbacks_scaled(0.5).unwrap();
    assert_eq!(
        scaled_pick.chains[0].setbacks,
        vec![1.0; 3],
        "not a member: unchanged"
    );
    assert_eq!(scaled_pick.chains[1].setbacks, vec![1.5; 3]);

    let room = RoomField::from_chain(Rc::new(bounded));
    let scaled_room = room.with_setbacks_scaled(3.0).unwrap();
    assert_eq!(scaled_room.one.unwrap().setbacks, vec![3.0, 6.0, 3.0]);
    let room = RoomField::from_pick(Rc::new(pick));
    assert_eq!(
        room.with_setbacks_scaled(2.0)
            .unwrap()
            .several
            .unwrap()
            .chains[1]
            .setbacks,
        vec![6.0; 3]
    );
}

#[test]
fn rust_only_the_exhaustive_reading_agrees_with_the_tree() {
    let fields = [
        ChainField::from_polyline(&l_points(), false, &[1.0, 2.0, 1.0]).unwrap(),
        ChainField::from_polyline_sharp(&l_points(), false, &[1.0, 1.0, 1.0], 45.0).unwrap(),
        ChainField::from_polyline_rayed(&l_points(), false, &[1.0, 1.0, 1.0], 45.0, (true, true))
            .unwrap(),
    ];

    for field in &fields {
        for at in [
            Vector3::new(2.0, 5.0, 0.0),
            Vector3::new(6.0, -1.0, 0.0),
            Vector3::new(-2.0, 0.5, 1.0),
            Vector3::new(4.0, 6.0, 0.0),
        ] {
            let tree = field.evaluate(at).unwrap();
            let exhaustive = field.evaluate_exhaustive(at);
            assert!((tree.value - exhaustive.value).abs() < 1e-12, "{at:?}");
            assert!(
                (tree.parameter - exhaustive.parameter).abs() < 1e-9,
                "{at:?}"
            );
        }
    }
}

#[test]
fn rust_only_a_parameter_finds_its_segment_and_tangent() {
    let open = ChainField::from_polyline(&l_points(), false, &[1.0; 3]).unwrap();
    assert_eq!(open.segment_holding(-1.0), (0, 0.0));
    assert_eq!(open.segment_holding(2.0), (0, 2.0));
    assert_eq!(open.segment_holding(5.0), (1, 1.0));
    assert_eq!(open.segment_holding(9.0), (1, 3.0));

    let square = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(2.0, 2.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
    ];
    let closed = ChainField::from_polyline(&square, true, &[0.5; 4]).unwrap();
    assert_eq!(closed.segment_holding(-1.0), (3, 1.0));
    assert_eq!(closed.segment_holding(8.0), (0, 0.0));

    // At a segment's middle the tangent is its own; at the joint, with no threshold, the two
    // directions blend by the half-lengths: 2 of the x leg's and 1.5 of the y leg's, so the share
    // of y is 2 / 3.5 - (3/7, 4/7), along (0.6, 0.8).
    let x = Vector3::new(1.0, 0.0, 0.0);
    assert!((fillet_arc::tangent_at(&open, 2.0) - x).length() < 1e-12);
    let at_the_joint = fillet_arc::tangent_at(&open, 4.0);
    assert!(
        (at_the_joint - Vector3::new(0.6, 0.8, 0.0)).length() < 1e-9,
        "{at_the_joint:?}"
    );
    let bisector = Vector3::new(1.0, 1.0, 0.0).get_normal();
    // Past the last segment of an open chain there is no neighbour.
    assert!((fillet_arc::tangent_at(&open, 7.0) - Vector3::new(0.0, 1.0, 0.0)).length() < 1e-12);

    // A sharp joint keeps each segment's own direction off the vertex, the bisector on it.
    let sharp = ChainField::from_polyline_sharp(&l_points(), false, &[1.0; 3], 45.0).unwrap();
    assert!((fillet_arc::tangent_at(&sharp, 3.9) - x).length() < 1e-12);
    assert!((fillet_arc::tangent_at(&sharp, 4.0) - bisector).length() < 1e-9);
}

#[test]
fn rust_only_a_cell_bound_is_infinite_on_a_clipped_chain_and_sound_on_a_rayed_one() {
    let corners = [
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(2.0, -1.0, 0.0),
        Vector3::new(1.0, -2.0, 0.0),
    ];

    let rayed =
        ChainField::from_polyline_rayed(&l_points(), false, &[1.0, 1.0, 1.0], 45.0, (true, true))
            .unwrap();
    let samples: Vec<_> = corners
        .iter()
        .map(|&c| rayed.evaluate(c).unwrap())
        .collect();
    let bound = rayed
        .segment_cell_bound(
            0,
            &samples[0],
            &samples[1],
            &samples[2],
            corners[0],
            corners[1],
            corners[2],
        )
        .unwrap();
    assert!(bound.is_finite());
    for (corner, sample) in corners.iter().zip(&samples) {
        assert!(
            bound >= sample.value - 1e-12,
            "{corner:?}: {bound} < {}",
            sample.value
        );
    }
    // The same corners on the plain chain bound the same way: the rays only lower the field.
    let plain = ChainField::from_polyline(&l_points(), false, &[1.0, 1.0, 1.0]).unwrap();
    assert_eq!(
        plain
            .segment_cell_bound(
                0,
                &samples[0],
                &samples[1],
                &samples[2],
                corners[0],
                corners[1],
                corners[2]
            )
            .unwrap(),
        bound
    );

    let clipped = ChainField::from_polyline_bounded(
        &l_points(),
        false,
        &[1.0, 1.0, 1.0],
        f64::INFINITY,
        (true, false),
        (Some(EndRayBound::default()), None),
    )
    .unwrap();
    assert_eq!(
        clipped
            .segment_cell_bound(
                0,
                &samples[0],
                &samples[1],
                &samples[2],
                corners[0],
                corners[1],
                corners[2]
            )
            .unwrap(),
        f64::INFINITY,
        "an end clip raises the field, so a clipped chain gives no bound"
    );
}
