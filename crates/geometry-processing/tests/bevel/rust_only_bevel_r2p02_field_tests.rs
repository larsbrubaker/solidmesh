//! Rust-only checks of the bevel core's R2.P02 chain, pick and room field readings, on the chain
//! fields of `chain_field_fixtures.rs` (assembled as `ChainField.FromPolyline`, R2.P05, will build
//! an open, unmitred, unclipped chain). The rail and fill readings are in
//! `rust_only_bevel_r2p02_fill_tests.rs`.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{
    fillet_arc, self_room, swallowed_side, ChainField, FieldKind, FieldSample, LevelLineMesh,
    PickField, RoomField, ScalarField, VertexFieldReading,
};
use vector_math::Vector3;

use crate::chain_field_fixtures::{l_chain, open_chain};

#[test]
fn rust_only_a_chain_parameter_locates_a_point_and_a_setback() {
    let chain = l_chain();
    assert_eq!(chain.locate_on_chain(-1.0), (0, 0.0));
    assert_eq!(chain.locate_on_chain(2.0), (0, 0.5));
    assert_eq!(chain.locate_on_chain(5.5), (1, 0.5));
    assert_eq!(chain.locate_on_chain(9.0), (1, 1.0));
    assert_eq!(chain.point_at(5.5), Vector3::new(4.0, 1.5, 0.0));
    assert_eq!(chain.setback_at(2.0), 1.5);
    assert_eq!(chain.setback_at(5.5), 1.5);
    assert!(chain.sharp_joints().is_empty());
    assert_eq!(chain.wrapped(-2.0), 0.0);
    assert_eq!(chain.wrapped(9.0), 7.0);
    assert!(!chain.has_smooth_joints());
    assert!(!chain.lies_in_an_end_plane(Vector3::ZERO, Vector3::UNIT_X));
    // No ray: the point past an end is clamped to it.
    assert_eq!(chain.point_on_the_ray(-3.0), (0.0, Vector3::ZERO));
    let rayed = ChainField {
        end_rays: (true, true),
        ..l_chain()
    };
    assert_eq!(
        rayed.point_on_the_ray(-3.0),
        (-3.0, Vector3::new(-3.0, 0.0, 0.0))
    );
    assert_eq!(
        rayed.point_on_the_ray(8.0),
        (8.0, Vector3::new(4.0, 4.0, 0.0))
    );
}

#[test]
fn rust_only_a_chain_field_reads_the_nearest_point_less_its_setback() {
    let chain = l_chain();
    let sample = chain
        .evaluate(Vector3::new(2.0, 5.0, 0.0))
        .expect("no budget in force");
    assert_eq!(sample.nearest, Vector3::new(4.0, 3.0, 0.0));
    assert!((sample.value - (8f64.sqrt() - 1.0)).abs() <= 1e-12);
    assert!((sample.parameter - 7.0).abs() <= 1e-12);
    assert_eq!(chain.kind(), FieldKind::ChainField);
    assert!(chain.as_chain_field().is_some());
    let at = Vector3::new(2.0, -3.0, 0.0);
    // The reading is the least of |q - c(u)| - s(u) along the chain, not the nearest point's own:
    // on the x leg, whose setback climbs 1 to 2, it stands where the slope of the distance is a
    // quarter, (2 + sqrt(0.6), 0).
    let value = chain.value_at(at).expect("no budget in force");
    let expected = 9.6f64.sqrt() - (1.0 + (2.0 + 0.6f64.sqrt()) / 4.0);
    assert!((value - expected).abs() <= 1e-12);
    assert_eq!(chain.value_across_the_known_cut(at).ok(), Some(value));
    // No bend threshold: C#'s Math.Cos(+infinity) is NaN, as Rust's.
    assert!(fillet_arc::sharp_cosine(&chain).is_nan());
    assert_eq!(swallowed_side::tolerance(&chain), 2e-5);
}

#[test]
fn rust_only_a_chain_fields_bounded_maxima_and_cell_bound() {
    let chain = l_chain();
    // Along the line y = -2 from x = 0 to 4 the setback climbs toward the corner, so the greatest
    // reading is at x = 0: 7.5 / sqrt(15) - 1.
    let along = chain
        .max_along_segment(
            Vector3::new(0.0, -2.0, 0.0),
            Vector3::new(4.0, -2.0, 0.0),
            1e-6,
        )
        .expect("no budget in force");
    assert!((along.value - (7.5 / 15f64.sqrt() - 1.0)).abs() <= 1e-6);
    let floor = chain
        .max_along_segment_at_least(
            Vector3::new(0.0, -2.0, 0.0),
            Vector3::new(4.0, -2.0, 0.0),
            1e-6,
            9.0,
        )
        .expect("no budget in force");
    assert_eq!(floor.value, 9.0);
    let over = chain
        .max_over_triangle(
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(0.0, -3.0, 0.0),
            1e-6,
        )
        .expect("no budget in force");
    // At the far corner (0, -3): 11.25 / sqrt(15) - 1.
    assert!((over.value - (11.25 / 15f64.sqrt() - 1.0)).abs() <= 1e-6);
    // The island search agrees with the plain one where it says yes.
    let island = chain
        .island_peak(
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(0.0, -3.0, 0.0),
            1e-3,
            1.5,
        )
        .expect("no budget in force");
    assert!(island.value > 1.5);
    let sample = |parameter: f64| FieldSample {
        parameter,
        chain: 0,
        ..FieldSample::default()
    };
    let bound = chain
        .segment_cell_bound(
            0,
            &sample(1.0),
            &sample(1.0),
            &sample(5.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(2.0, -1.0, 0.0),
            Vector3::new(1.0, -2.0, 0.0),
        )
        .expect("no budget in force");
    // Segment 0 alone, greatest at the corner (1, -2): 7.5 / sqrt(15) - 1.25.
    assert!((bound - (7.5 / 15f64.sqrt() - 1.25)).abs() <= 1e-12);
    // A sample of another chain is not read.
    let other = FieldSample {
        chain: 1,
        ..sample(1.0)
    };
    assert_eq!(
        chain
            .segment_cell_bound(
                0,
                &other,
                &other,
                &other,
                Vector3::ZERO,
                Vector3::UNIT_X,
                Vector3::UNIT_Y
            )
            .ok(),
        Some(f64::INFINITY)
    );
    // Going round the corner from 3.5 to 4.5 cuts across the turn by far less than the setback.
    assert!(!chain.spoke_gives_way_across_a_turn(3.0, 3.5, 4.5, 5.0, true));
}

#[test]
fn rust_only_a_pick_and_a_room_read_their_members() {
    let x_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let y_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 5.0, 0.0), Vector3::new(4.0, 5.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let pick =
        Rc::new(PickField::new(vec![x_leg.clone(), y_leg.clone()], &[0, 1]).expect("two members"));
    let near_y = pick
        .evaluate(Vector3::new(2.0, 4.0, 0.0))
        .expect("no budget in force");
    assert_eq!(near_y.chain, 1);
    assert!((near_y.value - 0.0).abs() <= 1e-12);
    // Half way between, the tie goes to the lower member.
    let middle = pick
        .evaluate(Vector3::new(2.0, 2.5, 0.0))
        .expect("no budget in force");
    assert_eq!(middle.chain, 0);
    assert_eq!(pick.kind(), FieldKind::PickField);
    assert_eq!(pick.value_at(Vector3::new(2.0, 4.0, 0.0)).ok(), Some(0.0));
    assert_eq!(pick.setback_of(&middle), 1.0);
    assert_eq!(
        pick.value_across_the_known_cut(Vector3::new(2.0, 4.0, 0.0))
            .ok(),
        Some(0.0)
    );
    let peak = pick
        .max_along_segment(
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(2.0, 5.0, 0.0),
            1e-6,
        )
        .expect("no budget in force");
    assert!((peak.value - 1.5).abs() <= 1e-6);

    let room = RoomField::from_pick(pick);
    assert_eq!(
        room.evaluate(Vector3::new(2.0, 4.0, 0.0))
            .ok()
            .map(|s| s.chain),
        Some(1)
    );
    assert_eq!(room.setback_of(&middle), 1.0);
    let one = RoomField::from_chain(x_leg);
    let mut samples = vec![None; 2];
    let read = self_room::sample_at(&one, &mut samples, 1, Vector3::new(2.0, 3.0, 0.0))
        .expect("no budget");
    assert!((read.value - 2.0).abs() <= 1e-12);
    // Cached: the vertex reads the first answer whatever position it is asked at again.
    let again = self_room::sample_at(&one, &mut samples, 1, Vector3::new(2.0, 9.0, 0.0))
        .expect("no budget");
    assert_eq!(again, read);
    let tri = one
        .max_over_triangle(
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(4.0, 1.0, 0.0),
            Vector3::new(2.0, 3.0, 0.0),
            1e-6,
            f64::INFINITY,
            f64::NEG_INFINITY,
        )
        .expect("no budget in force");
    assert!((tri.value - 2.0).abs() <= 1e-6);

    let mesh = Rc::new(std::cell::RefCell::new(LevelLineMesh::new()));
    let field: Rc<dyn ScalarField> = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let reading = VertexFieldReading::new(mesh, field, 0.0).expect("a length");
    assert_eq!(
        reading.value_at_point(Vector3::new(2.0, 3.0, 0.0)).ok(),
        Some(2.0)
    );
    assert_eq!(reading.pick().kind(), FieldKind::ChainField);
}

#[test]
fn rust_only_a_group_with_run_outs_reads_the_least_and_marks_it_clipped() {
    use geometry_processing::bevel::level_line::{
        corner_cap_flat_ends, BandGroupFieldsWithRunOuts, ChainDistanceField, PointField,
    };
    let chain = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    // A run-out ball at (2, 4) of radius 1.
    let run_out: Rc<dyn ChainDistanceField> =
        Rc::new(PointField::sphere(Vector3::new(2.0, 4.0, 0.0), 1.0));
    let pick_chain = chain.clone();
    let wrapped = Rc::new(BandGroupFieldsWithRunOuts::new(
        Rc::new(PointField::sphere(Vector3::new(9.0, 9.0, 9.0), 1.0)),
        Rc::new(move |p| pick_chain.evaluate(p)),
        None,
        vec![run_out],
    ));
    // Near the chain the pick's own reading stands.
    let near_chain = wrapped
        .evaluate(Vector3::new(2.0, 1.0, 0.0))
        .expect("no budget");
    assert_eq!((near_chain.value, near_chain.clipped), (0.0, false));
    // Near the ball the run-out wins, keeping the pick's chain point and marked clipped.
    let near_ball = wrapped
        .evaluate(Vector3::new(2.0, 3.5, 0.0))
        .expect("no budget");
    assert!((near_ball.value - -0.5).abs() <= 1e-12 && near_ball.clipped);
    assert_eq!(near_ball.nearest, Vector3::new(2.0, 0.0, 0.0));
    assert_eq!(
        wrapped.value_at(Vector3::new(2.0, 3.5, 0.0)).ok(),
        Some(-0.5)
    );
    assert_eq!(
        wrapped.run_out_value_at(Vector3::new(2.0, 1.0, 0.0)).ok(),
        Some(2.0)
    );
    assert_eq!(wrapped.kind(), FieldKind::WithRunOuts);
    // Between the chain and the ball the wrapped field peaks where the two meet, y = 2.
    let peak = wrapped
        .max_along_segment(
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(2.0, 4.0, 0.0),
            1e-6,
        )
        .expect("no budget");
    assert!((peak.value - 1.0).abs() <= 1e-6);
    let over = wrapped
        .max_over_triangle(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(3.0, 0.0, 0.0),
            Vector3::new(2.0, 4.0, 0.0),
            1e-6,
        )
        .expect("no budget");
    // Over the triangle the ridge between them (y = 2 + (x - 2)^2 / 8) rises away from x = 2, so
    // the greatest is where it meets the side from (1, 0) to (2, 4): t = 17 - sqrt(272) along it.
    let t = 17.0 - 272f64.sqrt();
    assert!((over.value - (4.0 * t - 1.0)).abs() <= 1e-5);

    // A vertex reading over the wrapper reads across the known cut: the least of the pick's
    // value across it and the run-outs'.
    let field: Rc<dyn ScalarField> = wrapped;
    let mesh = Rc::new(std::cell::RefCell::new(LevelLineMesh::new()));
    let reading = VertexFieldReading::new(mesh, field, 0.0).expect("a length");
    corner_cap_flat_ends::set_use_flat_ends_override(Some(true));
    let across = reading.known_cut_value_at(Vector3::new(2.0, 3.5, 0.0));
    let in_plane =
        reading.value_in_the_end_plane(Vector3::new(2.0, 1.0, 0.0), Vector3::ZERO, Vector3::UNIT_X);
    corner_cap_flat_ends::set_use_flat_ends_override(Some(false));
    let plain = reading.known_cut_value_at(Vector3::new(2.0, 1.0, 0.0));
    corner_cap_flat_ends::set_use_flat_ends_override(None);
    // The wrapper's inner pick here is a point field, neither a chain's nor a pick's: the plain
    // field answers.
    assert_eq!(across.ok(), Some(-0.5));
    assert_eq!(in_plane.ok(), Some(0.0));
    assert_eq!(plain.ok(), Some(0.0));
}

#[test]
fn rust_only_a_vertex_reading_of_a_chain_reads_across_its_known_cut() {
    use geometry_processing::bevel::level_line::corner_cap_flat_ends;
    let chain = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let field: Rc<dyn ScalarField> = chain;
    let mesh = Rc::new(std::cell::RefCell::new(LevelLineMesh::new()));
    let reading = VertexFieldReading::new(mesh, field, 0.0).expect("a length");
    corner_cap_flat_ends::set_use_flat_ends_override(Some(true));
    let across = reading.known_cut_value_at(Vector3::new(2.0, 3.0, 0.0));
    let in_plane =
        reading.value_in_the_end_plane(Vector3::new(2.0, 3.0, 0.0), Vector3::ZERO, Vector3::UNIT_X);
    corner_cap_flat_ends::set_use_flat_ends_override(None);
    // Unclipped, the chain's value across the cut and in its plane is its own: 3 less 1.
    assert_eq!(across.ok(), Some(2.0));
    assert_eq!(in_plane.ok(), Some(2.0));
}
