//! Rust-only checks of the bevel core's R2.P02 chain, pick and room field readings, on a field
//! assembled as `ChainField.FromPolyline` (R2.P05) will assemble it for an open chain with no sharp
//! turn: points, setbacks, the parameter at each point, the segment box tree and nothing clipped.

use std::cell::OnceCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::{
    fillet_arc, segment_box_tree, self_room, swallowed_side, ChainField, FieldKind, FieldSample,
    LevelLineMesh, PickField, RoomField, ScalarField, VertexFieldReading,
};
use vector_math::Vector3;

/// An open chain through `points` asking `setbacks`, no sharp turn, no end ray, no clip.
fn open_chain(points: Vec<Vector3>, setbacks: Vec<f64>) -> ChainField {
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
    }
}

/// An L: (0,0,0) to (4,0,0) to (4,3,0), setback 1 then 2 then 1.
fn l_chain() -> ChainField {
    open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        ],
        vec![1.0, 2.0, 1.0],
    )
}

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

#[test]
fn rust_only_a_rail_is_recut_at_a_jump_nearest_the_other_start() {
    use geometry_processing::bevel::level_line::gap_fill::within_excursion;
    use geometry_processing::bevel::level_line::ZipRail;
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let rail = ZipRail {
        vertices: vec![0, 1, 2, 3],
        parameters: vec![0.0, 1.0, 6.0, 7.0],
        reversed: false,
    };
    // From 1 to 6 of chain parameter across one unit edge: a jump; the steps either side are not.
    assert!(ZipRail::is_a_jump(&mesh, &rail, 1, 8.0));
    assert!(!ZipRail::is_a_jump(&mesh, &rail, 0, 8.0));
    assert!(!ZipRail::is_a_jump(&mesh, &rail, 3, 8.0));
    let spans: Vec<(f64, f64)> = rail.parameters.iter().map(|&p| (p, p)).collect();
    let recut = ZipRail::start_nearest(&mesh, &rail, &spans, 6.0, 8.0, 0.5);
    assert_eq!(recut.vertices, vec![2, 3, 0, 1]);
    assert_eq!(recut.parameters, vec![-2.0, -1.0, 0.0, 1.0]);
    // Within the margin the rail is left as it was.
    assert_eq!(
        ZipRail::start_nearest(&mesh, &rail, &spans, 6.0, 8.0, 3.0),
        rail
    );

    let chain = l_chain();
    // At parameter 4 the setback is 2, so three setbacks past the slack is 6.
    assert!(within_excursion(4.0, -1.9, &chain, 0.1));
    assert!(!within_excursion(4.0, -2.2, &chain, 0.1));
}

#[test]
fn rust_only_a_sharp_joints_inside_corner_and_carry_weight() {
    use geometry_processing::bevel::level_line::sharp_joint_corners::the_inside_corner;
    let chain = ChainField {
        sharp_turn_degrees: 45.0,
        ..l_chain()
    };
    assert_eq!(chain.sharp_joints(), &[(4.0, 2.0)]);
    let (corner, plane) = the_inside_corner(&chain, 4.0, 1.0).expect("the L turns");
    assert!((corner - Vector3::new(3.0, 1.0, 0.0)).length() <= 1e-9);
    assert_eq!(plane, Vector3::new(0.0, 0.0, -1.0));
    assert!(the_inside_corner(&chain, 2.0, 1.0).is_none());

    let x = Vector3::new(1.0, 0.0, 0.0);
    // 3.5 from the joint of setback 2: smooth_step(0.75).
    assert!((fillet_arc::carry_weight(&chain, 0.0, 0.5, x, x) - 0.84375).abs() <= 1e-15);
    // Over the joint, none.
    assert_eq!(fillet_arc::carry_weight(&chain, 3.9, 4.1, x, x), 0.0);
    // A row turning by the whole threshold carries none; by half of it, all that the joint allows.
    let turned = Vector3::new(1.0, 1.0, 0.0).get_normal();
    assert!(fillet_arc::carry_weight(&chain, 0.0, 0.5, x, turned).abs() <= 1e-9);
    let half = Vector3::new(1.0, 22.5f64.to_radians().tan(), 0.0).get_normal();
    assert!((fillet_arc::carry_weight(&chain, 0.0, 0.5, x, half) - 0.84375).abs() <= 1e-9);
    // No joints: the gentle weight alone.
    assert_eq!(fillet_arc::carry_weight(&l_chain(), 0.0, 0.5, x, x), 1.0);
}

#[test]
fn rust_only_a_loop_reads_as_a_rail_both_ways_and_the_rails_zip() {
    use geometry_processing::bevel::level_line::gap_fill::{merge, senses, try_envelope};
    use geometry_processing::bevel::level_line::{GapLoop, ZipRail};
    let chain = l_chain();
    // Cut open after its wrap (3 -> 0) and already monotone.
    let (rail, cost, offending) = try_envelope(
        &[10, 11, 12, 13],
        &[2.0, 3.0, 0.0, 1.0],
        1,
        false,
        &chain,
        0.0,
    );
    assert_eq!(
        rail.as_ref().map(|r| r.vertices.clone()),
        Some(vec![12, 13, 10, 11])
    );
    assert_eq!((cost, offending), (0.0, -1));
    // A shallow excursion is flattened onto the envelope.
    let (rail, cost, _) = try_envelope(
        &[10, 11, 12, 13],
        &[0.0, 2.0, 1.5, 3.0],
        3,
        false,
        &chain,
        0.0,
    );
    assert_eq!(rail.map(|r| r.parameters), Some(vec![0.0, 2.0, 2.0, 3.0]));
    assert_eq!(cost, 0.5);
    // One deeper than three setbacks is refused, naming where.
    let (rail, _, offending) = try_envelope(
        &[10, 11, 12, 13],
        &[0.0, 10.0, 1.0, 11.0],
        3,
        false,
        &chain,
        0.0,
    );
    assert!(rail.is_none());
    assert_eq!(offending, 2);

    let gap_loop = GapLoop::new(
        1,
        true,
        vec![10, 11, 12, 13],
        vec![0.0, 1.0, 2.0, 3.0],
        0,
        false,
        None,
    );
    let both = senses(&gap_loop, &chain, 0.0);
    assert_eq!(
        both.forward.as_ref().map(|r| r.vertices.clone()),
        Some(vec![10, 11, 12, 13])
    );
    assert_eq!(both.forward_cost, 0.0);
    assert_eq!(
        both.backward.as_ref().map(|r| r.vertices.clone()),
        Some(vec![12, 11, 10, 13])
    );
    assert_eq!(both.backward_cost, 3.0);
    assert_eq!(both.offending, 10);

    // Two rails along the chain's straight leg, their vertices far apart so no step is a jump.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
    ];
    let left = ZipRail {
        vertices: vec![0, 1],
        parameters: vec![0.0, 2.0],
        reversed: false,
    };
    let right = ZipRail {
        vertices: vec![2, 3],
        parameters: vec![1.0, 3.0],
        reversed: true,
    };
    let steps = merge(&mesh, &chain, &left, &right, 4.0, 0.0).expect("no budget in force");
    assert_eq!(
        steps,
        vec![(0, 0, true), (1, 0, false), (1, 1, true), (2, 1, false)]
    );
}

#[test]
fn rust_only_open_rails_extend_past_the_ends_and_tie_vertices_span_the_jump() {
    use geometry_processing::bevel::level_line::open_gap_fill::{extended, try_rail};
    use geometry_processing::bevel::level_line::{open_gap_fill_stations, rail_stations};
    let chain = open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        ],
        vec![1.0, 1.0, 1.0],
    );
    // Past the start, by the angle about the end vertex times its setback.
    assert!(
        (extended(Vector3::new(-1.0, 0.0, 0.0), 0.0, &chain) + std::f64::consts::FRAC_PI_2).abs()
            <= 1e-12
    );
    assert_eq!(extended(Vector3::new(0.0, 1.0, 0.0), 0.0, &chain), 0.0);
    assert_eq!(extended(Vector3::new(2.0, 1.0, 0.0), 2.0, &chain), 2.0);

    // The inside corner (3, 1) of the L ties the two legs: read just off it toward (3.5, 2) the
    // chain reads the second leg, so the vertex stands for 3 up to 5.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![Vector3::new(3.0, 1.0, 0.0), Vector3::new(3.5, 2.0, 0.0)];
    let (read_from, read_to) = open_gap_fill_stations::on_the_edge(
        &chain,
        mesh.vertices[0],
        mesh.vertices[1],
        3.0,
        6.0,
        1e-9,
    )
    .expect("no budget");
    assert!((read_from - 5.0).abs() <= 1e-5);
    assert_eq!(read_to, 6.0);

    let rail = try_rail(&mesh, &chain, &[0, 1], &[3.0, 6.0], false, 0.0).expect("monotone");
    assert_eq!(rail.parameters, vec![3.0, 6.0]);
    let spans = open_gap_fill_stations::tie_spans(&mesh, &chain, &rail, 1e-9).expect("no budget");
    assert_eq!(spans[0].0, 3.0);
    assert!((spans[0].1 - 5.0).abs() <= 1e-5);
    assert_eq!(spans[1], (6.0, 6.0));
    // Read backward the rail is flattened within three setbacks, refused past them.
    let back = try_rail(&mesh, &chain, &[0, 1], &[3.0, 6.0], true, 0.0).expect("within reach");
    assert_eq!(back.parameters, vec![6.0, 6.0]);
    assert_eq!(
        try_rail(&mesh, &chain, &[0, 1], &[1.0, 6.0], true, 0.0).err(),
        Some(0)
    );

    let stations = rail_stations::tie_spans(&mesh, &chain, &[0, 1], &[3.0, 6.0], 1e-9, 0.0)
        .expect("no budget");
    assert!((stations[0].1 - 5.0).abs() <= 1e-4);
    assert_eq!(stations[1], (6.0, 6.0));
    let near = rail_stations::jump_near(
        &chain,
        mesh.vertices[0],
        mesh.vertices[1],
        3.0,
        6.0,
        1e-9,
        1,
        0.0,
    )
    .expect("no budget");
    assert!((near - 5.0).abs() <= 1e-4);
    assert_eq!(
        rail_stations::jump_near(
            &chain,
            mesh.vertices[1],
            mesh.vertices[0],
            6.0,
            3.0,
            1e-9,
            -1,
            0.0
        )
        .ok(),
        Some(6.0)
    );
}

#[test]
fn rust_only_two_chains_level_lines_meet_at_the_mitre_and_a_run_is_cut_back_to_the_rail() {
    use geometry_processing::bevel::level_line::corner_tie_points::try_mitre;
    use geometry_processing::bevel::level_line::virtual_rail_past_the_end::to_the_rail;
    use geometry_processing::bevel::level_line::GapLoop;
    let along_x = open_chain(
        vec![Vector3::new(-5.0, 0.0, 0.0), Vector3::new(5.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    );
    let along_y = open_chain(
        vec![Vector3::new(0.0, -5.0, 0.0), Vector3::new(0.0, 5.0, 0.0)],
        vec![1.0, 1.0],
    );
    let found = try_mitre(
        &along_x,
        &along_y,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
    )
    .expect("no budget");
    assert!(found.is_some_and(|p| (p - Vector3::new(1.0, 1.0, 0.0)).length() <= 1e-9));
    let outside = try_mitre(
        &along_x,
        &along_y,
        Vector3::new(2.0, 2.0, 0.0),
        Vector3::new(3.0, 2.0, 0.0),
        Vector3::new(2.0, 3.0, 0.0),
    )
    .expect("no budget");
    assert!(outside.is_none());

    // A side run along the rail y = 1 that then climbs the crease past it.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(2.0, 1.5, 0.0),
        Vector3::new(3.0, 2.0, 0.0),
    ];
    let run = GapLoop::new(
        1,
        false,
        vec![0, 1, 2, 3],
        vec![5.0, 6.0, 7.0, 8.0],
        0,
        false,
        None,
    );
    let cut = to_the_rail(&mesh, &along_x, &run, true, 1.0, false)
        .expect("no budget")
        .expect("the run climbs off the rail");
    assert_eq!((cut.keep_count, cut.at_tail), (2, true));
    assert_eq!(cut.away, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(cut.parameter_at_p, 6.0);
    // A run that stays on the rail needs no cut, unless it may end at P.
    let on_rail = GapLoop::new(1, false, vec![0, 1], vec![5.0, 6.0], 0, false, None);
    assert!(to_the_rail(&mesh, &along_x, &on_rail, true, 1.0, false)
        .expect("no budget")
        .is_none());
    assert!(to_the_rail(&mesh, &along_x, &on_rail, true, 1.0, true)
        .expect("no budget")
        .is_some());
}

#[test]
fn rust_only_a_boundary_step_follows_round_the_fan() {
    use geometry_processing::bevel::level_line::gap_loop_walk::{follows, one_of};
    use geometry_processing::bevel::level_line::LevelLineTriangle;
    // A unit square as two triangles sharing the diagonal 0-2.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
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

    assert_eq!(one_of(&mesh, 1, 2), Some(0));
    assert_eq!(one_of(&mesh, 2, 1), None);
    // From 1 -> 2 the boundary turns round vertex 2 across the diagonal to 2 -> 3.
    assert_eq!(follows(&mesh, (1, 2)), Some((2, 3)));
    assert_eq!(follows(&mesh, (3, 0)), Some((0, 1)));
    assert_eq!(follows(&mesh, (1, 0)), None);
}

#[test]
fn rust_only_a_corner_is_where_chains_end_together_and_ties_are_read_by_value() {
    use geometry_processing::bevel::level_line::corner_fill::{
        corner_points, fields_tie, takes, ties,
    };
    use geometry_processing::bevel::level_line::{
        BandGapRefusal, BandGapResult, BandProfile, CornerFillPiece,
    };
    let x_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let y_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 4.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let chains = vec![x_leg.clone(), y_leg.clone()];
    assert_eq!(corner_points(&chains), vec![Vector3::ZERO]);
    assert!(corner_points(&chains[..1]).is_empty());
    let gap = BandGapResult {
        chains: chains.clone(),
        ..BandGapResult::default()
    };
    assert!(takes(Some(&gap), BandProfile::Fillet));
    assert!(takes(Some(&gap), BandProfile::Chamfer));
    let refused = BandGapResult {
        refusal: BandGapRefusal::Size,
        ..gap
    };
    assert!(!takes(Some(&refused), BandProfile::Fillet));
    assert!(!takes(None, BandProfile::Fillet));

    assert!(fields_tie(&x_leg, &y_leg, Vector3::new(1.0, 1.0, 0.0)).expect("no budget"));
    assert!(!fields_tie(&x_leg, &y_leg, Vector3::new(1.0, 2.0, 0.0)).expect("no budget"));
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![Vector3::new(3.0, 3.0, 0.0), Vector3::new(1.0, 2.0, 0.0)];
    let neighbour = CornerFillPiece {
        owner: 1,
        vertices: vec![0, 1],
        parameters: vec![3.0, 2.0],
    };
    assert!(ties(&mesh, &chains, 0, &neighbour, true).expect("no budget"));
    assert!(!ties(&mesh, &chains, 0, &neighbour, false).expect("no budget"));
}

#[test]
fn rust_only_corner_pieces_are_trimmed_off_the_line_and_closed_round_the_seam() {
    use geometry_processing::bevel::level_line::corner_fill::close_round_the_seam;
    use geometry_processing::bevel::level_line::corner_fill_pieces::trim_off_the_line;
    use geometry_processing::bevel::level_line::CornerFillPiece;
    let x_leg = open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    );
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 3.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(2.0, 1.0, 0.0),
        Vector3::new(3.0, 0.5, 0.0),
        Vector3::new(4.0, 4.0, 0.0),
    ];
    let mut piece = CornerFillPiece {
        owner: 0,
        vertices: vec![0, 1, 2, 3, 4],
        parameters: vec![0.0, 1.0, 2.0, 3.0, 4.0],
    };
    trim_off_the_line(&mesh, &x_leg, &mut piece).expect("no budget");
    assert_eq!(piece.vertices, vec![1, 2, 3]);
    assert_eq!(piece.parameters, vec![1.0, 2.0, 3.0]);

    // A closed square chain of side 4, its seam at (0, 0), and a loop inside it.
    let points = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(4.0, 4.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
    ];
    let setbacks = vec![1.0; 4];
    let nodes = segment_box_tree::build(&points, &setbacks, 4, None);
    let square = ChainField {
        nodes,
        parameter_at_point: vec![0.0, 4.0, 8.0, 12.0, 16.0],
        is_closed: true,
        length: 16.0,
        ..open_chain(points.clone(), setbacks.clone())
    };
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(2.0, 1.0, 0.0),
        Vector3::new(3.0, 2.0, 0.0),
        Vector3::new(2.0, 3.0, 0.0),
        Vector3::new(1.0, 2.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
    ];
    let mut round = CornerFillPiece {
        owner: 0,
        vertices: vec![0, 1, 2, 3, 4],
        parameters: vec![2.0, 6.0, 10.0, 14.0, 1.0],
    };
    close_round_the_seam(&mesh, &square, &mut round);
    // It starts at the mitre vertex nearest the seam, read at 1, and ends there read across the
    // seam at 15.
    assert_eq!(round.vertices, vec![4, 0, 1, 2, 3, 4]);
    assert_eq!(round.parameters, vec![1.0, 2.0, 6.0, 10.0, 14.0, 15.0]);
}
