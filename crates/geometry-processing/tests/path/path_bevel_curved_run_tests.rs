//! Ports `PathBevelCurvedRunTests`: what a bevel does where the path is not a polygon - curves
//! flattened finely enough to stay smooth, corners beside a curved run, and the room two such
//! corners share along it.
//!
//! `ARadiusThatCannotFitIsFoundQuicklyOnALongRun` reads `path_bevel_arcs::run_places_measured`,
//! which is per thread (C# `[ThreadStatic]`), so it needs no `[NotInParallel]` here.

use agg_path::{FlagsAndCommand, VertexSource, VertexStorage};
use geometry_processing::path::path_bevel::{bevel, bevel_contour, cleaned_contours};
use geometry_processing::path::{path_bevel_arcs, PathBevelSizeMode};
use vector_math::{math_helper, Vector2};

use super::path_bevel_test_geometry::{
    angle_between_directions, arc_center, distance_to_polyline, inserted_arc, local_direction_at,
    position_along, self_intersects, sweep_about, tangent_distance,
};

const PI: f64 = std::f64::consts::PI;

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

fn bevel_plain(contour: &[Vector2], closed: bool, size: f64, segments: i32) -> Vec<Vector2> {
    bevel_contour(
        contour,
        closed,
        size,
        segments,
        10.0,
        None,
        PathBevelSizeMode::Radius,
        0,
    )
}

fn bevel_source(source: &VertexStorage, size: f64, segments: i32) -> VertexStorage {
    bevel(
        source,
        size,
        segments,
        10.0,
        None,
        PathBevelSizeMode::Radius,
    )
}

fn points_of(source: &dyn VertexSource) -> Vec<Vector2> {
    source
        .vertices()
        .into_iter()
        .filter(|v| v.command == FlagsAndCommand::MOVE_TO || v.is_line_to())
        .map(|v| v.position)
        .collect()
}

fn flattened_points(source: &VertexStorage) -> Vec<Vector2> {
    cleaned_contours(source).remove(0).points
}

#[test]
fn a_nan_vertex_costs_only_itself() {
    const SEGMENTS: usize = 4;
    const SIZE: f64 = 1.0;
    let square = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(f64::NAN, 10.0),
        Vector2::new(20.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];
    let is_nan = |p: &&Vector2| p.x.is_nan() || p.y.is_nan();
    let beveled_square = bevel_plain(&square, true, SIZE, SEGMENTS as i32);
    assert_eq!(beveled_square.len(), (2 * (SEGMENTS + 1)) + 3);
    assert_eq!(beveled_square.iter().filter(is_nan).count(), 1);

    const HEXAGON_RADIUS: f64 = 20.0;
    let mut hexagon: Vec<Vector2> = (0..6)
        .map(|corner| {
            let angle = corner as f64 * PI / 3.0;
            Vector2::new(HEXAGON_RADIUS * angle.cos(), HEXAGON_RADIUS * angle.sin())
        })
        .collect();
    hexagon[1] = Vector2::new(f64::NAN, 5.0);
    let beveled_hexagon = bevel_plain(&hexagon, true, SIZE, SEGMENTS as i32);
    assert_eq!(beveled_hexagon.len(), (3 * (SEGMENTS + 1)) + 3);
    assert_eq!(beveled_hexagon.iter().filter(is_nan).count(), 1);
    assert_eq!(beveled_hexagon[0], hexagon[0]);
    assert_eq!(beveled_hexagon[2], hexagon[2]);
    for arc_index in 0..3 {
        let start = 3 + (arc_index * (SEGMENTS + 1));
        let arc = &beveled_hexagon[start..start + SEGMENTS + 1];
        let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
        for point in arc {
            assert_near((*point - center).length(), SIZE, 1e-9);
        }
    }
}

const CURVE_RADIUS: f64 = 20.0;

fn quarter_circle() -> VertexStorage {
    let k = 4.0 / 3.0 * (PI / 4.0 / 4.0).tan();
    let mut source = VertexStorage::new();
    source.move_to(CURVE_RADIUS, 0.0);
    append_arc(&mut source, 0.0, PI / 4.0, k);
    append_arc(&mut source, PI / 4.0, PI / 2.0, k);
    source
}

fn append_arc(source: &mut VertexStorage, start_angle: f64, end_angle: f64, k: f64) {
    let start = Vector2::new(start_angle.cos(), start_angle.sin()) * CURVE_RADIUS;
    let end = Vector2::new(end_angle.cos(), end_angle.sin()) * CURVE_RADIUS;
    let start_tangent = Vector2::new(-start_angle.sin(), start_angle.cos()) * CURVE_RADIUS;
    let end_tangent = Vector2::new(-end_angle.sin(), end_angle.cos()) * CURVE_RADIUS;
    source.curve4_vector(start + (start_tangent * k), end - (end_tangent * k), end);
}

#[test]
fn a_curve_survives_the_bevel_as_a_curve() {
    let beveled = bevel_source(&quarter_circle(), 0.5, 4);
    let points = points_of(&beveled);
    assert!(points.len() > 2);
    for i in 1..points.len() {
        let mid_point = (points[i - 1] + points[i]) / 2.0;
        assert!((mid_point.length() - CURVE_RADIUS).abs() < 0.01);
    }
}

const CORNER_CURVE_RADIUS: f64 = 20.0;

fn straight_into_quarter_circle() -> VertexStorage {
    let mut source = VertexStorage::new();
    source.move_to(0.0, -30.0);
    source.line_to(0.0, 0.0);
    let center = Vector2::new(0.0, CORNER_CURVE_RADIUS);
    append_arc_about(
        &mut source,
        center,
        CORNER_CURVE_RADIUS,
        -PI / 2.0,
        -PI / 4.0,
    );
    append_arc_about(&mut source, center, CORNER_CURVE_RADIUS, -PI / 4.0, 0.0);
    source
}

fn append_arc_about(
    source: &mut VertexStorage,
    center: Vector2,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) {
    let k = 4.0 / 3.0 * ((end_angle - start_angle) / 4.0).tan();
    let start = center + (Vector2::new(start_angle.cos(), start_angle.sin()) * radius);
    let end = center + (Vector2::new(end_angle.cos(), end_angle.sin()) * radius);
    let start_tangent = Vector2::new(-start_angle.sin(), start_angle.cos()) * radius;
    let end_tangent = Vector2::new(-end_angle.sin(), end_angle.cos()) * radius;
    source.curve4_vector(start + (start_tangent * k), end - (end_tangent * k), end);
}

fn angle_along_corner_curve(point: Vector2) -> f64 {
    let from_center = point - Vector2::new(0.0, CORNER_CURVE_RADIUS);
    from_center.y.atan2(from_center.x) + (PI / 2.0)
}

#[test]
fn a_corner_beside_a_curve_still_gets_its_full_radius() {
    const SEGMENTS: usize = 8;
    const RADIUS: f64 = 2.0;
    let beveled = bevel_source(&straight_into_quarter_circle(), RADIUS, SEGMENTS as i32);
    let points = points_of(&beveled);
    assert!(points.len() > SEGMENTS + 2);
    let corner = Vector2::new(0.0, 0.0);
    let arc = &points[1..SEGMENTS + 2];
    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    assert_near((arc[0] - center).length(), RADIUS, 1e-9);

    let reach_along_straight = (((CORNER_CURVE_RADIUS + RADIUS) * (CORNER_CURVE_RADIUS + RADIUS))
        - (RADIUS * RADIUS))
        .sqrt()
        - CORNER_CURVE_RADIUS;
    assert_near(tangent_distance(corner, arc[0]), reach_along_straight, 0.02);
    assert_near(arc[0].x, 0.0, 1e-9);

    let arc_end = arc[SEGMENTS];
    assert_near(
        (arc_end - Vector2::new(0.0, CORNER_CURVE_RADIUS)).length(),
        CORNER_CURVE_RADIUS,
        0.02,
    );
    let reach_along_curve = (RADIUS / (CORNER_CURVE_RADIUS + RADIUS)).asin() * CORNER_CURVE_RADIUS;
    assert_near(
        angle_along_corner_curve(arc_end) * CORNER_CURVE_RADIUS,
        reach_along_curve,
        0.02,
    );
}

#[test]
fn the_vertices_an_arc_eats_are_gone_from_the_output() {
    const SEGMENTS: usize = 8;
    let beveled = bevel_source(&straight_into_quarter_circle(), 2.0, SEGMENTS as i32);
    let points = points_of(&beveled);
    let eaten_up_to = angle_along_corner_curve(points[1 + SEGMENTS]);
    for (i, point) in points.iter().enumerate() {
        if (1..=1 + SEGMENTS).contains(&i) {
            continue;
        }
        let angle = angle_along_corner_curve(*point);
        let inside_the_eaten_stretch = angle > 1e-9 && angle < eaten_up_to - 1e-9;
        assert!(
            !inside_the_eaten_stretch,
            "a flattened vertex the arc reached past must not still be in the output"
        );
    }
}

fn assert_matches(actual: &[Vector2], expected: &[(f64, f64)]) {
    assert_eq!(actual.len(), expected.len());
    for (point, (x, y)) in actual.iter().zip(expected) {
        assert_near(point.x, *x, 1e-9);
        assert_near(point.y, *y, 1e-9);
    }
}

#[test]
fn a_polygon_bevels_exactly_as_before() {
    let square = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];
    let expected_square = [
        (0.0, 3.0000000000000004),
        (0.22836140246613956, 1.851949702904731),
        (0.8786796564403572, 0.8786796564403576),
        (1.851949702904729, 0.22836140246614045),
        (2.9999999999999996, 0.0),
        (17.0, 0.0),
        (18.14805029709527, 0.22836140246614),
        (19.121320343559642, 0.8786796564403576),
        (19.77163859753386, 1.8519497029047314),
        (20.0, 3.000000000000001),
        (20.0, 17.0),
        (19.771638597533858, 18.14805029709527),
        (19.121320343559642, 19.121320343559642),
        (18.14805029709527, 19.771638597533858),
        (17.0, 20.0),
        (3.0, 20.0),
        (1.8519497029047307, 19.771638597533858),
        (0.8786796564403576, 19.121320343559642),
        (0.22836140246614, 18.14805029709527),
        (0.0, 17.0),
    ];
    assert_matches(&bevel_plain(&square, true, 3.0, 4), &expected_square);

    let hexagon: Vec<Vector2> = (0..6)
        .map(|i| {
            let a = PI * 2.0 * i as f64 / 6.0;
            Vector2::new(a.cos() * 10.0, a.sin() * 10.0)
        })
        .collect();
    let expected_hexagon = [
        (9.278312163512968, -1.2499999999999996),
        (9.57526803658239, -0.43412044416732576),
        (9.57526803658239, 0.43412044416732565),
        (9.278312163512968, 1.2499999999999996),
        (5.721687836487033, 7.4102540378443855),
        (5.163593351242286, 8.07536514564183),
        (4.411674685340109, 8.509485589809156),
        (3.5566243270259372, 8.660254037844386),
        (-3.5566243270259337, 8.660254037844387),
        (-4.411674685340106, 8.50948558980916),
        (-5.163593351242283, 8.075365145641832),
        (-5.721687836487032, 7.410254037844387),
        (-9.278312163512968, 1.2500000000000004),
        (-9.57526803658239, 0.4341204441673274),
        (-9.575268036582392, -0.43412044416732337),
        (-9.278312163512968, -1.249999999999997),
        (-5.721687836487036, -7.410254037844383),
        (-5.163593351242288, -8.075365145641829),
        (-4.411674685340111, -8.509485589809154),
        (-3.55662432702594, -8.660254037844384),
        (3.5566243270259363, -8.660254037844386),
        (4.411674685340109, -8.509485589809156),
        (5.163593351242286, -8.07536514564183),
        (5.721687836487034, -7.4102540378443855),
    ];
    assert_matches(&bevel_plain(&hexagon, true, 2.5, 3), &expected_hexagon);
}

fn two_corners_on_one_arc() -> VertexStorage {
    let mut source = VertexStorage::new();
    source.move_to(0.0, -10.0);
    source.line_to(0.0, 0.0);
    let center = Vector2::new(0.0, 4.0);
    append_arc_about(&mut source, center, 4.0, -PI / 2.0, -PI / 4.0);
    append_arc_about(&mut source, center, 4.0, -PI / 4.0, 0.0);
    source.line_to(14.0, 4.0);
    source
}

#[test]
fn two_corners_on_one_curved_run_share_it() {
    const SEGMENTS: usize = 4;
    const CURVE_RADIUS: f64 = 4.0;
    let source = two_corners_on_one_arc();
    let points = points_of(&bevel_source(&source, 5.0, SEGMENTS as i32));
    let first_arc = &points[1..SEGMENTS + 2];
    let second_start = points.len() - 2 - SEGMENTS;
    let second_arc = &points[second_start..second_start + SEGMENTS + 1];
    let run_length = CURVE_RADIUS * PI / 2.0;
    let first_center = arc_center(first_arc[0], first_arc[SEGMENTS / 2], first_arc[SEGMENTS]);
    let second_center = arc_center(
        second_arc[0],
        second_arc[SEGMENTS / 2],
        second_arc[SEGMENTS],
    );
    assert_near(
        (first_arc[0] - first_center).length(),
        run_length / 2.0,
        0.05,
    );
    assert_near(
        (second_arc[0] - second_center).length(),
        run_length / 2.0,
        0.05,
    );

    let flattened = flattened_points(&source);
    let first_reach =
        position_along(&flattened, first_arc[SEGMENTS]) - position_along(&flattened, Vector2::ZERO);
    let second_reach = position_along(&flattened, Vector2::new(4.0, 4.0))
        - position_along(&flattened, second_arc[0]);
    assert_near(first_reach, second_reach, 0.05);
    assert!(first_reach <= run_length - second_reach + 1e-9);
}

#[test]
fn the_arc_is_tangent_to_the_curve_where_it_lands() {
    const SEGMENTS: usize = 8;
    const RADIUS: f64 = 2.0;
    let source = straight_into_quarter_circle();
    let points = points_of(&bevel_source(&source, RADIUS, SEGMENTS as i32));
    let arc = &points[1..SEGMENTS + 2];
    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    let start = arc[0];
    let end = arc[SEGMENTS];
    let direction_at_start = Vector2::new(-(start.y - center.y), start.x - center.x);
    let direction_at_end = Vector2::new(-(end.y - center.y), end.x - center.x);
    let flattened = flattened_points(&source);

    assert!(angle_between_directions(direction_at_start, Vector2::new(0.0, 1.0)) < 1e-6);
    let end_misalignment =
        angle_between_directions(direction_at_end, local_direction_at(&flattened, end));
    assert!(math_helper::radians_to_degrees(end_misalignment) < 0.5);
    assert_near(center.x.abs(), RADIUS, 0.01);
    assert_near(
        (center - Vector2::new(0.0, CORNER_CURVE_RADIUS)).length(),
        CORNER_CURVE_RADIUS + RADIUS,
        0.01,
    );
    for point in arc {
        assert_near((*point - center).length(), RADIUS, 1e-9);
    }
}

#[test]
fn the_arc_radius_is_exactly_what_was_asked() {
    const SEGMENTS: usize = 8;
    const RADIUS: f64 = 2.0;
    let points = points_of(&bevel_source(
        &straight_into_quarter_circle(),
        RADIUS,
        SEGMENTS as i32,
    ));
    let arc = &points[1..SEGMENTS + 2];
    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    assert_near((arc[0] - center).length(), RADIUS, 1e-9);
}

fn two_arcs_meeting_at_a_corner() -> VertexStorage {
    let mut source = VertexStorage::new();
    source.move_to(-10.0, 10.0);
    let big = Vector2::new(0.0, 10.0);
    let small = Vector2::new(5.0, 0.0);
    append_arc_about(&mut source, big, 10.0, PI, PI * 1.25);
    append_arc_about(&mut source, big, 10.0, PI * 1.25, PI * 1.5);
    append_arc_about(&mut source, small, 5.0, PI, PI * 0.75);
    append_arc_about(&mut source, small, 5.0, PI * 0.75, PI * 0.5);
    source
}

#[test]
fn two_curves_meeting_at_a_corner_fillet_tangent_to_both() {
    const SEGMENTS: usize = 8;
    const RADIUS: f64 = 2.0;
    let source = two_arcs_meeting_at_a_corner();
    let points = points_of(&bevel_source(&source, RADIUS, SEGMENTS as i32));
    let flattened = flattened_points(&source);
    assert!(!points.iter().any(|p| p.length() < 1e-9));

    let arc = inserted_arc(&points, &flattened, 1e-6);
    assert_eq!(arc.len(), SEGMENTS + 1);
    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    for point in &arc {
        assert_near((*point - center).length(), RADIUS, 1e-9);
    }
    let direction_at_start = Vector2::new(-(arc[0].y - center.y), arc[0].x - center.x);
    let direction_at_end = Vector2::new(-(arc[SEGMENTS].y - center.y), arc[SEGMENTS].x - center.x);
    let start_misalignment =
        angle_between_directions(direction_at_start, local_direction_at(&flattened, arc[0]));
    let end_misalignment = angle_between_directions(
        direction_at_end,
        local_direction_at(&flattened, arc[SEGMENTS]),
    );
    assert!(math_helper::radians_to_degrees(start_misalignment) < 0.5);
    assert!(math_helper::radians_to_degrees(end_misalignment) < 0.5);
}

fn two_corners_inside_one_arc() -> VertexStorage {
    let mut source = VertexStorage::new();
    source.move_to(0.0, -30.0);
    source.line_to(0.0, 0.0);
    let center = Vector2::new(0.0, -4.0);
    append_arc_about(&mut source, center, 4.0, PI / 2.0, PI / 4.0);
    append_arc_about(&mut source, center, 4.0, PI / 4.0, 0.0);
    source.line_to(14.0, -4.0);
    source
}

#[test]
fn a_radius_too_big_for_its_curved_run_shrinks_to_what_fits() {
    const SEGMENTS: usize = 8;
    const CURVE_RADIUS: f64 = 4.0;
    let points = points_of(&bevel_source(
        &two_corners_inside_one_arc(),
        10.0,
        SEGMENTS as i32,
    ));
    let arc = &points[1..SEGMENTS + 2];
    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    let radius = (arc[0] - center).length();
    let largest_that_fits = CURVE_RADIUS * (2.0_f64.sqrt() - 1.0);
    assert_near(radius, largest_that_fits, 5e-3);
    for point in arc {
        assert_near((*point - center).length(), radius, 1e-9);
    }
    let end = arc[SEGMENTS];
    assert_near(
        (end - Vector2::new(0.0, -CURVE_RADIUS)).length(),
        CURVE_RADIUS,
        0.01,
    );

    let other_start = points.len() - 2 - SEGMENTS;
    let other_arc = &points[other_start..other_start + SEGMENTS + 1];
    let flattened = flattened_points(&two_corners_inside_one_arc());
    let reach = position_along(&flattened, end);
    let other_reach = position_along(&flattened, other_arc[0]);
    assert!(reach <= other_reach + 1e-9);
}

#[test]
fn a_smooth_run_gets_no_beveled_corners() {
    let source = quarter_circle();
    let beveled = bevel_source(&source, 0.5, 4);
    let flattened_count: usize = cleaned_contours(&source)
        .iter()
        .map(|contour| contour.points.len())
        .sum();
    assert_eq!(points_of(&beveled).len(), flattened_count);
}

const SPIKE_INTERIOR_ANGLE: f64 = 30.0;
const SPIKE_FLANK_RADIUS: f64 = 5.0;
const SPIKE_FLANK_SWEEP: f64 = 60.0;

fn spike_of_two_arcs(samples_per_flank: i32) -> Vec<Vector2> {
    let half = math_helper::degrees_to_radians(SPIKE_INTERIOR_ANGLE / 2.0);
    let left_center = Vector2::new(-half.cos(), -half.sin()) * SPIKE_FLANK_RADIUS;
    let right_center = Vector2::new(half.cos(), -half.sin()) * SPIKE_FLANK_RADIUS;
    let sweep = math_helper::degrees_to_radians(SPIKE_FLANK_SWEEP);
    let n = samples_per_flank as f64;
    let mut points = Vec::new();
    for i in 0..=samples_per_flank {
        let angle = half + sweep - (sweep * i as f64 / n);
        points.push(left_center + (Vector2::new(angle.cos(), angle.sin()) * SPIKE_FLANK_RADIUS));
    }
    for i in 1..=samples_per_flank {
        let angle = PI - half - (sweep * i as f64 / n);
        points.push(right_center + (Vector2::new(angle.cos(), angle.sin()) * SPIKE_FLANK_RADIUS));
    }
    points
}

const ARCH_INTERIOR_ANGLE: f64 = 150.0;
const ARCH_FLANK_RADIUS: f64 = 5.0;
const ARCH_FLANK_SWEEP: f64 = 135.0;
const ARCH_SAMPLES_PER_FLANK: i32 = 100;

fn arch_of_two_arcs(samples_per_flank: i32) -> Vec<Vector2> {
    let half = math_helper::degrees_to_radians(ARCH_INTERIOR_ANGLE / 2.0);
    let sweep = math_helper::degrees_to_radians(ARCH_FLANK_SWEEP);
    let left_center = Vector2::new(-half.cos(), -half.sin()) * ARCH_FLANK_RADIUS;
    let right_center = Vector2::new(half.cos(), -half.sin()) * ARCH_FLANK_RADIUS;
    let n = samples_per_flank as f64;
    let mut points = Vec::new();
    for i in (0..=samples_per_flank).rev() {
        let angle = half - (sweep * i as f64 / n);
        points.push(left_center + (Vector2::new(angle.cos(), angle.sin()) * ARCH_FLANK_RADIUS));
    }
    for i in 1..=samples_per_flank {
        let angle = PI - half + (sweep * i as f64 / n);
        points.push(right_center + (Vector2::new(angle.cos(), angle.sin()) * ARCH_FLANK_RADIUS));
    }
    points
}

#[test]
fn a_fillet_sweeping_past_half_a_circle_still_goes_round_the_corner_the_way_it_turns() {
    const SEGMENTS: usize = 16;
    const REQUESTED: f64 = 4.0;
    let arch = arch_of_two_arcs(ARCH_SAMPLES_PER_FLANK);
    let beveled = bevel_plain(&arch, false, REQUESTED, SEGMENTS as i32);
    let arc = inserted_arc(&beveled, &arch, 1e-7);
    assert_eq!(arc.len(), SEGMENTS + 1);

    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    let radius = (arc[0] - center).length();
    assert!(radius < REQUESTED);
    for point in &arc {
        assert_near((*point - center).length(), radius, 1e-6 * radius);
    }
    let sweep = sweep_about(&arc, center);
    assert!(
        sweep > PI,
        "the limiting fillet lands just past half a circle"
    );
    assert!(math_helper::radians_to_degrees(sweep) < 190.0);

    let apex_to_center = center.length();
    assert_near(
        distance_to_polyline(&arc, Vector2::ZERO),
        apex_to_center - radius,
        0.05,
    );
    assert!(!self_intersects(&beveled));
}

fn work_of(contour: &[Vector2]) -> (u64, Vec<Vector2>) {
    path_bevel_arcs::reset_run_places_measured();
    let beveled = bevel_plain(contour, false, 100.0, 8);
    (path_bevel_arcs::run_places_measured(), beveled)
}

#[test]
fn a_radius_that_cannot_fit_is_found_quickly_on_a_long_run() {
    let short_run = spike_of_two_arcs(500);
    let long_run = spike_of_two_arcs(2000);
    let (short_work, _) = work_of(&short_run);
    let (long_work, beveled) = work_of(&long_run);
    assert!(!beveled.is_empty());
    assert!(short_work > 0);
    assert!(
        long_work < short_work * 5,
        "the construction has to grow with the run, not its square: {short_work} -> {long_work}"
    );
}
