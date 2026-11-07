//! The geometry of the path bevel pinned directly - arcs tangent to their edges, and corners that
//! share an edge agreeing on how to split it.
//!
//! `a_reflex_corner_is_filleted_on_the_outside_of_the_turn` runs both windings inside
//! the one test.

use geometry_processing::path::path_bevel::{bevel, bevel_contour};
use geometry_processing::path::{Contour, PathBevelSizeMode};
use vector_math::Vector2;

use super::path_bevel_test_geometry::{arc_center, tangent_distance};

fn square(side: f64) -> Vec<Vector2> {
    vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(side, 0.0),
        Vector2::new(side, side),
        Vector2::new(0.0, side),
    ]
}

/// A contour bevel with the defaults: no selection, Radius mode, contour 0.
fn bevel_plain(
    contour: &[Vector2],
    closed: bool,
    size: f64,
    segments: i32,
    bend: f64,
) -> Vec<Vector2> {
    bevel_contour(
        contour,
        closed,
        size,
        segments,
        bend,
        None,
        PathBevelSizeMode::Radius,
        0,
    )
}

fn bevel_setback(contour: &[Vector2], size: f64, segments: i32) -> Vec<Vector2> {
    bevel_contour(
        contour,
        true,
        size,
        segments,
        10.0,
        None,
        PathBevelSizeMode::TangentDistance,
        0,
    )
}

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

#[test]
fn a_squares_corners_become_arcs_tangent_to_its_edges() {
    const RADIUS: f64 = 3.0;
    const SEGMENTS: usize = 4;
    let beveled = bevel_plain(&square(20.0), true, RADIUS, SEGMENTS as i32, 10.0);

    assert_eq!(beveled.len(), 4 * (SEGMENTS + 1));
    assert!(
        beveled
            .iter()
            .all(|p| p.x >= -1e-9 && p.x <= 20.0 + 1e-9 && p.y >= -1e-9 && p.y <= 20.0 + 1e-9),
        "every arc point has to stay inside the original bounds"
    );

    let centers = [
        Vector2::new(3.0, 3.0),
        Vector2::new(17.0, 3.0),
        Vector2::new(17.0, 17.0),
        Vector2::new(3.0, 17.0),
    ];
    for (corner, center) in centers.iter().enumerate() {
        for point in 0..SEGMENTS + 1 {
            let arc_point = beveled[corner * (SEGMENTS + 1) + point];
            assert_near((arc_point - *center).length(), RADIUS, 1e-6);
        }
    }
}

#[test]
fn a_squares_arc_center_is_the_bisector_point() {
    const RADIUS: f64 = 3.0;
    const SEGMENTS: usize = 4;
    let beveled = bevel_plain(&square(20.0), true, RADIUS, SEGMENTS as i32, 10.0);
    let arc = &beveled[0..SEGMENTS + 1];
    let center = arc_center(arc[0], arc[SEGMENTS / 2], arc[SEGMENTS]);
    assert_near(center.x, RADIUS, 1e-12);
    assert_near(center.y, RADIUS, 1e-12);
    assert_near((arc[0] - center).length(), RADIUS, 1e-12);
}

#[test]
fn one_segment_gives_the_chamfer_chord_between_the_tangent_points() {
    const RADIUS: f64 = 3.0;
    let beveled = bevel_plain(&square(20.0), true, RADIUS, 1, 10.0);
    assert_eq!(beveled.len(), 4 * 2);
    assert!((beveled[0] - Vector2::new(0.0, RADIUS)).length() < 1e-9);
    assert!((beveled[1] - Vector2::new(RADIUS, 0.0)).length() < 1e-9);
}

#[test]
fn corners_competing_for_one_short_edge_split_it_evenly() {
    let rectangle = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(10.0, 0.0),
        Vector2::new(10.0, 40.0),
        Vector2::new(0.0, 40.0),
    ];
    const SEGMENTS: usize = 4;
    let beveled = bevel_plain(&rectangle, true, 8.0, SEGMENTS as i32, 10.0);

    let first_radius = tangent_distance(rectangle[0], beveled[0]);
    let second_radius = tangent_distance(rectangle[1], beveled[SEGMENTS + 1]);
    assert_near(first_radius, 5.0, 0.01);
    assert_near(second_radius, 5.0, 0.01);
    assert_near(first_radius + second_radius, 10.0, 0.01);

    let end_of_first_arc = beveled[SEGMENTS];
    let start_of_second_arc = beveled[SEGMENTS + 1];
    assert!((end_of_first_arc - start_of_second_arc).length() < 0.01);

    let rotated = vec![rectangle[1], rectangle[2], rectangle[3], rectangle[0]];
    let from_rotated = bevel_plain(&rotated, true, 8.0, SEGMENTS as i32, 10.0);
    assert_near(
        tangent_distance(rotated[0], from_rotated[0]),
        second_radius,
        1e-9,
    );
}

#[test]
fn corners_with_room_to_spare_each_keep_the_full_radius() {
    const RADIUS: f64 = 8.0;
    const SEGMENTS: usize = 4;
    let square = square(40.0);
    let beveled = bevel_plain(&square, true, RADIUS, SEGMENTS as i32, 10.0);
    for (corner, point) in square.iter().enumerate() {
        assert_near(
            tangent_distance(*point, beveled[corner * (SEGMENTS + 1)]),
            RADIUS,
            1e-9,
        );
    }
}

#[test]
fn a_vertex_that_barely_bends_is_left_where_it_is() {
    let nearly_straight = Vector2::new(10.0, 0.2);
    let path = vec![
        Vector2::new(0.0, 0.0),
        nearly_straight,
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];
    const SEGMENTS: usize = 4;
    let beveled = bevel_plain(&path, true, 2.0, SEGMENTS as i32, 10.0);
    assert_eq!(beveled.len(), 4 * (SEGMENTS + 1) + 1);
    assert_eq!(beveled.iter().filter(|p| **p == nearly_straight).count(), 1);
}

#[test]
fn only_selected_vertices_are_beveled() {
    let square = vec![
        Vector2::new(-10.0, -10.0),
        Vector2::new(10.0, -10.0),
        Vector2::new(10.0, 10.0),
        Vector2::new(-10.0, 10.0),
    ];
    const SEGMENTS: usize = 4;
    let selection = |_contour: usize, _index: usize, position: Vector2| position.x < 0.0;
    let beveled = bevel_contour(
        &square,
        true,
        3.0,
        SEGMENTS as i32,
        10.0,
        Some(&selection),
        PathBevelSizeMode::Radius,
        0,
    );
    assert_eq!(beveled.len(), 2 * (SEGMENTS + 1) + 2);
    assert!(beveled.contains(&Vector2::new(10.0, -10.0)));
    assert!(beveled.contains(&Vector2::new(10.0, 10.0)));
    assert!(!beveled.contains(&Vector2::new(-10.0, -10.0)));
    assert!(!beveled.contains(&Vector2::new(-10.0, 10.0)));
}

#[test]
fn an_open_contour_keeps_its_ends_and_bevels_what_is_between_them() {
    let polyline = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(10.0, 0.0),
        Vector2::new(10.0, 10.0),
    ];
    const SEGMENTS: usize = 4;
    let beveled = bevel_plain(&polyline, false, 3.0, SEGMENTS as i32, 10.0);
    assert_eq!(beveled.len(), SEGMENTS + 1 + 2);
    assert_eq!(beveled[0], polyline[0]);
    assert_eq!(beveled[beveled.len() - 1], polyline[2]);
    assert!((beveled[1] - Vector2::new(7.0, 0.0)).length() < 1e-9);
    assert!((beveled[SEGMENTS + 1] - Vector2::new(10.0, 3.0)).length() < 1e-9);
}

#[test]
fn a_reflex_corner_is_filleted_on_the_outside_of_the_turn() {
    for clockwise in [false, true] {
        let mut l_shape = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(20.0, 0.0),
            Vector2::new(20.0, 10.0),
            Vector2::new(10.0, 10.0),
            Vector2::new(10.0, 20.0),
            Vector2::new(0.0, 20.0),
        ];
        if clockwise {
            l_shape.reverse();
        }
        const RADIUS: f64 = 3.0;
        const SEGMENTS: usize = 4;
        let beveled = bevel_plain(&l_shape, true, RADIUS, SEGMENTS as i32, 10.0);
        assert_eq!(beveled.len(), 6 * (SEGMENTS + 1));

        let reflex = l_shape
            .iter()
            .position(|p| *p == Vector2::new(10.0, 10.0))
            .unwrap();
        let arc = &beveled[reflex * (SEGMENTS + 1)..(reflex + 1) * (SEGMENTS + 1)];
        let center = Vector2::new(13.0, 13.0);
        for point in arc {
            assert_near((*point - center).length(), RADIUS, 1e-6);
        }
        let tangent_points = [arc[0], arc[arc.len() - 1]];
        assert!(tangent_points
            .iter()
            .any(|p| (*p - Vector2::new(13.0, 10.0)).length() < 1e-9));
        assert!(tangent_points
            .iter()
            .any(|p| (*p - Vector2::new(10.0, 13.0)).length() < 1e-9));
    }
}

#[test]
fn a_cusp_vertex_is_left_alone_rather_than_becoming_nan() {
    let cusp = Vector2::new(10.0, 0.0);
    let doubles_back = vec![
        Vector2::new(0.0, 0.0),
        cusp,
        Vector2::new(5.0, 0.0),
        Vector2::new(5.0, 10.0),
    ];
    let beveled = bevel_plain(&doubles_back, true, 3.0, 4, 10.0);
    assert!(beveled.iter().all(|p| p.x.is_finite() && p.y.is_finite()));
    assert_eq!(beveled.iter().filter(|p| **p == cusp).count(), 1);
}

#[test]
fn a_repeated_mid_path_vertex_does_not_survive_the_bevel() {
    let with_repeat = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];
    let beveled = bevel_plain(&with_repeat, true, 3.0, 4, 10.0);
    let plain = bevel_plain(&square(20.0), true, 3.0, 4, 10.0);
    assert_eq!(beveled, plain);
}

/// Every point of every beveled contour, in order.
fn drawn_points(contours: &[Contour]) -> Vec<Vector2> {
    contours
        .iter()
        .flat_map(|contour| contour.points.iter().copied())
        .collect()
}

#[test]
fn an_open_contour_that_ends_where_it_started_keeps_its_last_edge() {
    let source = [Contour {
        points: vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(20.0, 0.0),
            Vector2::new(20.0, 20.0),
            Vector2::new(0.0, 20.0),
            Vector2::new(0.0, 0.0),
        ],
        is_closed: false,
    }];
    const SEGMENTS: usize = 4;
    let beveled = bevel(
        &source,
        3.0,
        SEGMENTS as i32,
        10.0,
        None,
        PathBevelSizeMode::Radius,
    );
    let points = drawn_points(&beveled);
    assert_eq!(points.len(), 3 * (SEGMENTS + 1) + 2);
    assert_eq!(points[0], Vector2::new(0.0, 0.0));
    assert_eq!(points[points.len() - 1], Vector2::new(0.0, 0.0));
    assert!(!beveled.iter().any(|contour| contour.is_closed));
}

#[test]
fn nothing_to_bevel_passes_straight_through() {
    let square = square(20.0);
    assert_eq!(bevel_plain(&square, true, 0.0, 4, 10.0), square);
    assert_eq!(bevel_plain(&square, true, f64::NAN, 4, 10.0), square);
    let two_points = vec![Vector2::new(0.0, 0.0), Vector2::new(10.0, 0.0)];
    assert_eq!(bevel_plain(&two_points, true, 3.0, 4, 10.0), two_points);
}

#[test]
fn a_repeated_closing_vertex_does_not_become_a_stray_point() {
    let square = square(20.0);
    let mut repeated = square.clone();
    repeated.push(square[0]);
    let from_repeated = bevel_plain(&repeated, true, 3.0, 4, 10.0);
    let from_plain = bevel_plain(&square, true, 3.0, 4, 10.0);
    assert_eq!(from_repeated, from_plain);
}

#[test]
fn beveling_contours_keeps_each_contours_openness() {
    let source = [
        Contour {
            points: square(20.0),
            is_closed: true,
        },
        Contour {
            points: vec![
                Vector2::new(40.0, 0.0),
                Vector2::new(50.0, 0.0),
                Vector2::new(50.0, 10.0),
            ],
            is_closed: false,
        },
    ];
    const SEGMENTS: usize = 4;
    let beveled = bevel(
        &source,
        3.0,
        SEGMENTS as i32,
        10.0,
        None,
        PathBevelSizeMode::Radius,
    );
    assert_eq!(beveled.len(), 2);
    assert_eq!(
        beveled.iter().filter(|contour| contour.is_closed).count(),
        1
    );
    assert_eq!(
        drawn_points(&beveled).len(),
        4 * (SEGMENTS + 1) + (SEGMENTS + 1 + 2)
    );
}

#[test]
fn tangent_distance_mode_sets_the_corner_back_by_the_size_asked() {
    const SIZE: f64 = 3.0;
    let triangle = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(10.0, 10.0 * 3.0_f64.sqrt()),
    ];
    let chamfered = bevel_setback(&triangle, SIZE, 1);
    assert_eq!(chamfered.len(), 3 * 2);
    for (corner, point) in triangle.iter().enumerate() {
        assert_near(tangent_distance(*point, chamfered[corner * 2]), SIZE, 1e-9);
        assert_near(
            tangent_distance(*point, chamfered[corner * 2 + 1]),
            SIZE,
            1e-9,
        );
    }

    let filleted = bevel_plain(&triangle, true, SIZE, 1, 10.0);
    assert_near(
        tangent_distance(triangle[0], filleted[0]),
        SIZE / (std::f64::consts::PI / 6.0).tan(),
        1e-9,
    );
}

#[test]
fn tangent_distance_mode_sets_every_corner_back_equally_however_sharp_it_is() {
    const SIZE: f64 = 2.0;
    const SEGMENTS: usize = 4;
    let polygon = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(30.0, 0.0),
        Vector2::new(36.0, 14.0),
        Vector2::new(18.0, 30.0),
        Vector2::new(2.0, 16.0),
    ];
    let chamfered = bevel_setback(&polygon, SIZE, SEGMENTS as i32);
    let filleted = bevel_plain(&polygon, true, SIZE, SEGMENTS as i32, 10.0);

    let mut radius_mode_setbacks = Vec::new();
    for (corner, point) in polygon.iter().enumerate() {
        let arc_start = corner * (SEGMENTS + 1);
        let arc_end = arc_start + SEGMENTS;
        assert_near(tangent_distance(*point, chamfered[arc_start]), SIZE, 1e-9);
        assert_near(tangent_distance(*point, chamfered[arc_end]), SIZE, 1e-9);
        radius_mode_setbacks.push(tangent_distance(*point, filleted[arc_start]));
    }
    let max = radius_mode_setbacks
        .iter()
        .copied()
        .fold(f64::MIN, f64::max);
    let min = radius_mode_setbacks
        .iter()
        .copied()
        .fold(f64::MAX, f64::min);
    assert!(max - min > 0.1);
}

#[test]
fn tangent_distance_mode_still_shares_a_contested_edge() {
    const SEGMENTS: usize = 4;
    let trapezoid = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(10.0, 0.0),
        Vector2::new(12.0, 20.0),
        Vector2::new(-8.0, 20.0),
    ];
    let chamfered = bevel_setback(&trapezoid, 8.0, SEGMENTS as i32);
    let first_setback = tangent_distance(trapezoid[0], chamfered[0]);
    let second_setback = tangent_distance(trapezoid[1], chamfered[SEGMENTS + 1]);
    assert_near(first_setback, 5.0, 0.01);
    assert_near(second_setback, 5.0, 0.01);
    assert_near(first_setback + second_setback, 10.0, 0.01);
    assert!((chamfered[SEGMENTS] - chamfered[SEGMENTS + 1]).length() < 0.01);

    let filleted = bevel_plain(&trapezoid, true, 8.0, SEGMENTS as i32, 10.0);
    let first_radius_setback = tangent_distance(trapezoid[0], filleted[0]);
    let second_radius_setback = tangent_distance(trapezoid[1], filleted[SEGMENTS + 1]);
    assert!((first_radius_setback - second_radius_setback).abs() > 0.5);
}

#[test]
fn a_contour_closing_on_a_near_duplicate_of_its_first_point_bevels_as_one_corner() {
    const SEGMENTS: usize = 4;
    let mut square = square(20.0);
    square.push(Vector2::new(0.0, 5e-11));
    let beveled = bevel_plain(&square, true, 3.0, SEGMENTS as i32, 10.0);
    assert_eq!(beveled.len(), 4 * (SEGMENTS + 1));
}

#[test]
fn a_chamfer_cuts_a_corner_that_bends_less_than_a_right_angle() {
    const RADIUS: f64 = 1.0;
    const POINTS: usize = 5;
    const CIRCUMRADIUS: f64 = 10.0;
    let pentagon: Vec<Vector2> = (0..POINTS)
        .map(|i| {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / POINTS as f64;
            Vector2::new(CIRCUMRADIUS * angle.cos(), CIRCUMRADIUS * angle.sin())
        })
        .collect();
    let chamfered = bevel_plain(&pentagon, true, RADIUS, 1, 10.0);
    assert_eq!(chamfered.len(), POINTS * 2);
    let expected_setback = RADIUS / (std::f64::consts::PI * 108.0 / 180.0 / 2.0).tan();
    for (corner, point) in pentagon.iter().enumerate() {
        assert_near(
            tangent_distance(*point, chamfered[corner * 2]),
            expected_setback,
            1e-9,
        );
        assert_near(
            tangent_distance(*point, chamfered[(corner * 2) + 1]),
            expected_setback,
            1e-9,
        );
    }
}

#[test]
fn a_vertex_that_barely_turns_is_left_alone_even_with_no_bend_threshold() {
    let almost_straight = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(10.0, 1e-9),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];
    let beveled = bevel_plain(&almost_straight, true, 3.0, 4, 0.0);
    assert_eq!(beveled.len(), (4 * 5) + 1);
    assert!(beveled
        .iter()
        .any(|p| (*p - almost_straight[1]).length() < 1e-9));
}
