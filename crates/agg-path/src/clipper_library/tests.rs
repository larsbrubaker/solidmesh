//! Rust-only checks of the clipper_library helpers MatterCAD calls but agg-sharp's tests do not
//! cover: Clipper 6's area sign and point-in-polygon, the PolyTree walks, and
//! DataConverters2D's winding correction and round trip.

use clipper2_rust::{make_path64, FillRule, Paths64};

use super::*;
use crate::data_converters_2d::{create_vertex_storage, get_corrected_winding};
use crate::VertexSourceToClipperPolygons;

/// A square `outer` across with a square hole `inner` across, both counter-clockwise.
fn frame(outer: i64, inner: i64) -> Paths64 {
    vec![
        make_path64(&[0, 0, outer, 0, outer, outer, 0, outer]),
        make_path64(&[
            (outer - inner) / 2,
            (outer - inner) / 2,
            (outer + inner) / 2,
            (outer - inner) / 2,
            (outer + inner) / 2,
            (outer + inner) / 2,
            (outer - inner) / 2,
            (outer + inner) / 2,
        ]),
    ]
}

#[test]
fn rust_only_area_is_positive_counter_clockwise() {
    let square = make_path64(&[0, 0, 10, 0, 10, 10, 0, 10]);
    assert_eq!(area(&square), 100.0);
    assert!(orientation(&square));
    let mut clockwise = square.clone();
    clockwise.reverse();
    assert_eq!(area(&clockwise), -100.0);
    assert!(!orientation(&clockwise));
}

#[test]
fn rust_only_point_in_polygon_reads_inside_outside_and_on() {
    let square = make_path64(&[0, 0, 10, 0, 10, 10, 0, 10]);
    assert_eq!(point_in_polygon(int_point(5.0, 5.0), &square), 1);
    assert_eq!(point_in_polygon(int_point(15.0, 5.0), &square), 0);
    assert_eq!(point_in_polygon(int_point(10.0, 5.0), &square), -1);
}

#[test]
fn rust_only_simplify_polygon_splits_a_bow_tie() {
    let bow_tie = make_path64(&[0, 0, 10, 10, 10, 0, 0, 10]);
    assert_eq!(simplify_polygon(&bow_tie, FillRule::NonZero).len(), 2);
}

#[test]
fn rust_only_a_frame_is_one_outline_with_its_hole() {
    let frame = frame(100, 50);

    let sets = frame.separate_into_outlines_and_contained_holes();
    assert_eq!(sets.len(), 1);
    assert_eq!(sets[0].len(), 2);

    let groups = frame.separate_polygon_groups();
    assert_eq!(groups.len(), 1);
    assert_eq!(groups[0].len(), 2);

    let described = frame.describe_all_polygons();
    let shape: Vec<(i32, i32, i32, i32)> = described
        .iter()
        .map(|d| (d.group, d.count, d.path, d.depth))
        .collect();
    assert_eq!(shape, vec![(0, 1, 0, 0), (0, 1, 1, 1)]);
    assert!(orientation(&described[0].polygon));
    assert!(!orientation(&described[1].polygon));
}

#[test]
fn rust_only_corrected_winding_turns_the_hole_clockwise() {
    let corrected = get_corrected_winding(&frame(100_000, 50_000));
    assert_eq!(corrected.len(), 2);
    let orientations: Vec<bool> = corrected.iter().map(orientation).collect();
    assert_eq!(orientations.iter().filter(|o| **o).count(), 1);
    assert_eq!(
        area(&corrected[0]).abs() + area(&corrected[1]).abs(),
        1.25e10
    );
}

#[test]
fn rust_only_vertex_storage_round_trips_through_polygons() {
    let polygons = frame(100_000, 50_000);
    let storage = create_vertex_storage(&polygons, 1000.0);
    assert_eq!(storage.create_polygons(1000.0), polygons);
}

#[test]
fn rust_only_booleans_drop_collinear_points_as_clipper_6_does() {
    // Two squares side by side union to one rectangle; Clipper 6 drops the shared edge's ends.
    let left: Paths64 = vec![make_path64(&[0, 0, 10, 0, 10, 10, 0, 10])];
    let right: Paths64 = vec![make_path64(&[10, 0, 20, 0, 20, 10, 10, 10])];
    let union = left.union(&right, FillRule::EvenOdd);
    assert_eq!(union.len(), 1);
    assert_eq!(union[0].len(), 4);
}
