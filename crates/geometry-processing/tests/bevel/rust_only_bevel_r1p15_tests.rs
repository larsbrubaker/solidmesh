//! Rust-only checks of bevel-core package R1.P15's leaves: the point field's exact queries, the
//! room field's setbacks, the canonical maximum's tolerant offer and the tie-tolerant edge test,
//! the zip rail's canonical and unwrapped readings, and the corner strip's nearest rail vertex.
//! Their C# coverage comes through the whole-build test classes of later packages.

use geometry_processing::bevel::level_line::bounded_maximum::not_shorter;
use geometry_processing::bevel::level_line::corner_rail_strip::nearest;
use geometry_processing::bevel::level_line::{
    BoundedMaximumCanonicalMaximum, ChainDistanceField, PointField, ScalarField, ZipRail,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

#[test]
fn rust_only_a_point_field_answers_its_extremes_exactly() {
    let sphere = PointField::sphere(v(0.0, 0.0, 0.0), 1.0);
    assert_eq!(sphere.value_at(v(3.0, 4.0, 0.0)).ok(), Some(4.0));
    let least = sphere
        .min_along_segment(v(-2.0, 2.0, 0.0), v(2.0, 2.0, 0.0))
        .expect("uncharged");
    assert_eq!((least.value, least.point), (1.0, v(0.0, 2.0, 0.0)));
    let tri = sphere
        .min_over_triangle(v(-1.0, -1.0, 2.0), v(1.0, -1.0, 2.0), v(0.0, 1.0, 2.0))
        .expect("uncharged");
    assert_eq!((tri.value, tri.point), (1.0, v(0.0, 0.0, 2.0)));
    let most = sphere
        .max_over_triangle(v(1.0, 0.0, 0.0), v(5.0, 0.0, 0.0), v(0.0, 2.0, 0.0), 0.1)
        .expect("uncharged");
    assert_eq!((most.value, most.point), (4.0, v(5.0, 0.0, 0.0)));
    let floor = sphere
        .max_along_segment_at_least(v(1.0, 0.0, 0.0), v(2.0, 0.0, 0.0), 0.1, 9.0)
        .expect("uncharged");
    assert_eq!((floor.value, floor.point), (9.0, v(1.0, 0.0, 0.0)));

    let plane = PointField::plane(v(0.0, 0.0, 0.0), v(0.0, 0.0, -1.0), 0.5);
    assert_eq!(plane.value_at(v(7.0, 7.0, -2.0)).ok(), Some(1.5));
    let least = plane
        .min_along_segment(v(0.0, 0.0, 1.0), v(0.0, 0.0, -1.0))
        .expect("uncharged");
    assert_eq!((least.value, least.point), (-1.5, v(0.0, 0.0, 1.0)));
}

#[test]
fn rust_only_the_canonical_maximum_keeps_the_first_of_a_tie() {
    let mut best = BoundedMaximumCanonicalMaximum::new(f64::NEG_INFINITY, v(9.0, 9.0, 9.0));
    best.offer(1.0, v(1.0, 0.0, 0.0));
    best.offer(1.0 + 1e-12, v(2.0, 0.0, 0.0));
    assert_eq!(best.best_point, v(1.0, 0.0, 0.0));
    best.offer(1.1, v(3.0, 0.0, 0.0));
    assert_eq!((best.best, best.best_point), (1.1, v(3.0, 0.0, 0.0)));

    assert!(not_shorter(1.0, 1.0 + 1e-13));
    assert!(!not_shorter(1.0, 1.0 + 1e-9));
}

#[test]
fn rust_only_a_zip_rail_reads_canonically_and_past_its_wrap() {
    let rail = ZipRail {
        vertices: vec![4, 5, 6],
        parameters: vec![0.5, 1.5, 2.5],
        reversed: true,
    };
    assert_eq!(rail.count(), 3);
    let canonical = ZipRail::canonical(&rail, &[(0.4, 0.6), (1.5, 1.5), (2.0, 3.0)]);
    assert_eq!(canonical.parameters, vec![0.4, 1.5, 2.0]);
    assert!(canonical.reversed);
    assert_eq!(ZipRail::unwrapped(&rail, 1, 10.0, 0.25), 1.75);
    assert_eq!(ZipRail::unwrapped(&rail, 3, 10.0, 0.25), 10.75);

    let parameters = [0.0, 1.0, 2.0, 3.0];
    assert_eq!(nearest(&parameters, 1, 1.8, 1), 2);
    assert_eq!(nearest(&parameters, 1, 1.2, 1), 1);
    assert_eq!(nearest(&parameters, 3, 9.0, 1), 3);
}
