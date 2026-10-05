//! Rust-only checks of the R1.P14 roll-across-the-wall geometry: the two-plane centre, the
//! out-of-the-facet lean, the section projections and the finiteness checks.

use geometry_processing::bevel::surface::surface_roll_across_the_wall::{
    in_the_section, in_the_section_of_the_normal, is_finite, is_finite_vector, out_of_the_facet,
    try_centre,
};
use vector_math::Vector3;

#[test]
fn rust_only_the_centre_of_a_square_corner_sits_offset_from_both_planes() {
    // Flat face z = 0 (normal +z), facet x = 0 (normal +x), edge along y.
    let centre = try_centre(
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        -2.0,
    )
    .expect("square planes have a centre");
    assert!((centre - Vector3::new(-2.0, 0.0, -2.0)).length() < 1e-12);
}

#[test]
fn rust_only_parallel_planes_have_no_centre() {
    let up = Vector3::new(0.0, 0.0, 1.0);
    assert!(try_centre(
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        up,
        Vector3::new(1.0, 0.0, 0.0),
        up,
        1.0,
    )
    .is_none());
}

#[test]
fn rust_only_out_of_the_facet_leans_with_the_facet_normal() {
    let along = Vector3::new(0.0, 1.0, 0.0);
    let trace = Vector3::new(1.0, 0.0, 0.0);
    let normal = Vector3::new(0.0, 0.0, 1.0);
    let out = out_of_the_facet(along, trace, normal);
    assert!((out - normal).length() < 1e-12);
    let flipped = out_of_the_facet(along, trace, -normal);
    assert!((flipped + normal).length() < 1e-12);
    // A trace along the edge leaves nothing square to both: the normal itself.
    assert_eq!(out_of_the_facet(along, along, normal), normal);
}

#[test]
fn rust_only_the_section_projections_drop_the_along_component() {
    let along = Vector3::new(0.0, 1.0, 0.0);
    let p = in_the_section(
        Vector3::new(1.0, 5.0, 2.0),
        Vector3::new(0.0, 1.0, 0.0),
        along,
    );
    assert!((p - Vector3::new(1.0, 1.0, 2.0)).length() < 1e-12);
    let n = in_the_section_of_the_normal(Vector3::new(3.0, 7.0, 4.0), along);
    assert!((n - Vector3::new(0.6, 0.0, 0.8)).length() < 1e-12);
    assert_eq!(in_the_section_of_the_normal(along, along), along);
}

#[test]
fn rust_only_finiteness_refuses_nan_and_infinities() {
    assert!(is_finite(1.0));
    assert!(!is_finite(f64::NAN));
    assert!(!is_finite(f64::NEG_INFINITY));
    assert!(is_finite_vector(Vector3::new(1.0, 2.0, 3.0)));
    assert!(!is_finite_vector(Vector3::new(1.0, f64::INFINITY, 3.0)));
}
