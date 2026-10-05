//! Rust-only checks of the bevel core's R1.P07 package (the facet cut, the room crossing, the end
//! section readings and the run-out's column ends). The C# classes R1.P07 unlocks
//! (`SurfaceCutRulerTests`, `SurfaceEndGainsTheSectionReadingTests`, `SurfaceRunOutColumnTopTests`)
//! need the whole package; these pin the pieces alone.

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
