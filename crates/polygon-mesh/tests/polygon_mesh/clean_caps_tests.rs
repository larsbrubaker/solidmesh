//! Revolve and bevelled Linear Extrude make clean solids from paths whose loops touch: a hole
//! touching its outline at a point or along an edge, two parts meeting at a corner, a bowtie, and
//! the traced picture outline. Each solid is closed on shared vertices, wound outward and free of
//! zero-area faces, and every flat cap covers exactly the area the path fills (the checker and
//! the paths are in `clean_solid_support.rs`).

use agg_path::clipper_library::JoinType;
use agg_path::VertexStorage;
use polygon_mesh::processors::vertex_source_to_mesh::{extrude_bevelled, revolve};
use polygon_mesh::Mesh;
use std::f64::consts::FRAC_PI_2;
use vector_math::math_helper::TAU;

use super::clean_solid_support::{
    assert_closed_outward, even_odd_area, flat_area, moved, path_of, square_at,
    touching_loop_cases, traced_silhouette, AREA_FRACTION,
};

/// Asserts the faces flat where coordinate `axis` is `at` all face one way and cover `area`.
fn assert_cap(mesh: &Mesh, axis: usize, at: f64, area: f64, what: &str) {
    assert_cap_within(mesh, axis, at, area, 1e-6 * area.max(1.0), what);
}

/// [`assert_cap`] within `tolerance` mm².
fn assert_cap_within(mesh: &Mesh, axis: usize, at: f64, area: f64, tolerance: f64, what: &str) {
    let (up, down) = flat_area(mesh, axis, at);
    assert!(
        up.min(down) <= tolerance && (up.max(down) - area).abs() <= tolerance,
        "{what}: the cap covers {up} mm² facing one way and {down} mm² the other, the path fills \
         {area} mm²"
    );
}

#[test]
fn touching_loops_revolve_to_clean_solids() {
    for (name, path) in touching_loop_cases() {
        // off the axis, so the profile is revolved as drawn
        let path = moved(&path, 1.0, 0.0);
        let area = even_odd_area(&path);
        let full = revolve(&path, 24, 0.0, TAU, false).expect("angles in range");
        assert_closed_outward(&full, &format!("{name}, full turn"));

        let quarter = revolve(&path, 6, 0.0, FRAC_PI_2, false).expect("angles in range");
        let what = format!("{name}, quarter turn");
        assert_closed_outward(&quarter, &what);
        // the start cap lies in z = 0, the end cap a quarter turn round about y, in x = 0
        assert_cap(&quarter, 2, 0.0, area, &format!("{what}, start cap"));
        assert_cap(&quarter, 0, 0.0, area, &format!("{what}, end cap"));
    }
}

#[test]
fn a_traced_picture_outline_revolves_to_a_clean_solid() {
    let path = traced_silhouette();
    let full = revolve(&path, 24, 0.0, TAU, false).expect("angles in range");
    assert_closed_outward(&full, "traced silhouette, full turn");
    let partial = revolve(&path, 8, 0.0, FRAC_PI_2, false).expect("angles in range");
    assert_closed_outward(&partial, "traced silhouette, quarter turn");
}

/// Extrudes `path` 3 mm with a bevel at the top and, when `both`, at the bottom too, and holds
/// it to the rule: a closed outward solid whose flat bottom (when not bevelled) covers the path,
/// within `area_fraction` of its area.
fn assert_bevel_extrudes_cleanly(path: &VertexStorage, both: bool, area_fraction: f64, what: &str) {
    // two steps of 0.4 mm inward, rounded at the corners
    let top = [(2.0, -0.2), (2.5, -0.4)];
    let bottom = [(1.0, -0.2), (0.5, -0.4)];
    let mesh = extrude_bevelled(
        path,
        3.0,
        Some(&top),
        JoinType::Round,
        both.then_some(&bottom[..]),
        JoinType::Round,
    )
    .expect("the profile cleans up");
    assert_closed_outward(&mesh, what);
    if !both {
        let area = even_odd_area(path);
        let tolerance = area_fraction * area.max(1.0);
        assert_cap_within(&mesh, 2, 0.0, area, tolerance, &format!("{what}, bottom"));
    }
}

#[test]
fn touching_loops_extrude_with_bevels_to_clean_solids() {
    for (name, path) in touching_loop_cases() {
        assert_bevel_extrudes_cleanly(&path, false, 1e-6, &format!("{name}, top bevel"));
        assert_bevel_extrudes_cleanly(&path, true, 1e-6, &format!("{name}, both bevels"));
    }
}

#[test]
fn a_traced_picture_outline_extrudes_with_bevels_to_a_clean_solid() {
    let path = traced_silhouette();
    // the profile's clean-up merges points closer than a clipper unit and a half, which moves a
    // finely traced outline's area by a few millionths of itself
    let fraction = AREA_FRACTION;
    assert_bevel_extrudes_cleanly(&path, false, fraction, "traced silhouette, top bevel");
    assert_bevel_extrudes_cleanly(&path, true, fraction, "traced silhouette, both bevels");
}

#[test]
fn an_outline_both_bevels_inset_away_adds_nothing_where_the_bevels_meet() {
    // A tiny square beside a large one, both bevels 3 mm deep on a 6 mm extrusion: the bevels
    // meet with no straight wall between them, and every inset ring of the tiny square is empty.
    let large = square_at(0.0, 0.0, 20.0, false);
    let tiny = square_at(30.0, 0.0, 0.1, false);
    let path = path_of(&[&large, &tiny]);
    let top = [(3.0, -1.5), (4.5, -3.0)];
    let bottom = [(3.0, -1.5), (1.5, -3.0)];
    let mesh = extrude_bevelled(
        &path,
        6.0,
        Some(&top),
        JoinType::Round,
        Some(&bottom),
        JoinType::Round,
    )
    .expect("the profile cleans up");
    assert_closed_outward(&mesh, "large and tiny squares, meeting bevels");
    let beyond_large = mesh.vertices.iter().filter(|v| v.x > 25.0).count();
    assert_eq!(
        beyond_large, 0,
        "the tiny square left a sheet of no thickness"
    );
}
