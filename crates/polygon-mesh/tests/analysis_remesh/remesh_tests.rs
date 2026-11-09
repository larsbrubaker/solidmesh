//! Tests of [`remesh_for_analysis`] on real boolean parts: the surface a tetrahedral mesher will
//! take as exact constraints must be well shaped, closed, on the input surface, with its sharp
//! creases intact, or the analysis built on it is wrong.

use super::parts::{bracket_holes, plate_hole_grid};
use super::surface_checks::{lost_sharp_edges, self_intersections, soup_of};
use solidmesh_mesh::{
    remesh_for_analysis, AnalysisRemesh, AnalysisRemeshError, AnalysisRemeshOptions, CancelToken,
    Mesh,
};

/// Far enough to leave a 48-gon hole's rim a real chord, close enough that the rim stays round.
const MAX_DEVIATION: f64 = 0.05;

fn remesh(mesh: &Mesh, target: f64) -> AnalysisRemesh {
    let size = move |_: [f64; 3]| target;
    let options = AnalysisRemeshOptions::new(&size, MAX_DEVIATION);
    remesh_for_analysis(mesh, &options).expect("remesh succeeds")
}

fn assert_good_surface(input: &Mesh, result: &AnalysisRemesh, what: &str) {
    let report = &result.report;
    println!("{what}: {report:#?}");
    assert_eq!(
        lost_sharp_edges(
            &soup_of(input),
            &(
                result.surface.vertices.clone(),
                result.surface.faces.clone()
            ),
            30.0,
            MAX_DEVIATION + 1e-3
        ),
        0,
        "{what}: a sharp input edge has no sharp output edge along it"
    );
    assert!(
        result.surface.to_mesh().is_closed_consistently_wound(),
        "{what}: output must be closed and consistently wound"
    );
    assert!(
        report.min_unforced_angle_degrees >= 5.0,
        "{what}: min unforced angle {} below 5 degrees",
        report.min_unforced_angle_degrees
    );
    assert_eq!(
        report.unforced_small_angle_triangles, 0,
        "{what}: {} triangles under the floor with no feature forcing them",
        report.unforced_small_angle_triangles
    );
    assert_eq!(report.sharp_edges_lost, 0, "{what}: sharp edges lost");
    assert!(
        report.sampled_deviation <= MAX_DEVIATION,
        "{what}: deviation {} past bound",
        report.sampled_deviation
    );
    let (out_v, out_f) = (&result.surface.vertices, &result.surface.faces);
    let crossings = self_intersections(out_v, out_f);
    assert!(
        crossings.is_empty(),
        "{what}: {} self-intersecting triangle pairs, e.g. {:?}",
        crossings.len(),
        &crossings[..crossings.len().min(5)]
    );
}

#[test]
fn bracket_holes_remeshes_to_well_shaped_closed_surface() {
    let part = bracket_holes();
    let result = remesh(&part, 2.5);
    assert_good_surface(&part, &result, "bracket-holes");
    assert!(result.report.sharp_edges_kept > 0);
}

#[test]
fn plate_hole_grid_remeshes_to_well_shaped_closed_surface_keeping_volume() {
    let part = plate_hole_grid();
    let result = remesh(&part, 2.0);
    assert_good_surface(&part, &result, "plate-hole-grid");
    assert!(
        result.report.volume_change_percent.abs() < 0.5,
        "plate-hole-grid volume changed {}%",
        result.report.volume_change_percent
    );
}

#[test]
fn two_runs_give_identical_bits() {
    let part = bracket_holes();
    let first = remesh(&part, 2.5);
    let second = remesh(&part, 2.5);
    assert_eq!(first.report, second.report);
    assert_eq!(first.surface.vertices.len(), second.surface.vertices.len());
    for (a, b) in first.surface.vertices.iter().zip(&second.surface.vertices) {
        assert_eq!(
            (a[0].to_bits(), a[1].to_bits(), a[2].to_bits()),
            (b[0].to_bits(), b[1].to_bits(), b[2].to_bits())
        );
    }
    assert_eq!(first.surface.faces.len(), second.surface.faces.len());
    for t in 0..first.surface.faces.len() {
        let (a, b) = (&first.surface.faces[t], &second.surface.faces[t]);
        assert_eq!(*a, *b);
    }
}

#[test]
fn a_cancelled_token_returns_the_cancel_error() {
    let part = bracket_holes();
    let size = |_: [f64; 3]| 2.5;
    let token = CancelToken::new();
    token.cancel();
    let mut options = AnalysisRemeshOptions::new(&size, MAX_DEVIATION);
    options.cancel = Some(&token);
    assert_eq!(
        remesh_for_analysis(&part, &options).map(|_| ()),
        Err(AnalysisRemeshError::Cancelled)
    );
}

#[test]
fn open_input_is_refused_by_name() {
    let mut open = Mesh::new();
    for p in [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)] {
        open.vertices
            .push(solidmesh_math::Vector3Float::new(p.0, p.1, p.2));
    }
    open.faces.add(0, 1, 2, &open.vertices);
    let size = |_: [f64; 3]| 1.0;
    let result = remesh_for_analysis(&open, &AnalysisRemeshOptions::new(&size, 0.05));
    assert!(matches!(
        result,
        Err(AnalysisRemeshError::OpenInput { boundary_edges: 3 })
    ));
}

#[test]
fn non_manifold_input_is_refused_by_name() {
    // Three triangles fanned on one edge: the edge has three faces.
    let mut fan = Mesh::new();
    for p in [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
    ] {
        fan.vertices
            .push(solidmesh_math::Vector3Float::new(p.0, p.1, p.2));
    }
    fan.faces.add(0, 1, 2, &fan.vertices);
    fan.faces.add(1, 0, 3, &fan.vertices);
    fan.faces.add(0, 1, 4, &fan.vertices);
    let size = |_: [f64; 3]| 1.0;
    let result = remesh_for_analysis(&fan, &AnalysisRemeshOptions::new(&size, 0.05));
    assert!(matches!(
        result,
        Err(AnalysisRemeshError::NonManifoldInput { .. })
    ));
}

#[test]
fn a_bad_size_is_refused_with_where_it_happened() {
    let part = bracket_holes();
    let size = |_: [f64; 3]| -1.0;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05));
    assert!(matches!(
        result,
        Err(AnalysisRemeshError::InvalidTargetSize { value, .. }) if value == -1.0
    ));
}
