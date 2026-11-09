//! What [`remesh_for_analysis`] promises when input is hard: `Ok` means the surface it returns
//! really meets its contract. Each input here was once answered with a surface that did not:
//! walls collapsed onto each other, a deviation bound quietly exceeded, duplicate vertices far
//! from the origin, or a panic on an option.

use std::sync::atomic::{AtomicUsize, Ordering};

use solidmesh_math::Matrix4X4;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{
    remesh_for_analysis, AnalysisRemesh, AnalysisRemeshError, AnalysisRemeshOptions, CancelToken,
    Mesh,
};

use super::curved_parts::{
    bump_plate, components, dense_deviation, hollow_sphere, icosphere, mesh_of, needle_cone,
    triangles_of, P,
};
use super::parts::bracket_holes;
use super::surface_checks::self_intersections;

fn vertices(result: &AnalysisRemesh) -> Vec<P> {
    result.surface.vertices.clone()
}

fn triangles_points(result: &AnalysisRemesh) -> (Vec<[i32; 3]>, Vec<P>) {
    (
        result.surface.faces.clone(),
        result.surface.vertices.clone(),
    )
}

fn triangles(result: &AnalysisRemesh) -> Vec<[P; 3]> {
    result
        .surface
        .faces
        .iter()
        .map(|f| f.map(|v| result.surface.vertices[v as usize]))
        .collect()
}

fn sampled_deviation(result: &AnalysisRemesh) -> f64 {
    result.report.sampled_deviation
}

/// Remeshes a closed spherical shell of `wall` and holds `Ok` to its contract: no crossing
/// triangles, each surface still on its own side of the wall, and a volume change bounded by
/// what `bound` allows.
fn assert_thin_wall(wall: f64, target: f64, bound: f64) {
    let radius = 10.0;
    let part = hollow_sphere(radius, wall, 5);
    let size = move |_: P| target;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, bound))
        .unwrap_or_else(|error| panic!("wall {wall}: {error}"));
    let (faces, verts) = triangles_points(&result);
    let crossings = self_intersections(&verts, &faces);
    assert!(
        crossings.is_empty(),
        "wall {wall}: {} crossing triangle pairs came back as Ok",
        crossings.len()
    );
    let groups = components(verts.len(), &faces);
    assert_eq!(
        groups.len(),
        2,
        "wall {wall}: the two walls must stay separate"
    );
    for (group, centre) in groups.iter().zip([radius, radius - wall]) {
        let (mut low, mut high) = (f64::MAX, 0.0_f64);
        for &i in group {
            let d = verts[i];
            let r = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            low = low.min(r);
            high = high.max(r);
        }
        assert!(
            (low - centre).abs() < wall * 0.5 && (high - centre).abs() < wall * 0.5,
            "wall {wall}: a surface at radius {centre} wandered to {low}..{high}"
        );
    }
    assert!(
        result.report.volume_change_percent.abs() < 100.0 * bound / wall,
        "wall {wall}: volume changed {}%",
        result.report.volume_change_percent
    );
}

#[test]
fn a_hollow_sphere_with_a_tenth_mm_wall_keeps_its_walls_apart() {
    assert_thin_wall(0.1, 4.0, 0.05);
}

#[test]
fn a_hollow_sphere_with_a_fifth_mm_wall_keeps_its_walls_apart() {
    assert_thin_wall(0.2, 4.0, 0.05);
}

#[test]
fn a_hollow_sphere_with_a_half_mm_wall_keeps_its_walls_apart() {
    assert_thin_wall(0.5, 6.0, 0.05);
}

/// Holds the two-way distance to `bound`, measured at many more points than the remesher itself
/// samples.
fn assert_within_bound(part: &Mesh, result: &AnalysisRemesh, bound: f64, what: &str, n: usize) {
    let (input, output) = (triangles_of(part), triangles(result));
    let out_to_in = dense_deviation(&output, &input, n);
    let in_to_out = dense_deviation(&input, &output, 2);
    println!(
        "{what}: triangles {}, bound {bound}, dense out->in {out_to_in:.5}, in->out \
         {in_to_out:.5}, reported {:.5}",
        result.report.triangles,
        sampled_deviation(result)
    );
    assert!(
        out_to_in <= bound,
        "{what}: out->in {out_to_in} past {bound}"
    );
    assert!(
        in_to_out <= bound,
        "{what}: in->out {in_to_out} past {bound}"
    );
    assert!(sampled_deviation(result) <= bound);
}

#[test]
fn the_deviation_bound_holds_on_a_sphere_under_dense_sampling() {
    for (target, bound) in [(4.0, 0.02), (6.0, 0.05), (3.0, 0.01)] {
        let (points, faces) = icosphere(10.0, 5);
        let part = mesh_of(&points, &faces);
        let size = move |_: P| target;
        let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, bound))
            .unwrap_or_else(|error| panic!("sphere target {target} bound {bound}: {error}"));
        assert_within_bound(
            &part,
            &result,
            bound,
            &format!("sphere {target}/{bound}"),
            12,
        );
    }
}

#[test]
fn the_deviation_bound_holds_on_a_bumped_plate_under_dense_sampling() {
    let part = bump_plate();
    for (target, bound) in [(8.0, 0.05), (8.0, 0.15), (12.0, 0.1)] {
        let size = move |_: P| target;
        let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, bound))
            .unwrap_or_else(|error| panic!("bump target {target} bound {bound}: {error}"));
        assert_within_bound(&part, &result, bound, &format!("bump {target}/{bound}"), 10);
    }
}

/// A translated part must come back in `f64` with its vertices distinct: nothing narrows them.
#[test]
fn a_part_away_from_the_origin_keeps_every_vertex_distinct_and_its_shape() {
    let mut part = bracket_holes();
    part.transform(Matrix4X4::create_translation(8000.0, 8000.0, 0.0));
    let size = |_: P| 2.5;
    let mut options = AnalysisRemeshOptions::new(&size, 0.05);
    options.min_angle_floor_degrees = 5.0;
    let result = remesh_for_analysis(&part, &options)
        .unwrap_or_else(|error| panic!("bracket at x=y=8000: {error}"));
    assert_distinct_and_shaped(&result, "bracket at x=y=8000", 5.0);
}

/// At x=y=80000 the bracket's `f32` vertices sit on a grid of about 0.008 mm, comparable to the
/// bound and to the chord lengths around its holes: the bound cannot be held, and that is said by
/// name with the measured value.
#[test]
fn a_part_far_past_f32_precision_is_refused_with_its_measured_deviation() {
    let mut part = bracket_holes();
    part.transform(Matrix4X4::create_translation(80000.0, 80000.0, 0.0));
    let size = |_: P| 2.5;
    match remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05)) {
        Err(AnalysisRemeshError::DeviationNotMet { measured, bound }) => {
            assert_eq!(bound, 0.05);
            assert!(measured > bound, "measured {measured}");
        }
        other => panic!("expected DeviationNotMet, got {other:?}"),
    }
}

/// At the same offset with a bound finer than that grid, the rounded holes cross each other.
#[test]
fn a_bound_finer_than_the_coordinates_is_refused_as_crossing() {
    let mut part = bracket_holes();
    part.transform(Matrix4X4::create_translation(80000.0, 80000.0, 0.0));
    let size = |_: P| 2.5;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.005));
    assert!(
        matches!(result, Err(AnalysisRemeshError::SelfIntersecting { .. })),
        "got {result:?}"
    );
}

fn assert_distinct_and_shaped(result: &AnalysisRemesh, what: &str, min_angle: f64) {
    let verts = vertices(result);
    let mut seen = std::collections::HashSet::new();
    let duplicates = verts
        .iter()
        .filter(|p| !seen.insert((p[0].to_bits(), p[1].to_bits(), p[2].to_bits())))
        .count();
    assert_eq!(duplicates, 0, "{what}: duplicate output vertices");
    println!(
        "{what}: {} triangles, min angle {:.3}, forced {}, unforced small {}, sampled deviation {:.4}",
        result.report.triangles,
        result.report.min_angle_degrees,
        result.report.forced_small_angle_triangles,
        result.report.unforced_small_angle_triangles,
        result.report.sampled_deviation
    );
    assert_eq!(
        result.report.forced_small_angle_triangles, 0,
        "{what}: the bracket has no feature under 5 degrees"
    );
    assert!(
        result.report.min_angle_degrees >= min_angle,
        "{what}: min angle {}",
        result.report.min_angle_degrees
    );
}

/// A bound the size limits forbid reaching must be refused by name with the measured value,
/// never returned quietly past the bound.
#[test]
fn a_bound_that_cannot_be_reached_returns_the_deviation_error() {
    let (points, faces) = icosphere(10.0, 5);
    let part = mesh_of(&points, &faces);
    let size = |_: P| 4.0;
    let mut options = AnalysisRemeshOptions::new(&size, 0.001);
    options.min_edge_length = 3.0;
    match remesh_for_analysis(&part, &options) {
        Err(AnalysisRemeshError::DeviationNotMet { measured, bound }) => {
            assert!(measured > bound, "measured {measured}, bound {bound}");
        }
        other => panic!("expected DeviationNotMet, got {other:?}"),
    }
}

#[test]
fn a_zero_maximum_edge_length_is_refused_by_name() {
    let part = create_cube(10.0, 10.0, 10.0);
    let size = |_: P| 1.0;
    let mut options = AnalysisRemeshOptions::new(&size, 0.05);
    options.max_edge_length = 0.0;
    assert!(matches!(
        remesh_for_analysis(&part, &options),
        Err(AnalysisRemeshError::InvalidOptions(_))
    ));
}

#[test]
fn a_non_finite_vertex_is_refused_by_name() {
    let mut part = create_cube(10.0, 10.0, 10.0);
    part.vertices[0].x = f32::NAN;
    let size = |_: P| 1.0;
    let error = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05))
        .expect_err("a NaN vertex must not remesh");
    assert!(
        matches!(error, AnalysisRemeshError::NonFiniteVertex { index: 0 }),
        "got {error:?}"
    );
}

/// The size callback cancels partway through the passes, so this proves the flag is honoured
/// while the remesh is running rather than only before it starts.
#[test]
fn cancelling_during_the_passes_returns_the_cancel_error() {
    let part = bracket_holes();
    let token = CancelToken::new();
    let calls = AtomicUsize::new(0);
    let cancel_at = 2_000;
    let size = |_: P| {
        if calls.fetch_add(1, Ordering::Relaxed) == cancel_at {
            token.cancel();
        }
        2.5
    };
    let mut options = AnalysisRemeshOptions::new(&size, 0.05);
    options.cancel = Some(&token);
    let result = remesh_for_analysis(&part, &options);
    assert!(
        calls.load(Ordering::Relaxed) > cancel_at,
        "the remesh must run past the cancel point"
    );
    assert!(matches!(result, Err(AnalysisRemeshError::Cancelled)));
}

/// The apex of a long thin cone is a closed fan of about 12 degrees: too narrow for three corners
/// at the 5 degree target but wide enough for three at the 1 degree floor, so no corner there is
/// forced at the floor and the repair opens the fan to it.
#[test]
fn a_needle_cone_apex_is_opened_to_the_default_floor() {
    let part = needle_cone();
    let size = |_: P| 2.0;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05))
        .unwrap_or_else(|error| panic!("needle cone: {error}"));
    println!("needle at the default floor: {:#?}", result.report);
    assert_eq!(result.report.forced_small_angle_triangles, 0);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
    assert!(
        result.report.min_angle_degrees >= 1.0,
        "min angle {}",
        result.report.min_angle_degrees
    );
}
