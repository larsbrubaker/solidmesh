//! Input faces too thin to have a normal, and self-contact the input really has: slivers beside a
//! crease must neither blow the remesh up nor cost a crease, measured against the geometry the
//! test built and not against the crate's own bookkeeping; two solids that touch or overlap in
//! one mesh must be refused, however exactly their faces line up.

use solidmesh_math::Matrix4X4;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{
    remesh_for_analysis, AnalysisRemesh, AnalysisRemeshError, AnalysisRemeshOptions, Mesh,
};

use super::curved_parts::{mesh_of, P};
use super::surface_checks::sharp_edges;

/// A 40 x 40 x 10 box whose top face is a fan around `apex`, with its front edges through
/// (20, 0, z), so an apex near that point makes slivers along the crease.
fn sliver_box(apex: P) -> Mesh {
    let points: Vec<P> = vec![
        [0.0, 0.0, 10.0],
        [20.0, 0.0, 10.0],
        [40.0, 0.0, 10.0],
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [40.0, 0.0, 0.0],
        [40.0, 40.0, 10.0],
        [0.0, 40.0, 10.0],
        [40.0, 40.0, 0.0],
        [0.0, 40.0, 0.0],
        apex,
    ];
    let (a, b, d, e, f, g, h, i, j, k, c) = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10);
    let faces = vec![
        [a, b, c],
        [b, d, c],
        [d, h, c],
        [h, i, c],
        [i, a, c],
        [a, e, f],
        [a, f, b],
        [b, f, g],
        [b, g, d],
        [k, f, e],
        [k, g, f],
        [k, j, g],
        [d, g, j],
        [d, j, h],
        [h, j, k],
        [h, k, i],
        [i, k, e],
        [i, e, a],
    ];
    mesh_of(&points, &faces)
}

/// The same box with one sliver's flank being another sliver.
fn two_sliver_box() -> Mesh {
    let points: Vec<P> = vec![
        [0.0, 0.0, 10.0],
        [20.0, 0.0, 10.0],
        [40.0, 0.0, 10.0],
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [40.0, 0.0, 0.0],
        [40.0, 40.0, 10.0],
        [0.0, 40.0, 10.0],
        [40.0, 40.0, 0.0],
        [0.0, 40.0, 0.0],
        [19.99999, 1e-6, 10.0],
        [19.99998, 2e-6, 10.0],
    ];
    let (a, b, d, e, f, g, h, i, j, k, c1, c2) = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    let faces = vec![
        [a, b, c1],
        [b, d, c1],
        [d, h, c1],
        [h, c2, c1],
        [h, i, c2],
        [i, a, c2],
        [a, c1, c2],
        [a, e, f],
        [a, f, b],
        [b, f, g],
        [b, g, d],
        [k, f, e],
        [k, g, f],
        [k, j, g],
        [d, g, j],
        [d, j, h],
        [h, j, k],
        [h, k, i],
        [i, k, e],
        [i, e, a],
    ];
    mesh_of(&points, &faces)
}

/// The 14 straight crease segments of that box: its 12 edges, the two front ones split at x=20.
fn true_creases() -> Vec<(P, P)> {
    let (x0, x1, x2, y1, z1) = (0.0, 20.0, 40.0, 40.0, 10.0);
    let c = |x: f64, y: f64, z: f64| [x, y, z];
    vec![
        (c(x0, 0.0, z1), c(x1, 0.0, z1)),
        (c(x1, 0.0, z1), c(x2, 0.0, z1)),
        (c(x0, 0.0, 0.0), c(x1, 0.0, 0.0)),
        (c(x1, 0.0, 0.0), c(x2, 0.0, 0.0)),
        (c(x0, y1, z1), c(x2, y1, z1)),
        (c(x0, y1, 0.0), c(x2, y1, 0.0)),
        (c(x0, 0.0, z1), c(x0, y1, z1)),
        (c(x2, 0.0, z1), c(x2, y1, z1)),
        (c(x0, 0.0, 0.0), c(x0, y1, 0.0)),
        (c(x2, 0.0, 0.0), c(x2, y1, 0.0)),
        (c(x0, 0.0, 0.0), c(x0, 0.0, z1)),
        (c(x2, 0.0, 0.0), c(x2, 0.0, z1)),
        (c(x0, y1, 0.0), c(x0, y1, z1)),
        (c(x2, y1, 0.0), c(x2, y1, z1)),
    ]
}

fn point_segment_distance(p: P, a: P, b: P) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let span = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / span).clamp(0.0, 1.0);
    let d = [ap[0] - ab[0] * t, ap[1] - ab[1] * t, ap[2] - ab[2] * t];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// How many of the box's 14 crease segments have every sample within 1e-3 of an output edge
/// that really bends: measured from the returned coordinates against the known box.
fn creases_kept(result: &AnalysisRemesh) -> usize {
    let sharp = sharp_edges(&result.surface.vertices, &result.surface.faces, 30.0);
    true_creases()
        .iter()
        .filter(|&&(a, b)| {
            (0..=24).all(|i| {
                let t = i as f64 / 24.0;
                let p = [
                    a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t,
                ];
                sharp
                    .iter()
                    .any(|&(q, r)| point_segment_distance(p, q, r) <= 1e-3)
            })
        })
        .count()
}

fn remesh(part: &Mesh, target: f64, bound: f64) -> AnalysisRemesh {
    let size = move |_: P| target;
    remesh_for_analysis(part, &AnalysisRemeshOptions::new(&size, bound))
        .unwrap_or_else(|error| panic!("target {target} bound {bound}: {error}"))
}

/// Slivers along a crease near its vertex once made the remesh dice the crease for minutes.
#[test]
fn an_apex_beside_a_crease_vertex_remeshes_about_as_small_as_a_far_apex() {
    let far = remesh(&sliver_box([20.0, 20.0, 10.0]), 5.0, 0.05)
        .report
        .triangles;
    for (dx, dy) in [
        (0.0, 1e-6),
        (0.0, 1e-5),
        (-2.0, 1e-6),
        (-2.0, 1e-5),
        (0.3, 1e-6),
        (0.3, 1e-5),
        (2e-6, 1e-6),
    ] {
        let started = std::time::Instant::now();
        let result = remesh(&sliver_box([20.0 + dx, dy, 10.0]), 5.0, 0.05);
        let elapsed = started.elapsed().as_secs_f64();
        println!(
            "apex {dx} {dy}: {} triangles vs far {far}, {elapsed:.2}s",
            result.report.triangles
        );
        assert!(
            result.report.triangles <= far * 2,
            "apex {dx} {dy}: {} triangles against {far} for the far apex",
            result.report.triangles
        );
        assert!(elapsed < 30.0, "apex {dx} {dy}: took {elapsed}s");
        assert_eq!(creases_kept(&result), 14, "apex {dx} {dy}");
    }
}

#[test]
fn an_apex_near_the_middle_of_a_crease_keeps_all_fourteen_segments() {
    for (target, bound) in [(5.0, 2.0), (5.0, 0.05), (12.0, 2.0)] {
        let result = remesh(&sliver_box([10.0, 1e-6, 10.0]), target, bound);
        assert_eq!(creases_kept(&result), 14, "target {target} bound {bound}");
    }
}

#[test]
fn a_sliver_whose_flank_is_a_sliver_keeps_all_fourteen_segments() {
    assert_eq!(creases_kept(&remesh(&two_sliver_box(), 5.0, 0.02)), 14);
}

/// With the apex just past (20, 0, 10) along the crease, the crease is not the sliver's longest
/// side: the sliver's longest side is the apex-to-corner edge lying beside it.
#[test]
fn a_crease_that_is_not_the_slivers_longest_side_survives() {
    let result = remesh(&sliver_box([20.000002, 1e-6, 10.0]), 5.0, 0.02);
    assert_eq!(creases_kept(&result), 14);
}

fn two_cubes(offset: [f64; 3]) -> Mesh {
    let mut first = create_cube(10.0, 10.0, 10.0);
    let mut second = create_cube(10.0, 10.0, 10.0);
    second.transform(Matrix4X4::create_translation(
        offset[0], offset[1], offset[2],
    ));
    first.vertices.extend(second.vertices.iter().cloned());
    let base = (first.vertices.len() - second.vertices.len()) as i32;
    let faces: Vec<[i32; 3]> = second.faces.iter().map(|f| [f.v0, f.v1, f.v2]).collect();
    for [a, b, c] in faces {
        first
            .faces
            .add(a + base, b + base, c + base, &first.vertices);
    }
    first
}

fn assert_refused_as_crossing(offset: [f64; 3], target: f64, bound: f64) {
    let part = two_cubes(offset);
    let size = move |_: P| target;
    match remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, bound)) {
        Err(AnalysisRemeshError::SelfIntersecting { pairs }) => {
            assert!(pairs > 0, "offset {offset:?}");
            println!("offset {offset:?} target {target}: {pairs} crossing pairs");
        }
        other => panic!("offset {offset:?} target {target}: {other:?}"),
    }
}

/// The two cubes' faces meet exactly along the integer grid, where every sign is a zero.
#[test]
fn two_cubes_overlapping_on_the_integer_grid_are_refused_at_target_100() {
    assert_refused_as_crossing([5.0, 5.0, 5.0], 100.0, 1.0);
}

#[test]
fn two_cubes_overlapping_on_the_integer_grid_are_refused_at_target_12() {
    assert_refused_as_crossing([5.0, 5.0, 5.0], 12.0, 1.0);
}

/// Faces in one plane overlapping is contact like any other.
#[test]
fn two_cubes_with_coplanar_faces_are_refused() {
    assert_refused_as_crossing([5.0, 5.0, 0.0], 100.0, 1.0);
}

/// The cubes meet only at one corner point.
#[test]
fn two_cubes_touching_at_one_corner_are_refused() {
    assert_refused_as_crossing([10.0, 10.0, 10.0], 100.0, 1.0);
}

/// One cube's edge lies exactly along the middle of another's face.
#[test]
fn a_cube_edge_lying_on_a_cube_face_is_refused() {
    assert_refused_as_crossing([10.0, 0.0, 5.0], 100.0, 1.0);
}

/// Separate solids are the control: the exact tests must not turn a gap into a contact.
#[test]
fn two_separate_cubes_are_accepted() {
    let result = remesh(&two_cubes([20.0, 0.0, 0.0]), 100.0, 1.0);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
    assert_eq!(result.report.triangles, 24);
}

/// A remesh that finishes is judged by the hard floor, and the report says what the soft target
/// reached corner by corner.
#[test]
fn the_report_counts_corners_under_each_limit() {
    let part = create_cube(40.0, 40.0, 0.1);
    let size = |_: P| 5.0;
    let options = AnalysisRemeshOptions::new(&size, 0.05);
    assert_eq!(options.min_angle_floor_degrees, 1.0);
    assert_eq!(options.min_angle_target_degrees, 5.0);
    let result =
        remesh_for_analysis(&part, &options).expect("a thin box remeshes at the 1 degree floor");
    let mut counted = [0_usize; 5];
    let surface = &result.surface;
    let corner = |at: P, p: P, q: P| {
        let (u, v) = (
            [p[0] - at[0], p[1] - at[1], p[2] - at[2]],
            [q[0] - at[0], q[1] - at[1], q[2] - at[2]],
        );
        let cross = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
            .sqrt()
            .atan2(u[0] * v[0] + u[1] * v[1] + u[2] * v[2])
            .to_degrees()
    };
    for f in &surface.faces {
        let [a, b, c] = f.map(|v| surface.vertices[v as usize]);
        for angle in [corner(a, b, c), corner(b, c, a), corner(c, a, b)] {
            for (count, limit) in counted.iter_mut().zip([1.0, 2.0, 5.0, 10.0, 20.0]) {
                *count += usize::from(angle < limit);
            }
        }
    }
    assert_eq!(result.report.corners_under_degrees, counted);
    assert_eq!(result.report.corners_under_degrees[0], 0);
    assert!(result.report.min_angle_degrees >= 1.0);
}

/// The dedicated refusal path: no surface of a cube can have every corner above 50 degrees, so
/// a floor that high must end in the named error with the measured angle.
#[test]
fn a_floor_no_surface_can_reach_is_refused_with_the_measured_angle() {
    let part = create_cube(10.0, 10.0, 10.0);
    let size = |_: P| 3.0;
    let mut options = AnalysisRemeshOptions::new(&size, 0.05);
    options.min_angle_floor_degrees = 50.0;
    options.min_angle_target_degrees = 50.0;
    match remesh_for_analysis(&part, &options) {
        Err(AnalysisRemeshError::AngleFloorNotMet {
            triangles,
            worst_degrees,
            floor_degrees,
        }) => {
            println!("cube at a 50 degree floor: {triangles} triangles, worst {worst_degrees}");
            assert!(triangles > 0);
            assert!(worst_degrees < 50.0);
            assert_eq!(floor_degrees, 50.0);
        }
        other => panic!("expected a named angle refusal, got {other:?}"),
    }
}
