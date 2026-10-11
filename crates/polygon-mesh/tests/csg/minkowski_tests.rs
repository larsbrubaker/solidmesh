//! Port of agg-sharp `Agg.PolygonMesh/MinkowskiTests.cs`: the kernel's morphological operations
//! through `csg::minkowski_morph` and `csg::minkowski_processing`, the entry points a uniform
//! fillet and the bevel feature's test oracle both go through.
//!
//! Every volume is bracketed rather than compared to one analytic number: the structuring ball
//! is an inscribed polytope, so it contains the sphere of its own inradius and is contained by
//! the sphere of its radius, and Minkowski sums are monotone in either operand. C#'s async entry
//! points (`MinkowskiSumAsync`, `MinkowskiDifferenceAsync`, `MinkowskiDifferenceWithPathAsync`)
//! are the `*_with_progress` / `*_with_path_and_progress` calls in Rust (see
//! `minkowski_morph.rs`'s header). `dented` is shared with `minkowski_erosion_cancel_tests`, as
//! C#'s `MinkowskiTests.Dented` is.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use polygon_mesh::csg::boolean_processing::do_boolean;
use polygon_mesh::csg::consistent_winding;
use polygon_mesh::csg::mesh_repair_rust::rewind_backward_patches;
use polygon_mesh::csg::minkowski_morph::{
    erosion_tree_runs, minkowski_difference, minkowski_difference_with_path,
    minkowski_difference_with_path_and_progress, minkowski_sum, minkowski_sum_with_progress,
    ErosionPath,
};
use polygon_mesh::csg::minkowski_processing::{sphere_mesh, MinkowskiError};
use polygon_mesh::csg::{BooleanOptions, BooleanReporter, CsgModes};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3, Vector3Float};

/// The cube these tests round, as a centred 10mm box.
const CUBE_SIDE: f64 = 10.0;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

#[test]
fn dilating_a_cube_by_a_ball_rounds_it_and_grows_the_bounds_by_the_diameter() {
    let radius = 1.0;
    let ball = sphere_mesh(radius, 16).unwrap();

    let dilated =
        cleaned(minkowski_sum(&create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE), &ball).unwrap());

    assert!(
        dilated.is_manifold(),
        "a dilated solid is still one closed watertight body"
    );

    let volume = signed_volume(&dilated);
    assert!(
        volume <= rounded_box_volume(CUBE_SIDE, radius),
        "the tessellated ball is inside the true one, so the sum cannot exceed the analytic rounded cube"
    );
    assert!(
        volume >= rounded_box_volume(CUBE_SIDE, in_radius(&ball)),
        "the ball contains the sphere of its own inradius, so the sum contains that rounding"
    );

    // The ball is a subdivided octahedron, so its six poles sit exactly on the axes at the
    // radius - which is why this is an equality and not a bracket.
    let bounds = dilated.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), CUBE_SIDE + (2.0 * radius), 0.0001);
    assert_within(bounds.y_size(), CUBE_SIDE + (2.0 * radius), 0.0001);
    assert_within(bounds.z_size(), CUBE_SIDE + (2.0 * radius), 0.0001);
}

#[test]
fn closing_a_convex_solid_gives_it_back_unchanged() {
    let radius = 1.0;
    let ball = sphere_mesh(radius, 8).unwrap();
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    let dilated = cleaned(minkowski_sum(&cube, &ball).unwrap());
    let closed = cleaned(minkowski_difference(&dilated, &ball).unwrap());

    assert!(closed.is_manifold());

    // Exactly the cube back, not approximately; the tolerance is for the float vertex storage.
    assert_within(
        signed_volume(&closed),
        CUBE_SIDE * CUBE_SIDE * CUBE_SIDE,
        0.01,
    );

    let bounds = closed.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), CUBE_SIDE, 0.0001);
    assert_within(bounds.y_size(), CUBE_SIDE, 0.0001);
    assert_within(bounds.z_size(), CUBE_SIDE, 0.0001);
}

#[test]
fn opening_a_cube_rounds_its_edges_without_moving_its_faces() {
    let radius = 1.0;
    let ball = sphere_mesh(radius, 16).unwrap();
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    let eroded = cleaned(minkowski_difference(&cube, &ball).unwrap());

    // The erosion of a cube by a centred ball is the concentric cube two radii smaller, exactly.
    let eroded_side = CUBE_SIDE - (2.0 * radius);
    assert_within(
        signed_volume(&eroded),
        eroded_side * eroded_side * eroded_side,
        0.01,
    );

    let opened = cleaned(minkowski_sum(&eroded, &ball).unwrap());

    assert!(opened.is_manifold());

    let volume = signed_volume(&opened);
    assert!(
        volume < CUBE_SIDE * CUBE_SIDE * CUBE_SIDE,
        "rounding the edges of a solid can only take material away"
    );
    assert!(
        volume <= rounded_box_volume(eroded_side, radius),
        "the tessellated ball is inside the true one, so the rounding cannot exceed the analytic one"
    );
    assert!(
        volume >= rounded_box_volume(eroded_side, in_radius(&ball)),
        "the ball contains the sphere of its own inradius, so the rounding contains that answer"
    );

    // The faces themselves never moved: an opening is inscribed in what it opened.
    let bounds = opened.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), CUBE_SIDE, 0.0001);
    assert_within(bounds.y_size(), CUBE_SIDE, 0.0001);
    assert_within(bounds.z_size(), CUBE_SIDE, 0.0001);
}

#[test]
fn a_convex_erosion_takes_the_closed_form_and_leaves_its_own_mesh() {
    let ball = sphere_mesh(1.0, 12).unwrap();
    let box_mesh = create_cube(40.0, 20.0, 10.0);

    let (eroded, path) = minkowski_difference_with_path(&box_mesh, &ball).unwrap();
    let eroded = cleaned(eroded);

    assert_eq!(
        path,
        ErosionPath::ClosedForm,
        "a convex solid is exactly what the closed form is for"
    );
    assert_eq!(
        eroded.faces.len(),
        12,
        "the closed form hulls eight corners; the sweep this replaces leaves 36 triangles"
    );

    // 38 x 18 x 8, and exactly: every plane offset and corner solve is whole-number arithmetic.
    assert_within(signed_volume(&eroded), 38.0 * 18.0 * 8.0, 0.0001);

    let bounds = eroded.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 38.0, 0.0001);
    assert_within(bounds.y_size(), 18.0, 0.0001);
    assert_within(bounds.z_size(), 8.0, 0.0001);
}

#[test]
fn the_async_erosion_reports_its_path() {
    let ball = sphere_mesh(1.0, 12).unwrap();
    let box_mesh = create_cube(40.0, 20.0, 10.0);

    let (eroded, path) =
        minkowski_difference_with_path_and_progress(&box_mesh, &ball, None, &CancelToken::new())
            .unwrap();

    assert_eq!(path, ErosionPath::ClosedForm);
    assert_eq!(
        cleaned(eroded).faces.len(),
        12,
        "the result travels with the path, and it is the closed form's mesh"
    );
}

#[test]
fn a_non_convex_erosion_takes_the_tree_and_matches_the_sweeps_volume() {
    let ball = sphere_mesh(1.0, 12).unwrap();
    let l_shape = cleaned(
        do_boolean(
            &create_cube(20.0, 20.0, 20.0),
            Matrix4X4::IDENTITY,
            &create_cube(10.0, 10.0, 10.0),
            Matrix4X4::create_translation(10.0, 10.0, 10.0),
            CsgModes::Subtract,
            &CancelToken::new(),
            &BooleanOptions::default(),
        )
        .unwrap(),
    );

    let tree_runs_before = erosion_tree_runs();
    let (eroded, path) = minkowski_difference_with_path(&l_shape, &ball).unwrap();
    let eroded = cleaned(eroded);

    assert_eq!(
        path,
        ErosionPath::Sweep,
        "the closed form declines a non-convex solid, and the tree reports the sweep's cost regime"
    );
    assert!(
        erosion_tree_runs() > tree_runs_before,
        "a non-convex solid and a convex ball are exactly what the erosion tree takes"
    );

    assert!(
        signed_volume(&eroded) < 5800.0,
        "18^3 is 5832 - anything at or above it is the convex hull of this solid"
    );
    assert_within(signed_volume(&eroded), 5710.55, 0.5);
}

#[test]
fn the_closed_form_beats_the_sweep_on_the_same_mesh() {
    let ball = sphere_mesh(0.5, 8).unwrap();
    let convex = sphere_mesh(10.0, 16).unwrap();
    let dented_ball = dented(sphere_mesh(10.0, 16).unwrap());

    // Warm both paths on the small fixtures, so neither timing below pays for first-use costs
    // the other one has already paid.
    let small_ball = sphere_mesh(0.5, 4).unwrap();
    minkowski_difference(&create_cube(8.0, 8.0, 8.0), &small_ball).unwrap();
    minkowski_difference(&dented(sphere_mesh(4.0, 8).unwrap()), &small_ball).unwrap();

    let swept_watch = Instant::now();
    let swept = minkowski_difference(&dented_ball, &ball).unwrap();
    let swept_ms = swept_watch.elapsed().as_secs_f64() * 1000.0;

    let closed_watch = Instant::now();
    let closed = minkowski_difference(&convex, &ball).unwrap();
    let closed_ms = closed_watch.elapsed().as_secs_f64() * 1000.0;

    assert!(!swept.faces.is_empty());
    assert!(!closed.faces.is_empty());

    assert!(
        closed_ms * 5.0 < swept_ms,
        "closed form {closed_ms:.2} ms against the sweep's {swept_ms:.2} ms on the same mesh"
    );
}

#[test]
fn an_open_mesh_is_refused_the_way_a_boolean_refuses_it() {
    let ball = sphere_mesh(1.0, 8).unwrap();

    let as_solid = minkowski_sum(&open_box(8.0), &ball).unwrap_err();
    assert_import_rejected_not_closed(&as_solid);

    // Both sides are validated: a bad structuring element is as unusable as a bad solid.
    let as_tool = minkowski_difference(
        &create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE),
        &open_box(2.0),
    )
    .unwrap_err();
    assert_import_rejected_not_closed(&as_tool);
}

/// C#'s `Throws<InvalidOperationException>` whose message names `NotClosed`: the status is what
/// tells the user which operand was unusable.
fn assert_import_rejected_not_closed(error: &MinkowskiError) {
    assert!(
        matches!(error, MinkowskiError::ImportRejected { .. }),
        "expected the import's refusal, got {error:?}"
    );
    assert!(
        error.to_string().contains("NotClosed"),
        "the status is what tells the user which operand was unusable: {error}"
    );
}

#[test]
fn a_solid_with_a_patch_wound_backward_dilates_like_the_clean_solid() {
    let ball = sphere_mesh(1.0, 8).unwrap();
    let clean =
        cleaned(minkowski_sum(&create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE), &ball).unwrap());

    let mut patched = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    patched.reverse_face(0, true);
    patched.reverse_face(1, true);
    add_collapsed_sliver(&mut patched);

    let dilated = cleaned(minkowski_sum(&patched, &ball).unwrap());

    assert!(dilated.is_manifold());
    assert_within(signed_volume(&dilated), signed_volume(&clean), 1e-6);
}

#[test]
fn a_soup_solid_with_a_whole_side_wound_backward_dilates_like_the_clean_solid() {
    let ball = sphere_mesh(1.0, 8).unwrap();
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    let clean = cleaned(minkowski_sum(&cube, &ball).unwrap());

    let mut soup = as_soup(&cube);

    // Reverse both triangles of the side the first triangle lies on.
    let side = soup.faces[0].normal;
    for i in 0..soup.faces.len() {
        if (soup.faces[i].normal - side).length() < 1e-6 {
            soup.reverse_face(i, true);
        }
    }

    add_collapsed_sliver(&mut soup);

    let dilated = cleaned(minkowski_sum(&soup, &ball).unwrap());

    assert!(dilated.is_manifold());
    assert_within(signed_volume(&dilated), signed_volume(&clean), 1e-6);
}

#[test]
fn a_soup_solid_dilates_and_erodes_like_the_indexed_solid() {
    let ball = sphere_mesh(1.0, 8).unwrap();
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    let soup = as_soup(&cube);

    let dilated = cleaned(minkowski_sum(&soup, &ball).unwrap());
    assert!(dilated.is_manifold());
    assert_within(
        signed_volume(&dilated),
        signed_volume(&cleaned(minkowski_sum(&cube, &ball).unwrap())),
        1e-6,
    );

    let eroded = cleaned(minkowski_difference(&soup, &ball).unwrap());
    assert!(eroded.is_manifold());
    assert_within(
        signed_volume(&eroded),
        signed_volume(&cleaned(minkowski_difference(&cube, &ball).unwrap())),
        1e-6,
    );
}

#[test]
fn a_backward_side_across_split_seams_is_rewound_after_welding() {
    let mut soup = as_soup(&create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE));

    let side = soup.faces[0].normal;
    let nudge = Vector3Float::new_f32(1e-5, 1e-5, 1e-5);
    for i in 0..soup.faces.len() {
        let face = soup.faces[i];
        if (face.normal - side).length() < 1e-6 {
            // Soup, so these three vertices are this triangle's alone.
            soup.vertices[face.v0 as usize] += nudge;
            soup.vertices[face.v1 as usize] += nudge;
            soup.vertices[face.v2 as usize] += nudge;
            soup.reverse_face(i, true);
        }
    }

    // The branch under test: by exact position there is nothing to flip.
    assert!(consistent_winding::rewind(&soup).is_none());

    let rewound = rewind_backward_patches(&soup).expect(
        "the welded copy joins the backward side to the rest, and then it disagrees with it",
    );
    assert_within(
        signed_volume(&rewound),
        CUBE_SIDE * CUBE_SIDE * CUBE_SIDE,
        0.01,
    );
}

#[test]
fn an_empty_operand_is_refused_rather_than_quietly_doing_nothing() {
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    assert!(matches!(
        minkowski_sum(&cube, &Mesh::new()),
        Err(MinkowskiError::InvalidArgument(_))
    ));
    assert!(matches!(
        minkowski_difference(&Mesh::new(), &cube),
        Err(MinkowskiError::InvalidArgument(_))
    ));
}

#[test]
fn an_operand_with_no_volume_is_refused_rather_than_quietly_returning_the_other_one() {
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    // Closed, 8 vertices and 12 faces, and every one of them degenerate.
    let flat = create_cube(2.0, 2.0, 0.0);

    assert!(matches!(
        minkowski_sum(&cube, &flat),
        Err(MinkowskiError::InvalidArgument(_))
    ));
    assert!(matches!(
        minkowski_difference(&flat, &cube),
        Err(MinkowskiError::InvalidArgument(_))
    ));
}

#[test]
fn a_sphere_refuses_a_radius_or_segment_count_that_cannot_have_been_meant() {
    assert!(matches!(
        sphere_mesh(0.0, 16),
        Err(MinkowskiError::InvalidArgument(_))
    ));
    assert!(matches!(
        sphere_mesh(-1.0, 16),
        Err(MinkowskiError::InvalidArgument(_))
    ));
    assert!(matches!(
        sphere_mesh(1.0, -8),
        Err(MinkowskiError::InvalidArgument(_))
    ));
}

#[test]
fn an_async_sum_reports_a_bar_that_only_rises_and_reaches_one() {
    let ball = sphere_mesh(1.0, 8).unwrap();
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    // Locked because the kernel's callback may arrive on a worker thread.
    let reports: Arc<Mutex<Vec<(f64, String)>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&reports);
    let reporter: BooleanReporter = Arc::new(move |ratio: f64, message: &str| {
        sink.lock().unwrap().push((ratio, message.to_string()));
    });

    let dilated =
        minkowski_sum_with_progress(&cube, &ball, Some(reporter), &CancelToken::new()).unwrap();

    assert!(!dilated.faces.is_empty());
    let reports = reports.lock().unwrap();
    assert!(
        reports.len() > 1,
        "opening the bar is not progress on its own"
    );

    let mut previous = -1.0;
    for (ratio, message) in reports.iter() {
        assert!(
            *ratio >= previous,
            "the bar went backwards, {previous} then {ratio}"
        );
        assert!(*ratio <= 1.0);
        assert_eq!(message, "Dilate: minkowski");
        previous = *ratio;
    }

    assert_within(reports[0].0, 0.0, 1e-9);
    assert_within(reports[reports.len() - 1].0, 1.0, 1e-9);
}

#[test]
fn a_cancelled_convex_erosion_still_throws_rather_than_falling_into_the_sweep() {
    let solid = sphere_mesh(5.0, 16).unwrap();
    let tool = sphere_mesh(0.5, 8).unwrap();

    let source = CancelToken::new();
    source.cancel();

    assert!(matches!(
        minkowski_difference_with_path_and_progress(&solid, &tool, None, &source),
        Err(MinkowskiError::Cancelled)
    ));
}

#[test]
fn instrumenting_an_operation_does_not_change_its_geometry() {
    let ball = sphere_mesh(1.0, 8).unwrap();
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    let plain = minkowski_sum(&cube, &ball).unwrap();

    let source = CancelToken::new();
    let reporter: BooleanReporter = Arc::new(|_ratio: f64, _message: &str| {});
    let watched = minkowski_sum_with_progress(&cube, &ball, Some(reporter), &source).unwrap();

    assert_eq!(watched.vertices.len(), plain.vertices.len());
    assert_eq!(watched.faces.len(), plain.faces.len());

    for i in 0..plain.vertices.len() {
        // Exact, not within: the claim is that no bit moved.
        assert_eq!(watched.vertices[i].x, plain.vertices[i].x);
        assert_eq!(watched.vertices[i].y, plain.vertices[i].y);
        assert_eq!(watched.vertices[i].z, plain.vertices[i].z);
    }

    for i in 0..plain.faces.len() {
        assert_eq!(watched.faces[i].v0, plain.faces[i].v0);
        assert_eq!(watched.faces[i].v1, plain.faces[i].v1);
        assert_eq!(watched.faces[i].v2, plain.faces[i].v2);
    }
}

#[test]
fn the_async_path_refuses_the_same_operands_the_sync_path_does() {
    let cube = create_cube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    let flat = create_cube(2.0, 2.0, 0.0);

    assert!(matches!(
        minkowski_sum_with_progress(&cube, &Mesh::new(), None, &CancelToken::new()),
        Err(MinkowskiError::InvalidArgument(_))
    ));
    assert!(matches!(
        minkowski_difference_with_path_and_progress(&cube, &flat, None, &CancelToken::new()),
        Err(MinkowskiError::InvalidArgument(_))
    ));
}

/// The same solid with every triangle given its own three vertices.
fn as_soup(mesh: &Mesh) -> Mesh {
    let mut soup = Mesh::new();
    for face in mesh.faces.iter() {
        let first = soup.vertices.len() as i32;
        soup.vertices.push(mesh.vertices[face.v0 as usize]);
        soup.vertices.push(mesh.vertices[face.v1 as usize]);
        soup.vertices.push(mesh.vertices[face.v2 as usize]);
        let Mesh {
            vertices, faces, ..
        } = &mut soup;
        faces.add(first, first + 1, first + 2, vertices);
    }
    soup
}

/// Adds a zero-area triangle (a, b, a) across two opposite corners of a centred cube. Its edge
/// a-b is shared by no other triangle - the shape that must not read as a winding contradiction.
fn add_collapsed_sliver(mesh: &mut Mesh) {
    let a = 0usize;
    let opposite = -mesh.vertices[a];
    let b = mesh
        .vertices
        .iter()
        .position(|v| (*v - opposite).length() < 1e-6)
        .expect("a centred cube has the opposite corner");

    mesh.faces
        .add_with_normal(a as i32, b as i32, a as i32, Vector3Float::UNIT_Z);
}

/// What a boolean result gets before anyone looks at its topology: the kernel's export splits
/// vertices at run boundaries, so `is_manifold` is only meaningful after the merge.
fn cleaned(mut mesh: Mesh) -> Mesh {
    mesh.clean_and_merge();
    mesh.remove_unused_vertices();
    mesh
}

/// The signed volume of a closed mesh, positive when its faces wind outward (the
/// divergence-theorem sum of each triangle's tetrahedron with the origin).
pub(crate) fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;
    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);
        total += a.dot(b.cross(c)) / 6.0;
    }
    total
}

/// The largest sphere centred on the origin that fits inside a convex mesh, measured as the
/// nearest face plane.
fn in_radius(mesh: &Mesh) -> f64 {
    let mut nearest = f64::MAX;
    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        let normal = (b - a).cross(c - a);
        let length = normal.length();
        if length <= 0.0 {
            // A degenerate triangle has no plane to measure to.
            continue;
        }

        nearest = nearest.min(a.dot(normal / length).abs());
    }
    nearest
}

/// Steiner's formula for a cube of `side` rolled off by a ball of `radius`.
fn rounded_box_volume(side: f64, radius: f64) -> f64 {
    (side * side * side)
        + (6.0 * side * side * radius)
        + (12.0 * side * std::f64::consts::PI * radius * radius / 4.0)
        + (4.0 / 3.0 * std::f64::consts::PI * radius * radius * radius)
}

/// The same ball with one vertex pulled halfway in - still closed, still the same triangle
/// count, no longer convex.
pub(crate) fn dented(mut ball: Mesh) -> Mesh {
    ball.vertices[0] *= 0.5;
    ball
}

/// A box missing its top face - not closed, so even the robust import rejects it (NotClosed).
fn open_box(size: f64) -> Mesh {
    let mut mesh = create_cube(size, size, size);
    // Drop the two triangles of one face; any hole makes the surface open.
    mesh.faces.pop();
    mesh.faces.pop();
    mesh
}
