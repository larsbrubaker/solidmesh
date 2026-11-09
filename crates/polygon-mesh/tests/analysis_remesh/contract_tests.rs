//! What `Ok` means for the angle floor, pinned input by input: a corner under the floor is only
//! acceptable where the input's own geometry forces it, ordinary parts come back at the floor in
//! any pose, and verification costs memory in proportion to the triangles, not to how much bigger
//! the largest triangle is than the mean.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use solidmesh_math::{Matrix4X4, Vector2};
use solidmesh_mesh::processors::outline_to_mesh::extrude_contours;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{
    remesh_for_analysis, AnalysisRemesh, AnalysisRemeshError, AnalysisRemeshOptions, Mesh,
};

use super::curved_parts::{cone, icosphere, mesh_of, needle_cone, P};
use super::parts::{bracket_holes, plate_hole_grid};
use super::surface_checks::{lost_sharp_edges, self_intersections, soup_of};

/// Counts live and peak bytes for the memory-growth test; nextest runs each test in its own
/// process, so the peak is that test's alone.
struct CountingAllocator;

static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

// SAFETY: forwards every call to the system allocator unchanged and only updates counters.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let block = System.alloc(layout);
        if !block.is_null() {
            let now = LIVE.fetch_add(layout.size(), Ordering::Relaxed) + layout.size();
            PEAK.fetch_max(now, Ordering::Relaxed);
        }
        block
    }

    unsafe fn dealloc(&self, block: *mut u8, layout: Layout) {
        System.dealloc(block, layout);
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const FLOOR: f64 = 5.0;

/// These parts are held to a 5 degree hard floor, stricter than the default, to prove the repair
/// reaches the target it aims for.
fn remeshed(part: &Mesh, target: f64, bound: f64, what: &str) -> AnalysisRemesh {
    let size = move |_: P| target;
    let mut options = AnalysisRemeshOptions::new(&size, bound);
    options.min_angle_floor_degrees = 5.0;
    remesh_for_analysis(part, &options).unwrap_or_else(|error| panic!("{what}: {error}"))
}

fn assert_at_floor(result: &AnalysisRemesh, what: &str) {
    println!(
        "{what}: {} triangles, min {:.3}, forced {}, unforced {}, deviation {:.4}",
        result.report.triangles,
        result.report.min_angle_degrees,
        result.report.forced_small_angle_triangles,
        result.report.unforced_small_angle_triangles,
        result.report.sampled_deviation,
    );
    assert!(
        result.report.min_angle_degrees >= FLOOR,
        "{what}: min angle {}",
        result.report.min_angle_degrees
    );
    assert_eq!(result.report.forced_small_angle_triangles, 0, "{what}");
    assert_eq!(result.report.unforced_small_angle_triangles, 0, "{what}");
}

/// The rotated pose of an ordinary part must not change whether it remeshes: nothing about a
/// bracket depends on which way it faces.
fn assert_bracket_rotation_meets_the_floor(degrees: f64) {
    let mut part = bracket_holes();
    part.transform(Matrix4X4::create_rotation_z(degrees.to_radians()));
    assert_at_floor(
        &remeshed(&part, 2.5, 0.05, &format!("bracket rotated {degrees}")),
        &format!("bracket rotated {degrees}"),
    );
}

#[test]
fn a_bracket_rotated_7_degrees_meets_the_floor() {
    assert_bracket_rotation_meets_the_floor(7.0);
}

#[test]
fn a_bracket_rotated_17_degrees_meets_the_floor() {
    assert_bracket_rotation_meets_the_floor(17.0);
}

#[test]
fn a_bracket_rotated_33_degrees_meets_the_floor() {
    assert_bracket_rotation_meets_the_floor(33.0);
}

#[test]
fn a_bracket_rotated_53_degrees_meets_the_floor() {
    assert_bracket_rotation_meets_the_floor(53.0);
}

#[test]
fn a_bracket_rotated_71_degrees_meets_the_floor() {
    assert_bracket_rotation_meets_the_floor(71.0);
}

#[test]
fn a_bracket_tilted_on_two_axes_meets_the_floor() {
    let mut part = bracket_holes();
    part.transform(Matrix4X4::create_rotation_z(0.7));
    part.transform(Matrix4X4::create_rotation_x(0.45));
    assert_at_floor(
        &remeshed(&part, 2.5, 0.05, "tilted bracket"),
        "tilted bracket",
    );
}

fn plate_at(x: f64) -> AnalysisRemesh {
    let mut part = plate_hole_grid();
    part.transform(Matrix4X4::create_translation(x, 0.0, 0.0));
    remeshed(&part, 2.0, 0.05, &format!("plate at x={x}"))
}

#[test]
fn a_plate_at_x_1000_meets_the_floor() {
    assert_at_floor(&plate_at(1000.0), "plate at x=1000");
}

#[test]
fn a_plate_at_x_4000_meets_the_floor() {
    assert_at_floor(&plate_at(4000.0), "plate at x=4000");
}

#[test]
fn a_bracket_with_a_tight_bound_meets_the_floor() {
    let part = bracket_holes();
    assert_at_floor(
        &remeshed(&part, 2.5, 0.005, "bracket bound 0.005"),
        "bound 0.005",
    );
}

/// Every apex triangle is a true feature: the cone's fan totals about 12 degrees, too little for
/// any closed fan to give three corners of 5 degrees, so no triangulation could do better.
#[test]
fn a_needle_cone_apex_is_forced_by_its_own_geometry() {
    let part = needle_cone();
    let result = remeshed(&part, 2.0, 0.05, "needle cone");
    println!("needle: {:#?}", result.report);
    assert_eq!(result.report.forced_small_angle_triangles, 24);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
    assert!(
        (result.report.min_angle_degrees - 0.4986).abs() < 0.02,
        "min angle {}",
        result.report.min_angle_degrees
    );
}

/// A 100 long, 16 sided cone whose tip fan totals 5.96 degrees: three corners of a closed fan
/// that wide could have about 2 degrees each, so the input forces none of the 0.373 degree
/// corners it arrives with and `Ok` may not report them as forced.
#[test]
fn a_cone_tip_of_six_degrees_is_not_exempt_from_the_default_floor() {
    let part = cone(1.667, 100.0, 16);
    let size = |_: P| 5.0;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05))
        .unwrap_or_else(|error| panic!("6 degree cone: {error}"));
    println!("6 degree cone: {:#?}", result.report);
    assert_eq!(result.report.forced_small_angle_triangles, 0);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
    assert!(
        result.report.min_angle_degrees >= 1.0,
        "min angle {}",
        result.report.min_angle_degrees
    );
}

/// A cone tip whose whole fan totals about 2.4 degrees: no closed fan of it can give three
/// corners of 1 degree, so the fan is a true feature and its corners are reported as forced.
#[test]
fn a_cone_tip_under_three_floors_is_forced_at_the_default_floor() {
    let part = cone(0.2, 30.0, 20);
    let size = |_: P| 2.0;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05))
        .unwrap_or_else(|error| panic!("2.4 degree cone: {error}"));
    println!("2.4 degree cone: {:#?}", result.report);
    // The fan is coarsened to the three corners a closed fan needs, and the input forces them.
    assert_eq!(result.report.forced_small_angle_triangles, 3);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
    assert!(
        result.report.min_angle_degrees < 1.0,
        "min angle {}",
        result.report.min_angle_degrees
    );
}

/// A 2 degree prism tip: the cap corner between two sharp edges measures 2 degrees in the input.
#[test]
fn a_two_degree_wedge_tip_is_forced_on_each_cap_only() {
    let outline = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(30.0, 0.0),
        Vector2::new(30.0, 30.0 * 2.0_f64.to_radians().tan()),
    ];
    let part = extrude_contours(&[outline], 10.0).expect("extruded the wedge");
    let result = remeshed(&part, 2.0, 0.05, "wedge");
    println!("wedge: {:#?}", result.report);
    assert_eq!(result.report.forced_small_angle_triangles, 2);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
}

fn lens(chord: f64, height: f64, thickness: f64, samples: usize) -> Mesh {
    let radius = (chord * chord / 4.0 + height * height) / (2.0 * height);
    let half = (chord / 2.0 / radius).asin();
    let arc = |s: f64| {
        Vector2::new(
            radius * (half * s).sin(),
            radius * (half * s).cos() - (radius - height),
        )
    };
    let mut outline: Vec<Vector2> = (0..samples)
        .map(|i| arc(1.0 - 2.0 * i as f64 / samples as f64))
        .collect();
    outline.extend((0..samples).map(|i| {
        let p = arc(-1.0 + 2.0 * i as f64 / samples as f64);
        Vector2::new(p.x, -p.y)
    }));
    extrude_contours(&[outline], thickness).expect("extruded the lens")
}

/// The tip of this lens measures about 5.27 degrees in the input, above the floor: a coarsened
/// chain whose chord reads narrower is the remesh's doing and must not count as forced.
#[test]
fn a_lens_tip_above_the_floor_stays_above_the_floor() {
    let part = lens(30.0, 0.35, 5.0, 120);
    let result = remeshed(&part, 12.0, 0.1, "lens");
    assert_at_floor(&result, "lens");
}

/// A lens tip of about 5.04 degrees in the input, at a 5 degree hard floor: the chain chords read
/// narrower than the input's own angle, so the corner is the remesh's doing. It must not pass as
/// forced; at this size the repair cannot open it back up, so the caller hears about it by name.
#[test]
fn a_lens_tip_just_above_the_floor_is_not_narrowed_and_called_forced() {
    let part = lens(30.0, 0.33, 5.0, 120);
    let size = |_: P| 12.0;
    let mut options = AnalysisRemeshOptions::new(&size, 0.1);
    options.min_angle_floor_degrees = 5.0;
    match remesh_for_analysis(&part, &options) {
        Err(AnalysisRemeshError::AngleFloorNotMet { worst_degrees, .. }) => {
            assert!(worst_degrees < 5.0);
        }
        other => panic!("expected the named refusal, got {other:?}"),
    }
}

/// At the default floor the same tip is well inside the promise, with nothing claimed as forced.
#[test]
fn a_lens_tip_above_the_floor_needs_no_forced_corner_at_the_default_floor() {
    let part = lens(30.0, 0.33, 5.0, 120);
    let size = |_: P| 12.0;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.1))
        .unwrap_or_else(|error| panic!("lens 5.04: {error}"));
    assert_eq!(result.report.forced_small_angle_triangles, 0);
    assert!(result.report.min_angle_degrees >= 1.0);
}

/// A thin box at target 5 is thin-walled but has square corners everywhere: at 0.2 thick its
/// result keeps the floor like any other box.
#[test]
fn a_thin_box_two_tenths_thick_meets_the_floor() {
    let part = create_cube(40.0, 40.0, 0.2);
    assert_at_floor(&remeshed(&part, 5.0, 0.05, "thin box 0.2"), "thin box 0.2");
}

/// A tenth-millimetre wall at target 5 stays within the default 1 degree floor: the repair aims
/// at the 5 degree target and the report says how close it came.
#[test]
fn a_thin_box_a_tenth_thick_is_ok_at_the_default_floor() {
    let part = create_cube(40.0, 40.0, 0.1);
    let size = |_: P| 5.0;
    let result = remesh_for_analysis(&part, &AnalysisRemeshOptions::new(&size, 0.05))
        .unwrap_or_else(|error| panic!("thin box 0.1: {error}"));
    assert!(result.report.min_angle_degrees >= 1.0);
    assert_eq!(result.report.unforced_small_angle_triangles, 0);
}

/// A sliver on a real crease must not hide it: the crease survives both in the report and in
/// the returned coordinates.
#[test]
fn a_sliver_on_a_real_crease_keeps_the_crease() {
    let mesh = sliver_box(1e-5, 40.0);
    let result = remeshed(&mesh, 5.0, 0.05, "sliver box");
    let output = (
        result.surface.vertices.clone(),
        result.surface.faces.clone(),
    );
    assert_eq!(lost_sharp_edges(&soup_of(&mesh), &output, 30.0, 0.051), 0);
    assert_at_floor(&result, "sliver box");
}

fn sliver_box(offset: f64, z: f64) -> Mesh {
    let points: Vec<P> = vec![
        [0.0, 0.0, z],
        [40.0, 0.0, z],
        [40.0, 40.0, z],
        [0.0, 40.0, z],
        [0.0, 0.0, 0.0],
        [40.0, 0.0, 0.0],
        [40.0, 40.0, 0.0],
        [0.0, 40.0, 0.0],
        [20.0, offset, z],
    ];
    let (a, b, c, d, e, f, g, h, m) = (0, 1, 2, 3, 4, 5, 6, 7, 8);
    let faces = vec![
        [a, b, m],
        [a, m, d],
        [m, b, c],
        [m, c, d],
        [e, g, f],
        [e, h, g],
        [a, e, f],
        [a, f, b],
        [b, f, g],
        [b, g, c],
        [c, g, h],
        [c, h, d],
        [d, h, e],
        [d, e, a],
    ];
    mesh_of(&points, &faces)
}

/// A fine sphere beside a coarse rotated cube: the same triangle counts at side 1 and side 8,
/// so peak memory must not grow with the size ratio of the largest triangle to the mean.
#[test]
fn verification_memory_does_not_grow_with_the_triangle_size_ratio() {
    let peak_at = |side: f64| {
        let part = sphere_and_cube(side);
        // The pass count of zero and the wide bound keep this about verification memory, not
        // about the remesh passes.
        let size = |_: P| 1000.0;
        let mut options = AnalysisRemeshOptions::new(&size, 1.0);
        options.max_passes = 0;
        let before = LIVE.load(Ordering::Relaxed);
        PEAK.store(before, Ordering::Relaxed);
        let result = remesh_for_analysis(&part, &options);
        let peak = PEAK.load(Ordering::Relaxed);
        assert!(result.is_ok(), "side {side}: {result:?}");
        peak - before
    };
    let small = peak_at(1.0);
    let large = peak_at(8.0);
    println!("verification peak bytes: side 1 {small}, side 8 {large}");
    assert!(
        (large as f64) < (small as f64) * 1.25,
        "peak grew from {small} to {large} bytes with only the cube's size changing"
    );
}

fn sphere_and_cube(side: f64) -> Mesh {
    let (mut points, mut faces) = icosphere(1.0, 5);
    let mut cube = create_cube(side, side, side);
    cube.transform(Matrix4X4::create_rotation_z(0.7));
    cube.transform(Matrix4X4::create_rotation_x(0.6));
    cube.transform(Matrix4X4::create_translation(30.0, 0.0, 0.0));
    let (cube_points, cube_faces) = soup_of(&cube);
    let offset = points.len();
    points.extend(cube_points);
    faces.extend(cube_faces.iter().map(|f| f.map(|v| offset + v as usize)));
    mesh_of(&points, &faces)
}

/// A crossing is never refused because of rounding: a rotated cube's flat faces hold coplanar
/// pairs whose determinants are all noise.
#[test]
fn a_rotated_cube_is_not_refused_for_phantom_crossings() {
    let mut cube = create_cube(100.0, 100.0, 100.0);
    cube.transform(Matrix4X4::create_rotation_z(0.3));
    let result = remeshed(&cube, 12.5, 1.0, "rotated cube");
    assert_eq!(
        self_intersections(&result.surface.vertices, &result.surface.faces).len(),
        0
    );
    assert_at_floor(&result, "rotated cube");
}
