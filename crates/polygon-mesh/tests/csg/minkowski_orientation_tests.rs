//! Dilate and Erode (`csg::minkowski_morph`) read a part saved inside out as the solid it looks
//! like, while a hollow part's correctly wound cavity stays a cavity
//! (`csg::minkowski_shell_orientation`).
//!
//! Thingi10K 95430 is such a part: without the orientation repair its Dilate came back at 0.947
//! from a 7.156 part, because the kernel read the inverted shell as a void and dilated that. The
//! sum with progress is `minkowski_sum_with_progress`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};

use manifold_rust::linalg::Vec3;
use manifold_rust::manifold::Manifold;
use manifold_rust::progress::ProgressReporter;
use polygon_mesh::csg::manifold_import::import;
use polygon_mesh::csg::manifold_result_reader::to_mesh;
use polygon_mesh::csg::minkowski_morph::{
    minkowski_difference, minkowski_sum, minkowski_sum_with_progress,
};
use polygon_mesh::csg::minkowski_processing::{orient_shells_as_solid, sphere_mesh};
use polygon_mesh::csg::minkowski_progress_model::{
    tree_hull_units, TREE_LEAF_SIZE, TREE_LEAF_TIME_SHARE,
};
use polygon_mesh::csg::minkowski_shell_orientation::may_have_inverted_shell;
use polygon_mesh::csg::BooleanReporter;
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Matrix4X4;

use super::minkowski_tests::signed_volume;

const OUTER_SIDE: f64 = 10.0;

const CAVITY_SIDE: f64 = 4.0;

/// The hollow box's Dilate volume by `ball` as measured before the repair existed, pinned so
/// the repair provably leaves a correctly wound cavity alone.
const HOLLOW_DILATED_VOLUME: f64 = 1679.7955980304591;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

#[test]
fn an_inside_out_cube_dilates_and_erodes_as_the_cube_it_looks_like() {
    let mut inside_out = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    inside_out.reverse_faces();

    let dilated = minkowski_sum(&inside_out, &ball()).unwrap();
    let eroded = minkowski_difference(&inside_out, &ball()).unwrap();

    let cube = OUTER_SIDE * OUTER_SIDE * OUTER_SIDE;
    assert!(
        signed_volume(&dilated) > cube,
        "a Dilate never shrinks a part; the inverted cube is still a cube to the user"
    );
    let correct_eroded =
        minkowski_difference(&create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE), &ball()).unwrap();
    assert_within(signed_volume(&eroded), signed_volume(&correct_eroded), 1e-6);
    assert!(signed_volume(&eroded) > 0.0);
}

#[test]
fn an_inside_out_tool_works_like_the_correct_tool() {
    let cube = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    let mut inside_out_ball = ball();
    inside_out_ball.reverse_faces();

    assert_within(
        signed_volume(&minkowski_sum(&cube, &inside_out_ball).unwrap()),
        signed_volume(&minkowski_sum(&cube, &ball()).unwrap()),
        1e-6,
    );
}

#[test]
fn only_the_inside_out_one_of_two_separate_solids_is_rewound() {
    let mut mixed = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    let mut inside_out = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    inside_out.reverse_faces();
    mixed.copy_all_faces(&inside_out, Matrix4X4::create_translation(20.0, 0.0, 0.0));

    let mut correct = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    correct.copy_all_faces(
        &create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE),
        Matrix4X4::create_translation(20.0, 0.0, 0.0),
    );

    assert_within(
        signed_volume(&minkowski_sum(&mixed, &ball()).unwrap()),
        signed_volume(&minkowski_sum(&correct, &ball()).unwrap()),
        1e-6,
    );
}

#[test]
fn an_entirely_inside_out_three_level_nest_is_treated_as_the_nest() {
    let mut inside_out = three_level_nest();
    inside_out.reverse_faces();

    // Relative 1e-8 rather than bit-equal: rewound triangles list their corners in another
    // order, so the kernel's sums round differently (4e-6 of 9041 measured).
    assert_within(
        signed_volume(&minkowski_sum(&inside_out, &ball()).unwrap()),
        signed_volume(&minkowski_sum(&three_level_nest(), &ball()).unwrap()),
        1e-4,
    );
    assert_within(
        signed_volume(&minkowski_difference(&inside_out, &ball()).unwrap()),
        signed_volume(&minkowski_difference(&three_level_nest(), &ball()).unwrap()),
        1e-4,
    );
}

/// The repair is skipped exactly when no shell is inverted, which is the case in which it
/// cannot change anything.
#[test]
fn the_repair_runs_only_when_a_shell_is_inverted() {
    let mut inside_out = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    inside_out.reverse_faces();

    assert!(!may_have_inverted_shell(
        &import(&create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE), false).unwrap()
    ));
    assert!(may_have_inverted_shell(
        &import(&inside_out, false).unwrap()
    ));
    assert!(
        may_have_inverted_shell(&import(&hollow_box(), false).unwrap()),
        "a cavity winds inward; the repair has to see it to judge its nesting"
    );
}

/// A patched dilation's bar maps the kernel's units through the tree's real hull count: every
/// leaf-stage report is a whole number of the kernel's leaf-stage units.
#[test]
fn a_patched_dilations_bar_follows_the_kernels_hull_count() {
    let solid = Manifold::cube(Vec3::new(20.0, 20.0, 20.0), true)
        - Manifold::cube(Vec3::new(10.0, 10.0, 10.0), false)
        - Manifold::sphere(6.0, 24).translate(Vec3::new(-10.0, 0.0, 0.0));
    // The callback reads the reporter it belongs to, through a weak handle set once the reporter
    // exists (the reporter cannot hold a strong handle to itself).
    let phase_total = Arc::new(AtomicU64::new(0));
    let reporter_cell: Arc<OnceLock<Weak<ProgressReporter>>> = Arc::new(OnceLock::new());
    let kernel_reporter = {
        let phase_total = Arc::clone(&phase_total);
        let reporter_cell = Arc::clone(&reporter_cell);
        Arc::new(ProgressReporter::new(move |_phase, _fraction| {
            if let Some(reporter) = reporter_cell.get().and_then(Weak::upgrade) {
                phase_total.store(reporter.phase_total(), Ordering::SeqCst);
            }
        }))
    };
    reporter_cell
        .set(Arc::downgrade(&kernel_reporter))
        .expect("set once");
    assert!(solid
        .try_dilate_by_convex(&Manifold::sphere(1.0, 8), None, Some(&kernel_reporter))
        .is_some());
    let hulls = tree_hull_units(phase_total.load(Ordering::SeqCst) as i64, false);
    assert!(hulls < solid.num_tri() as i64);
    let leaf_size = TREE_LEAF_SIZE as i64;
    let leaf_stage_units = hulls + (hulls + leaf_size - 1) / leaf_size + 1;

    let ratios: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&ratios);
    let bar: BooleanReporter = Arc::new(move |ratio: f64, _message: &str| {
        sink.lock().unwrap().push(ratio);
    });
    minkowski_sum_with_progress(
        &to_mesh(&solid, "solid").unwrap(),
        &sphere_mesh(1.0, 8).unwrap(),
        Some(bar),
        &CancelToken::new(),
    )
    .unwrap();

    let mut leaf_reports = 0;
    for &ratio in ratios.lock().unwrap().iter() {
        if ratio > 0.0 && ratio < TREE_LEAF_TIME_SHARE {
            let units = ratio / TREE_LEAF_TIME_SHARE * leaf_stage_units as f64;
            assert_within(units, units.round_ties_even(), 1e-6);
            leaf_reports += 1;
        }
    }

    assert!(leaf_reports > 10, "only {leaf_reports} leaf-stage reports");
}

#[test]
fn a_hollow_box_with_a_correctly_wound_cavity_keeps_its_cavity() {
    let dilated = minkowski_sum(&hollow_box(), &ball()).unwrap();

    assert_within(signed_volume(&dilated), HOLLOW_DILATED_VOLUME, 1e-6);
}

#[test]
fn an_entirely_inside_out_hollow_box_is_treated_as_the_hollow_box() {
    let mut inside_out = hollow_box();
    inside_out.reverse_faces();

    let dilated = minkowski_sum(&inside_out, &ball()).unwrap();
    let eroded = minkowski_difference(&inside_out, &ball()).unwrap();
    let hollow_eroded = minkowski_difference(&hollow_box(), &ball()).unwrap();

    assert_within(signed_volume(&dilated), HOLLOW_DILATED_VOLUME, 1e-6);
    assert_within(signed_volume(&eroded), signed_volume(&hollow_eroded), 1e-6);
}

/// The public orientation returns the operand the way the kernel reads it: an inside-out body
/// beside a correct one is turned, and a correctly wound cavity is left a cavity.
#[test]
fn orient_shells_as_solid_turns_only_the_inside_out_shells() {
    let mut mixed = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    let mut inside_out = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    inside_out.reverse_faces();
    mixed.copy_all_faces(&inside_out, Matrix4X4::create_translation(20.0, 0.0, 0.0));
    assert_within(
        signed_volume(&orient_shells_as_solid(&mixed, &CancelToken::new()).unwrap()),
        2.0 * OUTER_SIDE * OUTER_SIDE * OUTER_SIDE,
        1e-6,
    );

    let hollow = hollow_box();
    assert_within(
        signed_volume(&orient_shells_as_solid(&hollow, &CancelToken::new()).unwrap()),
        signed_volume(&hollow_box()),
        1e-6,
    );
}

fn ball() -> Mesh {
    sphere_mesh(1.0, 8).unwrap()
}

/// A 10 mm cube with a 4 mm cavity at its centre, the cavity's faces pointing into it.
fn hollow_box() -> Mesh {
    let mut box_mesh = create_cube(OUTER_SIDE, OUTER_SIDE, OUTER_SIDE);
    let mut cavity = create_cube(CAVITY_SIDE, CAVITY_SIDE, CAVITY_SIDE);
    cavity.reverse_faces();
    box_mesh.copy_all_faces(&cavity, Matrix4X4::IDENTITY);
    box_mesh
}

/// A 20 mm cube holding a 14 mm cavity holding a 4 mm cube, each wound as its nesting demands.
fn three_level_nest() -> Mesh {
    let mut nest = create_cube(20.0, 20.0, 20.0);
    let mut cavity = create_cube(14.0, 14.0, 14.0);
    cavity.reverse_faces();
    nest.copy_all_faces(&cavity, Matrix4X4::IDENTITY);
    nest.copy_all_faces(&create_cube(4.0, 4.0, 4.0), Matrix4X4::IDENTITY);
    nest
}
