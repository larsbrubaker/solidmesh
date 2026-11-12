//! How much work finding where glass parts lie on each other does, counted rather than timed: a
//! large glass slab under a small, finely meshed glass sheet costs the same whatever the slab's
//! size and whichever part comes first, and a scene where no two glass parts come near each
//! other carries no triangle into the search at all. Also partly there glass lying on glass, in
//! a white furnace: whichever of two stacked boxes is half there, no sample gains light. Partly
//! there glass still loses some (glass_seam's header says why); an ignored test states the
//! wanted mean.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    ContactSearchWork, PlacedParts, RenderCamera, RenderScene, ScatteringSurface,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::glass_tray_support::{part_of, rig_lights};
use crate::path_tracer_glass_contact_measure_tests::grid_slab;
use crate::path_tracer_glass_contact_tests::{
    boxes_scene, furnace, furnace_samples, FURNACE_SLACK,
};
use crate::scattering_support::Stats;

/// The search's work for a slab `2 * half` square and 2 thick, its top at z = 0, under a sheet
/// 10 square and 1 thick of about 20,000 triangles lying on it, the slab first when
/// `slab_first`.
fn slab_and_sheet_work(half: f64, slab_first: bool) -> ContactSearchWork {
    let mut slab = create_cube(2.0 * half, 2.0 * half, 2.0);
    slab.translate_xyz(0.0, 0.0, -1.0);
    let sheet = grid_slab(0.0, 70, 0.3);
    let slab = part_of(&slab, Matrix4X4::IDENTITY);
    let sheet = part_of(
        &sheet,
        Matrix4X4::mult(
            Matrix4X4::create_scale(0.1, 0.1, 0.5),
            Matrix4X4::create_translation(-5.0, -5.0, 0.0),
        ),
    );
    let parts = if slab_first {
        vec![slab, sheet]
    } else {
        vec![sheet, slab]
    };
    let scene = RenderScene::new(
        PlacedParts::new(parts),
        vec![ScatteringSurface::glass(); 2],
        rig_lights(),
    )
    .unwrap();
    scene.glass_contact_search_work()
}

#[test]
fn a_large_slab_under_a_fine_sheet_costs_no_more_search_than_a_small_one() {
    for slab_first in [true, false] {
        let small = slab_and_sheet_work(100.0, slab_first);
        let large = slab_and_sheet_work(1000.0, slab_first);
        println!("slab first {slab_first}: half 100 {small:?}, half 1000 {large:?}");
        assert!(
            small.pairs_tested > 0,
            "the sheet lies on the slab: {small:?}"
        );
        assert!(
            large.cells_visited <= 2 * small.cells_visited
                && large.pairs_tested <= 2 * small.pairs_tested,
            "slab first {slab_first}: the search grew with the slab, {small:?} at half 100 and \
             {large:?} at half 1000"
        );
    }
}

#[test]
fn glass_parts_that_cannot_touch_carry_no_triangle_into_the_search() {
    let glass = ScatteringSurface::glass();
    let one = boxes_scene(&[([10.0; 3], [0.0; 3], glass)], true, rig_lights());
    let apart = boxes_scene(
        &[
            ([10.0; 3], [0.0; 3], glass),
            ([10.0; 3], [30.0, 0.0, 0.0], glass),
        ],
        true,
        rig_lights(),
    );
    for (scene, what) in [(one, "one glass part"), (apart, "two glass parts apart")] {
        let work = scene.glass_contact_search_work();
        assert_eq!(
            work,
            ContactSearchWork::default(),
            "{what}: the search did {work:?}"
        );
    }
}

/// Two boxes of the same glass stacked on a shared face, the top one at `top` opacity and the
/// bottom one at `bottom`, straight down in a white furnace: the brightest sample and the
/// statistics of every channel.
fn partly_there(top: f64, bottom: f64) -> (f64, Stats) {
    let glass = ScatteringSurface::glass();
    let scene = boxes_scene(
        &[
            ([10.0, 10.0, 4.0], [0.0, 0.0, 2.0], glass),
            ([10.0, 10.0, 4.0], [0.0, 0.0, 6.0], glass),
        ],
        false,
        furnace(),
    )
    .with_part_opacities(vec![bottom, top])
    .unwrap();
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 40.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        8.0,
    )
    .unwrap();
    furnace_samples(&scene, camera)
}

/// No sample carries more than the light it started in, and neither does the mean.
fn check_gains_nothing(top: f64, bottom: f64) {
    let (most, stats) = partly_there(top, bottom);
    println!(
        "top {top}, bottom {bottom}: brightest {most}, mean {}",
        stats.mean()
    );
    assert!(
        most <= 1.0 + FURNACE_SLACK,
        "top {top}, bottom {bottom}: a furnace sample came out at {most}"
    );
    let tolerance = 5.0 * stats.standard_error();
    assert!(
        stats.mean() <= 1.0 + tolerance,
        "top {top}, bottom {bottom}: the furnace's mean came out at {} (tolerance {tolerance})",
        stats.mean()
    );
}

#[test]
fn a_half_there_glass_box_on_a_glass_box_gains_nothing_in_a_furnace() {
    check_gains_nothing(0.5, 1.0);
}

#[test]
fn a_glass_box_on_a_half_there_glass_box_gains_nothing_in_a_furnace() {
    check_gains_nothing(1.0, 0.5);
}

#[test]
fn a_half_there_glass_box_on_an_invisible_one_gains_nothing_in_a_furnace() {
    check_gains_nothing(0.5, 0.0);
}

/// What partly there glass should do: keep every bit of the furnace's light.
#[test]
#[ignore = "fails until a line leaving partly there glass remembers it was found there: a new line             decides the part afresh and loses light (glass_seam's header)"]
fn partly_there_glass_on_glass_keeps_the_furnace_light() {
    for (top, bottom) in [(0.5, 1.0), (1.0, 0.5), (0.5, 0.0)] {
        let (_, stats) = partly_there(top, bottom);
        let tolerance = 5.0 * stats.standard_error() + 0.005;
        assert!(
            (stats.mean() - 1.0).abs() <= tolerance,
            "top {top}, bottom {bottom}: the furnace's mean came out at {} (tolerance              {tolerance})",
            stats.mean()
        );
    }
}
