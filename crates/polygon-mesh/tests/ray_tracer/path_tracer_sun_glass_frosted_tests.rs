//! `SunThroughGlass::Straight` at every roughness, where the light it drops is actually drawn (a
//! large sun low over the floor): frosted glass leaves no specks on the floor under a pane and no
//! reflection of the sun on the floor by a wall, each row equal to the smooth one; a glossy floor
//! under a pane, a partly-glass pane and glass overhanging an opaque box's edge read the same by
//! all three light strategies, which is each light path counted once; and the camera looking at a
//! frosted pane, at the sun through it or at its own glow, sees what Exact shows. The helpers are
//! the rules tests' scenes.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    LightStrategy, PathTraceSettings, RenderCamera, RenderScene, ScatteringSurface, SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::path_tracer_sun_glass_rules_tests::{
    disc_irradiance, exact, floor_camera, glass, matte, pane_scene, sample_values, scene, straight,
    sun, with, ALBEDO, IOR,
};
use crate::path_tracer_support::{agreement_tolerance, estimate, flat_patch};
use crate::scattering_support::Stats;

/// Each side of the smooth threshold (a roughness of about 0.0316), and well into frosted.
const ROUGHNESSES: [f64; 4] = [0.0, 0.032, 0.1, 0.3];

/// A sample brighter than this on a floor that reads about 0.15 is a speck: the sun itself, some
/// 60 times the floor, carried by a path that found it through or off glass.
const SPECK: f64 = 0.5;

/// The mean and standard error of `values`, and how many are specks.
fn summary(values: &[f64]) -> (Stats, usize) {
    let mut stats = Stats::default();
    for &value in values {
        stats.add(value);
    }
    (stats, values.iter().filter(|&&value| value > SPECK).count())
}

#[test]
fn frosted_glass_leaves_no_specks_on_the_floor_under_a_pane() {
    let sun = sun(60.0_f64.to_radians(), 1.0, 10.0_f64.to_radians());
    // Roulette off: a survivor's reweighting makes the odd sample a few times the floor's.
    let settings = with(SunThroughGlass::Straight, |s| {
        s.russian_roulette_after = None
    });
    let mut smooth: Option<Stats> = None;
    for roughness in ROUGHNESSES {
        let scene = pane_scene(matte(), glass(roughness, IOR), sun);
        let values = sample_values(&scene, floor_camera(0.0), settings, 16384);
        let (stats, specks) = summary(&values);
        let largest = values.iter().copied().fold(0.0, f64::max);
        println!(
            "pane at roughness {roughness}: {} +- {}, {specks} over {SPECK}, largest {largest}",
            stats.mean(),
            stats.standard_error()
        );
        assert_eq!(
            specks, 0,
            "the floor under a pane at roughness {roughness} has {specks} specks (largest \
             {largest})"
        );
        match smooth {
            None => smooth = Some(stats),
            Some(smooth) => {
                // What a frosted pane changes on the floor under it is the floor's own light the
                // pane reflects back onto it, which is kept: by a roughness of 0.3 that has grown
                // by 0.3% of the floor. The families left out came back as a step of 5.4% at 0.1.
                let kept = if roughness > 0.1 {
                    0.01 * smooth.mean()
                } else {
                    0.0
                };
                let allowed = agreement_tolerance(&stats, &smooth) + kept;
                assert!(
                    (stats.mean() - smooth.mean()).abs() <= allowed,
                    "the floor under a pane is {} at roughness {roughness} and {} under a smooth \
                     one (allowed {allowed})",
                    stats.mean(),
                    smooth.mean()
                );
            }
        }
    }
}

#[test]
fn a_frosted_glass_walls_reflection_of_the_sun_is_left_off_the_floor() {
    // The rules tests' wall: glass 40 high from x = 4.5 to 5.5, the sun low behind the patch at
    // x = 2 to 4, mirrored onto it by the wall's face from 30 degrees off its normal. The
    // reflection, where Exact draws it, is some 3% of the open floor, many standard errors of
    // these rows.
    let (from_zenith, radius) = (60.0_f64.to_radians(), 10.0_f64.to_radians());
    let sun = sun(from_zenith, -1.0, radius);
    let settings = with(SunThroughGlass::Straight, |s| {
        s.max_bounces = 2;
        s.russian_roulette_after = None;
    });
    let radiance = 1.0 / (radius.sin() * radius.sin());
    let axis = Vector3::new(-from_zenith.sin(), 0.0, from_zenith.cos());
    let open = ALBEDO / std::f64::consts::PI * disc_irradiance(axis, radius, radiance, |_| 1.0);
    let mut smooth: Option<Stats> = None;
    for roughness in ROUGHNESSES {
        let wall = scene(
            vec![
                (flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, matte()),
                (
                    create_cube(1.0, 2000.0, 40.0),
                    Matrix4X4::create_translation(5.0, 0.0, 20.0),
                    glass(roughness, IOR),
                ),
            ],
            sun,
        );
        let values = sample_values(&wall, floor_camera(3.0), settings, 4096);
        let (stats, specks) = summary(&values);
        let largest = values.iter().copied().fold(0.0, f64::max);
        println!(
            "wall at roughness {roughness}: {} +- {}, {specks} over {SPECK}, largest {largest}, \
             open {open}",
            stats.mean(),
            stats.standard_error()
        );
        assert_eq!(
            specks, 0,
            "the floor by a glass wall at roughness {roughness} has {specks} specks (largest \
             {largest})"
        );
        let allowed = 5.0 * stats.standard_error() + 1e-3 * open;
        assert!(
            (stats.mean() - open).abs() <= allowed,
            "the floor by a glass wall at roughness {roughness} is {}, where the open floor is \
             {open} (allowed {allowed})",
            stats.mean()
        );
        match smooth {
            None => smooth = Some(stats),
            Some(smooth) => {
                let allowed = agreement_tolerance(&stats, &smooth);
                assert!(
                    (stats.mean() - smooth.mean()).abs() <= allowed,
                    "the floor by a glass wall is {} at roughness {roughness} and {} by a smooth \
                     one (allowed {allowed})",
                    stats.mean(),
                    smooth.mean()
                );
            }
        }
    }
}

/// The three light strategies' estimates of `camera`'s view of `scene` under Straight.
fn three_strategies(
    scene: &RenderScene,
    camera: RenderCamera,
    samples: [u32; 3],
) -> [(LightStrategy, Stats); 3] {
    let run = |strategy, samples| {
        let settings = PathTraceSettings {
            strategy,
            ..straight()
        };
        (
            strategy,
            estimate(scene, camera, (4, 4), settings, samples).0[0],
        )
    };
    [
        run(LightStrategy::Combined, samples[0]),
        run(LightStrategy::NextEventOnly, samples[1]),
        run(LightStrategy::SurfaceSamplingOnly, samples[2]),
    ]
}

/// Each pair of the three estimates agrees within their noise, with no further slack.
fn assert_strategies_agree(what: &str, estimates: &[(LightStrategy, Stats); 3]) {
    for (strategy, stats) in estimates {
        println!(
            "{what}, {strategy:?}: {} +- {}",
            stats.mean(),
            stats.standard_error()
        );
    }
    assert!(estimates[0].1.mean() > 0.0, "{what}: no light at all");
    for (i, (a_strategy, a)) in estimates.iter().enumerate() {
        for (b_strategy, b) in &estimates[i + 1..] {
            let allowed = agreement_tolerance(a, b);
            assert!(
                (a.mean() - b.mean()).abs() <= allowed,
                "{what}: {a_strategy:?} reads {} and {b_strategy:?} {} (allowed {allowed}): a \
                 light path is counted twice or not at all",
                a.mean(),
                b.mean()
            );
        }
    }
}

#[test]
fn a_glossy_floor_under_a_pane_reads_the_same_by_every_strategy() {
    // The view's mirror direction off the floor is straight up, into the sun: the glossy lobe
    // both strategies reach the sun by, through the pane.
    let sun = sun(0.0, 1.0, 10.0_f64.to_radians());
    let floor = ScatteringSurface::new([0.9; 3], 1.0, 0.3, 0.0, 1.5);
    let open = scene(
        vec![(flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, floor)],
        sun,
    );
    let estimates = three_strategies(&open, floor_camera(0.0), [8192, 8192, 32768]);
    assert_strategies_agree("glossy floor in the open", &estimates);
    for roughness in [0.0, 0.1] {
        let scene = pane_scene(floor, glass(roughness, IOR), sun);
        let estimates = three_strategies(&scene, floor_camera(0.0), [8192, 8192, 32768]);
        assert_strategies_agree(
            &format!("glossy floor under a pane at roughness {roughness}"),
            &estimates,
        );
    }
}

/// Glass from z = 1 to 2 that is half glass and half frosted plastic.
fn partly_glass() -> ScatteringSurface {
    ScatteringSurface::new([0.8; 3], 0.0, 0.3, 0.5, IOR)
}

#[test]
fn a_partly_glass_pane_and_the_floor_under_it_read_the_same_by_every_strategy() {
    let sun = sun(30.0_f64.to_radians(), 1.0, 10.0_f64.to_radians());
    let scene = pane_scene(matte(), partly_glass(), sun);
    let floor = three_strategies(&scene, floor_camera(0.0), [1024, 1024, 4096]);
    assert_strategies_agree("the floor under a partly-glass pane", &floor);
    // Its own top, seen from above: the plastic share's glint and diffuse, the glass share's
    // reflection of the sun, and the floor seen through it.
    let above = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 5.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        2.0,
    )
    .unwrap();
    let own = three_strategies(&scene, above, [1024, 1024, 4096]);
    assert_strategies_agree("a partly-glass pane seen from above", &own);
}

#[test]
fn glass_overhanging_a_box_lights_the_top_under_it_through_its_side() {
    // A matte box from z = 0 to 2 reaching to x = 10, and a glass block from z = 2 to 6 on it
    // reaching past its edge to x = 15. The sun, 70 degrees from straight up toward +x, reaches
    // the box's top near its edge through the glass's side face, at x = 15.
    let sun = sun(70.0_f64.to_radians(), 1.0, 10.0_f64.to_radians());
    let scene = scene(
        vec![
            (
                create_cube(40.0, 40.0, 2.0),
                Matrix4X4::create_translation(-10.0, 0.0, 1.0),
                matte(),
            ),
            (
                create_cube(25.0, 20.0, 4.0),
                Matrix4X4::create_translation(2.5, 0.0, 4.0),
                glass(0.0, IOR),
            ),
        ],
        sun,
    );
    let camera = RenderCamera::orthographic(
        Vector3::new(8.0, 0.0, 10.0),
        Vector3::new(8.0, 0.0, 2.0),
        Vector3::UNIT_Y,
        1.0,
    )
    .unwrap();
    let estimates = three_strategies(&scene, camera, [2048, 2048, 8192]);
    assert_strategies_agree("the box's top under overhanging glass", &estimates);
}

/// Exact and Straight agree within their noise for `camera`'s view of `scene`.
fn assert_as_exact(what: &str, scene: &RenderScene, camera: RenderCamera, samples: u32) {
    let straight = estimate(scene, camera, (4, 4), straight(), samples).0[0];
    let exact = estimate(scene, camera, (4, 4), exact(), samples).0[0];
    println!(
        "{what}: straight {} +- {}, exact {} +- {}",
        straight.mean(),
        straight.standard_error(),
        exact.mean(),
        exact.standard_error()
    );
    assert!(straight.mean() > 0.0, "{what}: no light at all");
    let allowed = agreement_tolerance(&straight, &exact);
    assert!(
        (straight.mean() - exact.mean()).abs() <= allowed,
        "{what}: {} going straight through and {} exactly (allowed {allowed})",
        straight.mean(),
        exact.mean()
    );
}

#[test]
fn the_camera_sees_a_frosted_pane_as_exact_shows_it() {
    let pane = |roughness, sun| {
        scene(
            vec![(
                create_cube(2000.0, 2000.0, 1.0),
                Matrix4X4::create_translation(0.0, 0.0, 1.5),
                glass(roughness, IOR),
            )],
            sun,
        )
    };
    // Below the pane looking straight up.
    let camera =
        RenderCamera::orthographic(Vector3::ZERO, Vector3::UNIT_Z, Vector3::UNIT_Y, 2.0).unwrap();
    // The sun through it, blurred by it.
    let overhead = sun(0.0, 1.0, 2.5_f64.to_radians());
    assert_as_exact(
        "the sun through a frosted pane",
        &pane(0.1, overhead),
        camera,
        1024,
    );
    // Its own glow, lit from behind by a sun off the view.
    let behind = sun(30.0_f64.to_radians(), 1.0, 10.0_f64.to_radians());
    assert_as_exact("a frosted pane's glow", &pane(0.3, behind), camera, 1024);
}

#[test]
#[ignore = "reports what a vertex's line toward the sun costs; run in a release build"]
fn measure_what_a_vertexs_line_toward_the_sun_costs() {
    // The app's sun: 2.5 degrees across, 30 degrees off straight up, over a floor under a pane.
    let sun = sun(30.0_f64.to_radians(), 1.0, 1.25_f64.to_radians());
    for (label, floor) in [
        (
            "diffuse",
            ScatteringSurface::new([0.8; 3], 0.0, 1.0, 0.0, 1.5),
        ),
        (
            "glossy 0.3",
            ScatteringSurface::new([0.9; 3], 1.0, 0.3, 0.0, 1.5),
        ),
        (
            "polished metal",
            ScatteringSurface::polished_metal([0.9; 3]),
        ),
    ] {
        let scene = pane_scene(floor, glass(0.0, IOR), sun);
        // Looking down along the sun's mirror direction, so a glossy floor's lobe meets the sun.
        let camera = RenderCamera::orthographic(
            Vector3::new(-0.25, 0.0, 0.5 * 30.0_f64.to_radians().cos()),
            Vector3::ZERO,
            Vector3::UNIT_Y,
            2.0,
        )
        .unwrap();
        for (what, settings) in [("straight", straight()), ("exact", exact())] {
            let (_, work) = estimate(&scene, camera, (4, 4), settings, 4096);
            let vertices: u64 = work
                .bounce_histogram
                .iter()
                .enumerate()
                .map(|(bounces, paths)| bounces as u64 * paths)
                .sum();
            let shadow = work.first_hit_shadow_rays + work.later_shadow_rays;
            let traces = shadow + work.sun_line_rays;
            println!(
                "{label}, {what}: {shadow} shadow rays and {} more sun line traces over {vertices} \
                 vertices, {:.4} traces a vertex, {} bounce rays",
                work.sun_line_rays,
                traces as f64 / vertices as f64,
                work.bounce_rays
            );
        }
    }
}
