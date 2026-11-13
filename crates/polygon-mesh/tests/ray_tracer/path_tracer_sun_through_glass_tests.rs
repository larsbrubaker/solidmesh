//! Sunlight straight through smooth glass (`SunThroughGlass::Straight`): the floor under a pane
//! gets the sun the shadow ray carries, dimmed by each crossing's Fresnel transmission and tinted
//! as a refracted path is; a point inside glass gets one crossing's worth with no index-squared
//! scaling; the light is counted once, so every way of reaching the sun converges to the same
//! floor, the straight line's closed form (with the diffuse floor's small power heuristic share
//! of the exact route, well within the check's half of the gap), while Exact converges to the
//! pane's whole series;
//! resting and stacked glass light the floor under them as one piece; and an opaque or partly
//! there part over the glass still shades it.

use std::f64::consts::PI;
use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    LightSetup, LightStrategy, MeshSurface, PathTraceSettings, PlacedPart, PlacedParts,
    RenderCamera, RenderScene, ScatteringSurface, Sun, SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

use crate::path_tracer_support::{agreement_tolerance, estimate, flat_patch, SHARP_CREASE};
use crate::scattering_support::Stats;
use crate::surface_hit_support::tree_of;

const IOR: f64 = 1.5;
const ALBEDO: f64 = 0.3;

fn matte() -> ScatteringSurface {
    ScatteringSurface::new([ALBEDO; 3], 0.0, 1.0, 0.0, 1.0)
}

fn glass(color: [f64; 3]) -> ScatteringSurface {
    ScatteringSurface::new(color, 0.0, 0.0, 1.0, IOR)
}

fn part(mesh: &Mesh, matrix: Matrix4X4) -> PlacedPart {
    PlacedPart::new(
        tree_of(mesh),
        Arc::new(MeshSurface::new(mesh, SHARP_CREASE).unwrap()),
        matrix,
    )
    .unwrap()
}

/// The Fresnel reflectance of unpolarized light meeting glass of `IOR` from air at `cos_i`.
fn fresnel(cos_i: f64) -> f64 {
    let sin2_t = (1.0 - cos_i * cos_i) / (IOR * IOR);
    let cos_t = (1.0 - sin2_t).sqrt();
    let perpendicular = (cos_i - IOR * cos_t) / (cos_i + IOR * cos_t);
    let parallel = (IOR * cos_i - cos_t) / (IOR * cos_i + cos_t);
    0.5 * (perpendicular * perpendicular + parallel * parallel)
}

/// A sun `from_zenith` radians off straight up, tilted toward +X, of `radius`, its radiance
/// giving the unshaded floor an irradiance of about pi.
fn sun(from_zenith: f64, radius: f64) -> (Sun, Vector3, f64) {
    let direction = Vector3::new(from_zenith.sin(), 0.0, from_zenith.cos());
    let radiance = 1.0 / (radius.sin() * radius.sin());
    (
        Sun::new(direction, radius, [radiance; 3]).unwrap(),
        direction,
        radiance,
    )
}

/// `radiance` times the integral over the sun's disc of `passed(cos_z) * cos_z`, where `cos_z` is
/// a direction's cosine to +Z: the irradiance of a floor facing up, each direction passed by
/// `passed`. Midpoint rule in the disc's own polar coordinates.
fn disc_irradiance(axis: Vector3, radius: f64, radiance: f64, passed: impl Fn(f64) -> f64) -> f64 {
    let tangent = Vector3::new(axis.z, 0.0, -axis.x);
    let bitangent = Vector3::UNIT_Y;
    let (rings, turns) = (400, 400);
    let mut sum = 0.0;
    for ring in 0..rings {
        let alpha = (f64::from(ring) + 0.5) * radius / f64::from(rings);
        for turn in 0..turns {
            let phi = (f64::from(turn) + 0.5) * 2.0 * PI / f64::from(turns);
            let direction =
                axis * alpha.cos() + (tangent * phi.cos() + bitangent * phi.sin()) * alpha.sin();
            let cos_z = direction.z;
            sum += passed(cos_z) * cos_z * alpha.sin();
        }
    }
    radiance * sum * (radius / f64::from(rings)) * (2.0 * PI / f64::from(turns))
}

/// The floor at z = 0 and a pane from z = 1 to z = 2 far wider than the patch seen, plus
/// `above`: the parts over the pane and their surfaces and opacities.
fn pane_scene(
    pane: ScatteringSurface,
    lights: LightSetup,
    above: Vec<(Mesh, Matrix4X4, ScatteringSurface, f64)>,
) -> RenderScene {
    let floor = flat_patch(1000.0, 0.0);
    let slab = create_cube(2000.0, 2000.0, 1.0);
    let mut parts = vec![
        part(&floor, Matrix4X4::IDENTITY),
        part(&slab, Matrix4X4::create_translation(0.0, 0.0, 1.5)),
    ];
    let mut surfaces = vec![matte(), pane];
    let mut opacities = vec![1.0, 1.0];
    for (mesh, matrix, surface, opacity) in above {
        parts.push(part(&mesh, matrix));
        surfaces.push(surface);
        opacities.push(opacity);
    }
    RenderScene::new(PlacedParts::new(parts), surfaces, lights)
        .unwrap()
        .with_part_opacities(opacities)
        .unwrap()
}

/// Between the floor and the pane, looking straight down, so the view crosses no glass.
fn under_pane_camera() -> RenderCamera {
    RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 0.5),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        2.0,
    )
    .unwrap()
}

/// Only the light the first vertex's shadow ray brings: one scattering event, no roulette.
fn direct_only() -> PathTraceSettings {
    PathTraceSettings {
        max_bounces: 1,
        russian_roulette_after: None,
        ..PathTraceSettings::default()
    }
}

fn means(stats: &[Stats; 3]) -> [f64; 3] {
    [stats[0].mean(), stats[1].mean(), stats[2].mean()]
}

fn assert_near(what: &str, stats: &Stats, expected: f64) {
    let (mean, error) = (stats.mean(), stats.standard_error());
    let allowed = 5.0 * error + 1e-3 * expected;
    assert!(
        (mean - expected).abs() <= allowed,
        "{what}: {mean} +- {error}, expected {expected} (allowed {allowed})"
    );
}

#[test]
fn the_floor_under_a_pane_gets_the_sun_through_both_faces() {
    let radius = 2.5_f64.to_radians();
    let tint = [0.25, 0.5, 1.0];
    for from_zenith in [0.0_f64, 60.0_f64.to_radians()] {
        let (sun, axis, radiance) = sun(from_zenith, radius);
        let lights = LightSetup::sun_only(sun);
        let open = ALBEDO / PI * disc_irradiance(axis, radius, radiance, |_| 1.0);
        let through = ALBEDO / PI
            * disc_irradiance(axis, radius, radiance, |cos| {
                (1.0 - fresnel(cos)) * (1.0 - fresnel(cos))
            });
        let clear = pane_scene(glass([1.0; 3]), lights, Vec::new());
        let (stats, _) = estimate(&clear, under_pane_camera(), (4, 4), direct_only(), 64);
        println!(
            "{from_zenith}: clear pane {:?}, expected {through} (open floor {open})",
            means(&stats)
        );
        assert_near("the floor under a clear pane", &stats[0], through);

        let tinted = pane_scene(glass(tint), lights, Vec::new());
        let (stats, _) = estimate(&tinted, under_pane_camera(), (4, 4), direct_only(), 64);
        // One crossing tints by the color's square root, as the refraction does; two give it.
        for (channel, stat) in stats.iter().enumerate() {
            assert_near(
                "the floor under a tinted pane",
                stat,
                through * tint[channel],
            );
        }
    }
}

#[test]
fn a_point_inside_glass_gets_one_crossing_with_no_index_scaling() {
    let radius = 2.5_f64.to_radians();
    for from_zenith in [0.0_f64, 60.0_f64.to_radians()] {
        let (sun, axis, radiance) = sun(from_zenith, radius);
        // A matte patch at z = 3 inside a block of glass from z = 1 to z = 5, seen from inside
        // the glass, so only its shadow ray crosses a face: the block's top.
        let patch = flat_patch(1000.0, 3.0);
        let block = create_cube(2000.0, 2000.0, 4.0);
        let scene = RenderScene::new(
            PlacedParts::new(vec![
                part(&patch, Matrix4X4::IDENTITY),
                part(&block, Matrix4X4::create_translation(0.0, 0.0, 3.0)),
            ]),
            vec![matte(), glass([1.0; 3])],
            LightSetup::sun_only(sun),
        )
        .unwrap();
        let camera = RenderCamera::orthographic(
            Vector3::new(0.0, 0.0, 3.5),
            Vector3::new(0.0, 0.0, 3.0),
            Vector3::UNIT_Y,
            2.0,
        )
        .unwrap();
        let expected =
            ALBEDO / PI * disc_irradiance(axis, radius, radiance, |cos| 1.0 - fresnel(cos));
        let (stats, _) = estimate(&scene, camera, (4, 4), direct_only(), 64);
        println!(
            "{from_zenith}: inside {:?}, expected {expected}",
            means(&stats)
        );
        assert_near("a patch inside glass", &stats[0], expected);
    }
}

/// The Fresnel reflectance from air into glass of index `ior` at `cos_i`.
fn fresnel_of(cos_i: f64, ior: f64) -> f64 {
    let sin2_t = (1.0 - cos_i * cos_i) / (ior * ior);
    let cos_t = (1.0 - sin2_t).sqrt();
    let perpendicular = (cos_i - ior * cos_t) / (cos_i + ior * cos_t);
    let parallel = (ior * cos_i - cos_t) / (ior * cos_i + cos_t);
    0.5 * (perpendicular * perpendicular + parallel * parallel)
}

/// `integrand(theta)` integrated over [0, `upper`] by the midpoint rule.
fn integrate(upper: f64, integrand: impl Fn(f64) -> f64) -> f64 {
    let steps = 20_000;
    let step = upper / f64::from(steps);
    (0..steps)
        .map(|i| integrand((f64::from(i) + 0.5) * step) * step)
        .sum()
}

#[test]
fn the_sun_through_a_pane_is_counted_once() {
    // A dense glass and a large sun overhead: what Straight leaves out (the light that went back
    // and forth inside the pane, `F^2` of it) is several percent, and Exact, which finds the sun
    // only by refracting into its disc, finds it often enough to measure that.
    let (ior, radius) = (3.0, 30.0_f64.to_radians());
    let (sun, _, radiance) = sun(0.0, radius);
    let pane = ScatteringSurface::new([1.0; 3], 0.0, 0.0, 1.0, ior);
    let scene = pane_scene(pane, LightSetup::sun_only(sun), Vec::new());
    let f = |cos: f64| fresnel_of(cos, ior);
    let disc = |passed: &dyn Fn(f64) -> f64| {
        radiance
            * integrate(radius, |theta| {
                passed(theta.cos()) * theta.cos() * theta.sin() * 2.0 * PI
            })
    };
    // The floor's diffuse light the pane sends back down, its whole series.
    let returned = integrate(PI / 2.0, |theta| {
        2.0 * f(theta.cos()) / (1.0 + f(theta.cos())) * 2.0 * theta.cos() * theta.sin()
    });
    let floor = |direct: f64| ALBEDO / PI * direct / (1.0 - ALBEDO * returned);
    let straight = floor(disc(&|cos| (1.0 - f(cos)).powi(2)));
    let series = floor(disc(&|cos| (1.0 - f(cos)) / (1.0 + f(cos))));
    let run = |through, strategy, samples| {
        let settings = PathTraceSettings {
            strategy,
            sun_through_glass: through,
            ..PathTraceSettings::default()
        };
        estimate(&scene, under_pane_camera(), (4, 4), settings, samples).0[0]
    };
    let gap = series - straight;
    for (what, through, strategy, samples, expected) in [
        (
            "straight, both strategies",
            SunThroughGlass::Straight,
            LightStrategy::Combined,
            1024,
            straight,
        ),
        (
            "straight, surface sampling",
            SunThroughGlass::Straight,
            LightStrategy::SurfaceSamplingOnly,
            4096,
            straight,
        ),
        (
            "straight, next event",
            SunThroughGlass::Straight,
            LightStrategy::NextEventOnly,
            1024,
            straight,
        ),
        (
            "exact",
            SunThroughGlass::Exact,
            LightStrategy::Combined,
            65536,
            series,
        ),
    ] {
        let stats = run(through, strategy, samples);
        let allowed = 5.0 * stats.standard_error() + 1e-3 * expected;
        println!(
            "{what}: {} +- {}, expected {expected} (straight {straight}, series {series})",
            stats.mean(),
            stats.standard_error()
        );
        assert!(
            allowed < 0.5 * gap,
            "{what}: the check can't tell the series from the straight line (allowed {allowed}, \
             gap {gap})"
        );
        assert!(
            (stats.mean() - expected).abs() <= allowed,
            "{what}: the floor under the pane is {} +- {}, expected {expected} (allowed {allowed})",
            stats.mean(),
            stats.standard_error()
        );
        if strategy == LightStrategy::Combined && through == SunThroughGlass::Straight {
            // The shadow ray reaches the sun on every sample, so the floor is smooth.
            assert!(
                stats.standard_error() < 0.01 * stats.mean(),
                "the floor under a pane is {} +- {}",
                stats.mean(),
                stats.standard_error()
            );
        }
    }
}

/// The floor and glass `blocks` resting on it, the same glass, each `(center z, height)` of a 20
/// by 20 box.
fn resting_scene(blocks: &[(f64, f64)], sun: Sun) -> RenderScene {
    let floor = flat_patch(1000.0, 0.0);
    let mut parts = vec![part(&floor, Matrix4X4::IDENTITY)];
    let mut surfaces = vec![matte()];
    for &(center, height) in blocks {
        let mesh = create_cube(20.0, 20.0, height);
        parts.push(part(&mesh, Matrix4X4::create_translation(0.0, 0.0, center)));
        surfaces.push(glass([1.0; 3]));
    }
    RenderScene::new(PlacedParts::new(parts), surfaces, LightSetup::sun_only(sun)).unwrap()
}

#[test]
fn glass_resting_on_the_floor_and_stacked_glass_light_the_floor_under_them() {
    // Tilted off the view's mirror direction, so the camera never sees the sun in the top face.
    let (sun, _, _) = sun(20.0_f64.to_radians(), 10.0_f64.to_radians());
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        1.0,
    )
    .unwrap();
    let single = resting_scene(&[(1.0, 2.0)], sun);
    let stacked = resting_scene(&[(0.5, 1.0), (1.5, 1.0)], sun);
    let run = |scene: &RenderScene, strategy, samples| {
        let settings = PathTraceSettings {
            strategy,
            ..PathTraceSettings::default()
        };
        estimate(scene, camera, (4, 4), settings, samples).0[0]
    };
    let single_combined = run(&single, LightStrategy::Combined, 4096);
    let single_sampled = run(&single, LightStrategy::SurfaceSamplingOnly, 16384);
    let stacked_combined = run(&stacked, LightStrategy::Combined, 4096);
    println!(
        "resting: combined {} +- {}, surface sampling {} +- {}, stacked {} +- {}",
        single_combined.mean(),
        single_combined.standard_error(),
        single_sampled.mean(),
        single_sampled.standard_error(),
        stacked_combined.mean(),
        stacked_combined.standard_error()
    );
    let allowed = agreement_tolerance(&single_combined, &single_sampled);
    assert!(
        (single_combined.mean() - single_sampled.mean()).abs() <= allowed,
        "the floor under resting glass is {} with the shadow ray through it and {} by surface \
         sampling alone (allowed {allowed})",
        single_combined.mean(),
        single_sampled.mean()
    );
    let allowed = agreement_tolerance(&single_combined, &stacked_combined);
    assert!(
        (single_combined.mean() - stacked_combined.mean()).abs() <= allowed,
        "the floor under one block is {} and under two stacked blocks {} (allowed {allowed})",
        single_combined.mean(),
        stacked_combined.mean()
    );
    for (what, stats) in [
        ("one block", single_combined),
        ("two blocks", stacked_combined),
    ] {
        assert!(
            stats.standard_error() < 0.02 * stats.mean(),
            "the floor under {what} is {} +- {}",
            stats.mean(),
            stats.standard_error()
        );
    }
}

#[test]
fn an_opaque_part_over_the_glass_still_shades_it_and_a_partly_there_one_by_its_opacity() {
    let radius = 2.5_f64.to_radians();
    let (sun, axis, radiance) = sun(0.0, radius);
    let lights = LightSetup::sun_only(sun);
    let through = ALBEDO / PI
        * disc_irradiance(axis, radius, radiance, |cos| {
            (1.0 - fresnel(cos)) * (1.0 - fresnel(cos))
        });
    let blocker = || create_cube(2000.0, 2000.0, 0.5);
    let over = Matrix4X4::create_translation(0.0, 0.0, 3.0);
    let opaque = pane_scene(
        glass([1.0; 3]),
        lights,
        vec![(blocker(), over, matte(), 1.0)],
    );
    let (stats, _) = estimate(&opaque, under_pane_camera(), (4, 4), direct_only(), 64);
    assert_eq!(
        stats[0].mean(),
        0.0,
        "an opaque part over the pane lets sun through"
    );

    let half = pane_scene(
        glass([1.0; 3]),
        lights,
        vec![(blocker(), over, matte(), 0.5)],
    );
    let (stats, _) = estimate(&half, under_pane_camera(), (4, 4), direct_only(), 1024);
    assert_near(
        "the floor under a half-there part over a pane",
        &stats[0],
        0.5 * through,
    );
}
