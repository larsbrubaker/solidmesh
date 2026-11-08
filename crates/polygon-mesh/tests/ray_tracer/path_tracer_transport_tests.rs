//! The integrator against closed forms: a furnace returns the environment through every surface
//! kind, a sun on a flat patch gives its direct-light formula, the three ways of reaching the sun
//! agree (the combined weights stay within a stated band of the better one, and why not always
//! at or below it is recorded at that test), mirrors and glass reproduce the environment exactly,
//! and a glass slab's infinite internal reflection series is met from both of its sides.

use std::f64::consts::PI;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, LightStrategy, LocalFrame, PathTraceSettings, RenderCamera,
    ScatteringSurface, Sun,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::path_tracer_support::{
    agreement_tolerance, estimate, fine_sphere, flat_patch, scene_of, top_down_camera,
    SHARP_CREASE, SMOOTH_CREASE,
};
use crate::scattering_support::{Rng, Stats};

const WHITE: [f64; 3] = [1.0, 1.0, 1.0];

fn furnace(radiance: f64) -> LightSetup {
    LightSetup::environment_only(Environment::constant([radiance; 3]).unwrap())
}

/// What a surface returns in a furnace, looking at a closed sphere: the whole image, silhouettes
/// included, since that is where a shading-normal mistake would show.
fn furnace_mean_of(surface: ScatteringSurface, samples: u32) -> (f64, f64) {
    let scene = scene_of(
        &fine_sphere(10.0),
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        surface,
        furnace(1.0),
    );
    let (stats, _) = estimate(
        &scene,
        top_down_camera(40.0, 21.0),
        (6, 6),
        PathTraceSettings::default(),
        samples,
    );
    (stats[0].mean(), stats[0].standard_error())
}

/// Every opaque white surface returns all of its light, whatever the roughness: any lost or
/// doubled energy in the bounce logic moves this off 1. The tolerance is five standard errors
/// plus the small silhouette loss the integrator documents (directions below a bent face are
/// dropped), measured under 0.2% on this sphere.
#[test]
fn a_white_opaque_surface_in_a_furnace_returns_the_furnace() {
    let surfaces = [
        ("lambert", ScatteringSurface::new(WHITE, 0.0, 1.0, 0.0, 1.0)),
        (
            "metal r0.1",
            ScatteringSurface::new(WHITE, 1.0, 0.1, 0.0, 1.5),
        ),
        (
            "metal r0.5",
            ScatteringSurface::new(WHITE, 1.0, 0.5, 0.0, 1.5),
        ),
        (
            "metal r1.0",
            ScatteringSurface::new(WHITE, 1.0, 1.0, 0.0, 1.5),
        ),
        (
            "plastic r0.3",
            ScatteringSurface::new(WHITE, 0.0, 0.3, 0.0, 1.5),
        ),
        (
            "plastic r0.8",
            ScatteringSurface::new(WHITE, 0.0, 0.8, 0.0, 1.5),
        ),
    ];
    for (name, surface) in surfaces {
        let (mean, error) = furnace_mean_of(surface, 48);
        println!("furnace {name}: {mean} +- {error}");
        assert!(
            (mean - 1.0).abs() <= 5.0 * error + 0.002,
            "{name} returned {mean}, standard error {error}"
        );
    }
}

/// Perfect glass bends light without losing any, so the furnace comes back through it however
/// many times a path bounces inside.
#[test]
fn smooth_glass_in_a_furnace_returns_the_furnace() {
    for ior in [1.2, 1.5, 2.4] {
        let glass = ScatteringSurface::new(WHITE, 0.0, 0.0, 1.0, ior);
        let (mean, error) = furnace_mean_of(glass, 96);
        println!("furnace smooth glass ior {ior}: {mean} +- {error}");
        assert!(
            (mean - 1.0).abs() <= 5.0 * error + 0.002,
            "ior {ior} returned {mean}, standard error {error}"
        );
    }
}

/// The furnace through a rough glass sphere of radius 10, by following one surface after another
/// on the exact sphere with the production `sample` and nothing of the integrator: what the model
/// says the sphere returns, in closed-form geometry, for rays over the camera's square.
fn furnace_through_exact_sphere(glass: &ScatteringSurface, paths: u32, rng: &mut Rng) -> Stats {
    let mut stats = Stats::default();
    for _ in 0..paths {
        let x = (rng.unit() - 0.5) * 14.0;
        let y = (rng.unit() - 0.5) * 14.0;
        let mut position = Vector3::new(x, y, (100.0 - x * x - y * y).sqrt());
        let mut direction = -Vector3::UNIT_Z;
        let mut throughput = 1.0;
        let mut escaped = false;
        for _ in 0..64 {
            let normal = position.get_normal();
            let frame = LocalFrame::from_normal(normal).unwrap();
            let Some(sample) = glass.sample(frame.to_local(-direction), rng.u3()) else {
                break;
            };
            throughput *= sample.weight[0];
            let outgoing = frame.to_world(sample.direction).get_normal();
            if outgoing.dot(normal) > 0.0 {
                escaped = true;
                break;
            }
            // Inside: the other end of the chord.
            position = position + outgoing * (-2.0 * position.dot(outgoing));
            direction = outgoing;
        }
        stats.add(if escaped { throughput } else { 0.0 });
    }
    stats
}

/// Rough glass loses exactly what its single-scatter model loses: the mesh sphere through the
/// whole integrator agrees with the same model followed on an exact sphere, so a doubled loss,
/// a lost refraction or an energy gain would all show. The margin covers the tessellation.
#[test]
fn rough_glass_in_a_furnace_loses_only_what_its_model_loses() {
    let mut rng = Rng(0xfeed_0011);
    for roughness in [0.3, 0.7] {
        let glass = ScatteringSurface::new(WHITE, 0.0, roughness, 1.0, 1.5);
        let reference = furnace_through_exact_sphere(&glass, 400_000, &mut rng);
        let scene = scene_of(
            &fine_sphere(10.0),
            SMOOTH_CREASE,
            Matrix4X4::IDENTITY,
            glass,
            furnace(1.0),
        );
        let (stats, _) = estimate(
            &scene,
            top_down_camera(40.0, 14.0),
            (6, 6),
            PathTraceSettings::default(),
            2000,
        );
        let margin = agreement_tolerance(&stats[0], &reference) + 0.003;
        println!(
            "furnace rough glass r{roughness}: {} +- {} model {} +- {}",
            stats[0].mean(),
            stats[0].standard_error(),
            reference.mean(),
            reference.standard_error()
        );
        assert!(reference.mean() < 1.0, "the model should lose energy here");
        assert!(
            (stats[0].mean() - reference.mean()).abs() <= margin,
            "r{roughness}: {} against the model's {} (margin {margin})",
            stats[0].mean(),
            reference.mean()
        );
    }
}

/// Direct sunlight on a Lambert patch has an exact closed form: the disc's irradiance is
/// `radiance * pi * sin^2(radius) * cos(tilt)`, the cosine-weighted integral over the disc,
/// so the reflected radiance is `albedo * radiance * sin^2(radius) * cos(tilt)`.
#[test]
fn a_sun_on_a_flat_patch_matches_the_closed_form() {
    let albedo = [0.8, 0.5, 0.25];
    let tilt = 40.0_f64.to_radians();
    let direction = Vector3::new(tilt.sin(), 0.0, tilt.cos());
    for radius in [0.02, 0.1, 0.4] {
        let radiance = 7.0;
        let sun = Sun::new(direction, radius, [radiance; 3]).unwrap();
        let surface = ScatteringSurface::new(albedo, 0.0, 1.0, 0.0, 1.0);
        let scene = scene_of(
            &flat_patch(100.0, 0.0),
            SHARP_CREASE,
            Matrix4X4::IDENTITY,
            surface,
            LightSetup::sun_only(sun),
        );
        let (stats, _) = estimate(
            &scene,
            top_down_camera(5.0, 2.0),
            (4, 4),
            PathTraceSettings::default(),
            2000,
        );
        for (channel, stat) in stats.iter().enumerate() {
            let expected = albedo[channel] * radiance * radius.sin().powi(2) * tilt.cos();
            let difference = (stat.mean() - expected).abs();
            let allowed = 5.0 * stat.standard_error() + expected * 2e-4;
            assert!(
                difference <= allowed,
                "radius {radius} channel {channel}: {} against {expected} (allowed {allowed})",
                stat.mean()
            );
        }
    }
}

/// Next-event estimation alone, surface sampling alone and the combined weights compute the same
/// integral by different routes: their means must agree. The combined estimator must also stay
/// within a stated band of the better of the two, not at or below it: combining two strategies
/// can only do so much when one is nearly noiseless. With a narrow lobe entirely inside the sun,
/// surface sampling alone almost always lands on the sun with the same weight, and adding the
/// next-event sample brings its noise along (measured up to 3.8x the standard error, on an
/// estimate next-event alone would be 30x noisier than). Where the two overlap the combination
/// stays within a few percent of the better one.
#[test]
fn the_three_light_strategies_agree_and_combining_stays_near_the_better_one() {
    let tilt = 25.0_f64.to_radians();
    let eye = Vector3::new(0.0, -tilt.sin() * 20.0, tilt.cos() * 20.0);
    // The specular lobe's center: a sun near the mirror direction of the view.
    let direction = Vector3::new(0.1, tilt.sin(), tilt.cos());
    let camera = RenderCamera::orthographic(eye, Vector3::ZERO, Vector3::UNIT_Y, 0.5).unwrap();
    let surfaces = [
        ("lambert", ScatteringSurface::new(WHITE, 0.0, 1.0, 0.0, 1.0)),
        (
            "plastic",
            ScatteringSurface::new([0.5; 3], 0.0, 0.5, 0.0, 1.5),
        ),
        (
            "rough metal",
            ScatteringSurface::new(WHITE, 1.0, 0.6, 0.0, 1.5),
        ),
        (
            "shiny metal",
            ScatteringSurface::new(WHITE, 1.0, 0.15, 0.0, 1.5),
        ),
    ];
    for (name, surface) in surfaces {
        for radius in [0.06, 0.25, 0.6] {
            let sun = Sun::new(direction, radius, [10.0, 10.0, 10.0]).unwrap();
            let scene = scene_of(
                &flat_patch(100.0, 0.0),
                SHARP_CREASE,
                Matrix4X4::IDENTITY,
                surface,
                LightSetup::sun_only(sun),
            );
            let run = |strategy| {
                let settings = PathTraceSettings {
                    strategy,
                    max_bounces: 1,
                    ..PathTraceSettings::default()
                };
                estimate(&scene, camera, (1, 1), settings, 120_000).0[0]
            };
            let combined = run(LightStrategy::Combined);
            let next_event = run(LightStrategy::NextEventOnly);
            let sampled = run(LightStrategy::SurfaceSamplingOnly);
            println!(
                "{name} r{radius}: combined {} +- {} next {} +- {} sampled {} +- {}",
                combined.mean(),
                combined.standard_error(),
                next_event.mean(),
                next_event.standard_error(),
                sampled.mean(),
                sampled.standard_error()
            );
            for (other, label) in [(next_event, "next-event"), (sampled, "sampled")] {
                let tolerance = agreement_tolerance(&combined, &other);
                assert!(
                    (combined.mean() - other.mean()).abs() <= tolerance,
                    "{name} r{radius}: combined {} against {label} {} (tolerance {tolerance})",
                    combined.mean(),
                    other.mean()
                );
            }
            let best_error = next_event.standard_error().min(sampled.standard_error());
            println!(
                "ratio {name} r{radius}: {}",
                combined.standard_error() / best_error
            );
            // The band: measured ratios run 0.9 to 1.2 except the narrow-lobe case above (up to 3.8).
            // This bound is that band, not a claim the combination always wins.
            assert!(
                combined.standard_error() <= best_error * 5.0,
                "{name} r{radius}: combined error {} over best {best_error}",
                combined.standard_error()
            );
        }
    }
}

/// A perfect mirror shows the environment along the mirrored view exactly: no sampling is
/// involved, so every sample is the closed form.
#[test]
fn a_perfect_mirror_images_the_environment_exactly() {
    let environment =
        Environment::gradient([0.1, 0.3, 0.9], [0.9, 0.9, 0.8], [0.2, 0.15, 0.1]).unwrap();
    let scene = scene_of(
        &flat_patch(100.0, 0.0),
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::new(WHITE, 1.0, 0.0, 0.0, 1.5),
        LightSetup::environment_only(environment),
    );
    let view = Vector3::new(0.3, -0.5, -0.8).get_normal();
    let eye = view * -30.0;
    let camera = RenderCamera::orthographic(eye, Vector3::ZERO, Vector3::UNIT_Y, 4.0).unwrap();
    let mirrored = Vector3::new(view.x, view.y, -view.z);
    let expected = environment.radiance(mirrored);
    let (stats, _) = estimate(&scene, camera, (3, 3), PathTraceSettings::default(), 16);
    for (channel, stat) in stats.iter().enumerate() {
        assert!(
            (stat.mean() - expected[channel]).abs() < 1e-12,
            "channel {channel}: {} against {}",
            stat.mean(),
            expected[channel]
        );
    }
}

/// Glass with the index of air bends nothing and reflects nothing, so a slab of it shows the
/// environment straight through, in closed form at every sample.
#[test]
fn a_glass_slab_of_index_one_is_invisible() {
    let environment =
        Environment::gradient([0.1, 0.3, 0.9], [0.9, 0.9, 0.8], [0.2, 0.15, 0.1]).unwrap();
    let scene = scene_of(
        &create_cube(50.0, 50.0, 4.0),
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::new(WHITE, 0.0, 0.0, 1.0, 1.0),
        LightSetup::environment_only(environment),
    );
    let view = Vector3::new(0.2, 0.1, -0.97).get_normal();
    let camera =
        RenderCamera::orthographic(view * -30.0, Vector3::ZERO, Vector3::UNIT_Y, 4.0).unwrap();
    let expected = environment.radiance(view);
    let (stats, _) = estimate(&scene, camera, (3, 3), PathTraceSettings::default(), 16);
    for (channel, stat) in stats.iter().enumerate() {
        assert!(
            (stat.mean() - expected[channel]).abs() < 1e-9,
            "channel {channel}: {} against {}",
            stat.mean(),
            expected[channel]
        );
    }
}

/// A smooth glass cube traps some rays by total internal reflection: bouncing off the sides
/// inside must lose nothing, so the furnace comes back in full.
#[test]
fn total_internal_reflection_loses_no_energy() {
    let scene = scene_of(
        &create_cube(8.0, 8.0, 8.0),
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::new(WHITE, 0.0, 0.0, 1.0, 1.9),
        furnace(1.0),
    );
    let eye = Vector3::new(6.0, 4.0, 25.0);
    let camera = RenderCamera::orthographic(eye, Vector3::ZERO, Vector3::UNIT_Y, 4.0).unwrap();
    let (stats, _) = estimate(&scene, camera, (4, 4), PathTraceSettings::default(), 256);
    let mean = stats[0].mean();
    println!("cube tir: {mean} +- {}", stats[0].standard_error());
    assert!(
        (mean - 1.0).abs() <= 5.0 * stats[0].standard_error() + 1e-6,
        "cube returned {mean}"
    );
}

/// A plane-parallel slab between a black half-space and a bright one has the exact infinite
/// series for its multiple internal reflections: reflectance `2R / (1 + R)`, transmittance
/// `(1 - R) / (1 + R)`, with `R` the Fresnel reflectance at normal incidence. The index is high
/// so several round trips matter; a bounce limit or a roulette that loses energy shows here.
#[test]
fn a_glass_slab_meets_its_internal_reflection_series() {
    let ior = 4.0_f64;
    let fresnel = ((ior - 1.0) / (ior + 1.0)).powi(2);
    let environment = Environment::gradient([1.0; 3], [0.0; 3], [0.0; 3]).unwrap();
    let scene = scene_of(
        &create_cube(100.0, 100.0, 2.0),
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::new(WHITE, 0.0, 0.0, 1.0, ior),
        LightSetup::environment_only(environment),
    );
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 30.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        2.0,
    )
    .unwrap();
    // Looking down: rays reflected back up reach the zenith (radiance 1), the rest reach the
    // black ground.
    let reflected = 2.0 * fresnel / (1.0 + fresnel);
    let settings = PathTraceSettings {
        russian_roulette_after: Some(1),
        max_bounces: 400,
        ..PathTraceSettings::default()
    };
    let (stats, _) = estimate(&scene, camera, (3, 3), settings, 3000);
    let difference = (stats[0].mean() - reflected).abs();
    let allowed = 5.0 * stats[0].standard_error() + 1e-3;
    println!(
        "slab series: {} +- {} against {reflected}",
        stats[0].mean(),
        stats[0].standard_error()
    );
    assert!(
        difference <= allowed,
        "{} against {reflected} (allowed {allowed})",
        stats[0].mean()
    );
    // The transmitted side, rendered: the same slab with the light below instead of above.
    let transmitted = (1.0 - fresnel) / (1.0 + fresnel);
    let environment = Environment::gradient([0.0; 3], [0.0; 3], [1.0; 3]).unwrap();
    let scene = scene_of(
        &create_cube(100.0, 100.0, 2.0),
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        ScatteringSurface::new(WHITE, 0.0, 0.0, 1.0, ior),
        LightSetup::environment_only(environment),
    );
    let (stats, _) = estimate(&scene, camera, (3, 3), settings, 3000);
    let allowed = 5.0 * stats[0].standard_error() + 1e-3;
    println!(
        "slab transmitted: {} +- {} against {transmitted}",
        stats[0].mean(),
        stats[0].standard_error()
    );
    assert!(
        (stats[0].mean() - transmitted).abs() <= allowed,
        "{} against {transmitted} (allowed {allowed})",
        stats[0].mean()
    );
}

/// A smooth glass sphere bends the view by Snell's law and splits it by Fresnel's equations, in
/// closed form for one ray: the direct reflection, the ray through and out, and the rays that
/// bounce inside first. A ray refracted with its normal the wrong way round (entering when it
/// should leave) comes out on a different side of the sky, so the gradient exposes it.
#[test]
fn a_glass_sphere_bends_the_view_as_snell_says() {
    let ior = 1.5_f64;
    let environment = Environment::gradient([1.0; 3], [0.5; 3], [0.0; 3]).unwrap();
    let glass = ScatteringSurface::new(WHITE, 0.0, 0.0, 1.0, ior);
    let scene = scene_of(
        &crate::surface_hit_support::sphere(10.0, Vector3::ZERO, 400, 200),
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        glass,
        LightSetup::environment_only(environment),
    );
    // The camera looks along -X at a ray 0.6 of the radius above the center.
    let height = 6.0_f64;
    let camera = RenderCamera::orthographic(
        Vector3::new(40.0, 0.0, height),
        Vector3::new(0.0, 0.0, height),
        Vector3::UNIT_Z,
        0.01,
    )
    .unwrap();
    let (stats, _) = estimate(&scene, camera, (1, 1), PathTraceSettings::default(), 6000);

    let sin_in = height / 10.0;
    let incidence = sin_in.asin();
    let refracted = (sin_in / ior).asin();
    let fresnel = {
        let (cos_i, cos_t) = (incidence.cos(), refracted.cos());
        let s = (cos_i - ior * cos_t) / (cos_i + ior * cos_t);
        let p = (ior * cos_i - cos_t) / (ior * cos_i + cos_t);
        (s * s + p * p) / 2.0
    };
    let through = |turn: f64| {
        let direction = Vector3::new(-turn.cos(), 0.0, -turn.sin());
        environment.radiance(direction)[0]
    };
    let normal = Vector3::new(incidence.cos(), 0.0, incidence.sin());
    let view = -Vector3::UNIT_X;
    let reflected = view - normal * (2.0 * view.dot(normal));
    let mut expected = fresnel * environment.radiance(reflected)[0];
    let mut weight = (1.0 - fresnel) * (1.0 - fresnel);
    let mut turn = 2.0 * (incidence - refracted);
    for _ in 0..12 {
        expected += weight * through(turn);
        weight *= fresnel;
        turn += PI - 2.0 * refracted;
    }
    println!(
        "glass sphere: {} +- {} against {expected}",
        stats[0].mean(),
        stats[0].standard_error()
    );
    let allowed = 5.0 * stats[0].standard_error() + 0.01;
    assert!(
        (stats[0].mean() - expected).abs() <= allowed,
        "{} against {expected} (allowed {allowed})",
        stats[0].mean()
    );
}
