//! A part's opacity through production renders: a white furnace still returns its environment, a
//! plate at opacity `a` scales its shadow by the closed form (a sheet, its reverse side, a closed
//! box and two plates in series), a half-there red part keeps its hue and diffuse brightness, a
//! deep stack ends and agrees with the closed form, opacity 0 hides a part from every ray, and
//! opacity 1 and determinism across tiles and threads are unchanged.

use std::f64::consts::{PI, TAU};
use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, MeshSurface, PathTraceSettings, PathTracer, PlacedPart, PlacedParts,
    RenderCamera, RenderScene, RenderSurface, ScatteringSurface, Sun, MAX_CROSSINGS_PER_LINE,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::path_tracer_golden_tests::{golden_scene, render_digest};
use crate::path_tracer_support::{
    estimate, fine_sphere, flat_patch, scene_of, top_down_camera, SHARP_CREASE,
};
use crate::surface_hit_support::tree_of;

fn lambert(albedo: [f64; 3]) -> ScatteringSurface {
    ScatteringSurface::new(albedo, 0.0, 1.0, 0.0, 1.0)
}

fn part(mesh: &Mesh) -> PlacedPart {
    PlacedPart::new(
        tree_of(mesh),
        Arc::new(MeshSurface::new(mesh, SHARP_CREASE).unwrap()),
        Matrix4X4::IDENTITY,
    )
    .unwrap()
}

/// `meshes` as one scene, each with its surface and opacity.
fn scene(meshes: &[(&Mesh, ScatteringSurface, f64)], lights: LightSetup) -> RenderScene {
    let parts = PlacedParts::new(meshes.iter().map(|(mesh, _, _)| part(mesh)).collect());
    RenderScene::new(
        parts,
        meshes.iter().map(|(_, surface, _)| *surface).collect(),
        lights,
    )
    .unwrap()
    .with_part_opacities(meshes.iter().map(|(_, _, opacity)| *opacity).collect())
    .unwrap()
}

/// A sheet over the whole floor at height `z`, facing +Z.
fn sheet(z: f64) -> Mesh {
    flat_patch(200.0, z)
}

/// A sheet from x = -60 to 0 at height `z`, its edge over the origin, facing +Z.
fn half_sheet(z: f64) -> Mesh {
    let vertices = [
        -60.0, -100.0, z, 0.0, -100.0, z, 0.0, 100.0, z, -60.0, 100.0, z,
    ];
    Mesh::from_arrays(&vertices, &[0, 1, 2, 0, 2, 3])
}

/// The same reach as `half_sheet` as a closed box, so a line crosses two surfaces of it.
fn closed_slab(z: f64) -> Mesh {
    let mut cube = create_cube(60.0, 200.0, 2.0);
    cube.transform(Matrix4X4::create_translation(-30.0, 0.0, z));
    cube
}

/// A sun straight overhead, `radius` wide.
fn overhead_sun(radius: f64) -> Sun {
    Sun::new(Vector3::UNIT_Z, radius, [1.0; 3]).unwrap()
}

/// One scattering event of the floor point at `x`, seen from below every plate: the direct sun
/// reaching it, with no light bounced around under the plates to hide the shadow.
fn floor_radiance(scene: &RenderScene, x: f64, samples: u32) -> (f64, f64) {
    let camera = RenderCamera::orthographic(
        Vector3::new(x, 0.0, 1.0),
        Vector3::new(x, 0.0, 0.0),
        Vector3::UNIT_Y,
        1e-3,
    )
    .unwrap();
    let settings = PathTraceSettings {
        max_bounces: 1,
        ..PathTraceSettings::default()
    };
    let (stats, _) = estimate(scene, camera, (1, 1), settings, samples);
    (stats[0].mean(), stats[0].standard_error())
}

/// The cosine-weighted solid angle of the part of a sun, `radius` wide straight overhead, that a
/// floor point at `x` sees past a half-plane at `height` (edge over the origin, blocking
/// `x <= 0`), by a fine midpoint rule in the disc's own angles.
fn visible_cosine_integral(x: f64, height: f64, radius: f64) -> f64 {
    let (rings, spokes) = (2400_u32, 4800_u32);
    let (dt, dp) = (radius / f64::from(rings), TAU / f64::from(spokes));
    let mut sum = 0.0;
    for ring in 0..rings {
        let t = (f64::from(ring) + 0.5) * dt;
        for spoke in 0..spokes {
            let phi = (f64::from(spoke) + 0.5) * dp;
            if x + height * t.tan() * phi.cos() > 0.0 {
                sum += t.cos() * t.sin() * dt * dp;
            }
        }
    }
    sum
}

/// A white part at any opacity in a furnace returns the environment: a path either misses it or
/// finds it and gets a perfect white bounce. Exact on a flat plate, to the sphere's facet loss on
/// a closed solid.
#[test]
fn a_furnace_returns_its_environment_at_any_opacity() {
    let radiance = [0.7, 0.8, 0.9];
    let light = LightSetup::environment_only(Environment::constant(radiance).unwrap());
    let plate = flat_patch(100.0, 0.0);
    for opacity in [0.1, 0.25, 0.5, 0.9] {
        let flat_furnace = scene(&[(&plate, lambert([1.0; 3]), opacity)], light);
        let (stats, work) = estimate(
            &flat_furnace,
            top_down_camera(30.0, 40.0),
            (5, 5),
            PathTraceSettings::default(),
            2000,
        );
        for (channel, expected) in radiance.into_iter().enumerate() {
            let mean = stats[channel].mean();
            println!("flat furnace opacity {opacity} channel {channel}: {mean}");
            assert!(
                (mean - expected).abs() < 1e-12,
                "flat plate, channel {channel}, opacity {opacity}: {mean} against {expected}"
            );
        }
        assert!(work.pass_through_rays > 0, "no ray went past the part");

        let sphere = fine_sphere(10.0);
        let closed_furnace = scene(&[(&sphere, lambert([1.0; 3]), opacity)], light);
        let (stats, _) = estimate(
            &closed_furnace,
            top_down_camera(40.0, 8.0),
            (9, 9),
            PathTraceSettings::default(),
            600,
        );
        for (channel, expected) in radiance.into_iter().enumerate() {
            let mean = stats[channel].mean();
            println!("sphere furnace opacity {opacity} channel {channel}: {mean}");
            assert!(
                (mean - expected).abs() < 4e-3,
                "sphere, channel {channel}, opacity {opacity}: {mean} against {expected}"
            );
        }
    }
}

/// The floor's shadow under a sheet of opacity `a` is `open - a * blocked` at every point across
/// the penumbra, whichever side of the sheet the shadow ray comes in from.
#[test]
fn a_sheets_shadow_is_scaled_by_its_opacity() {
    let radius = 0.1_f64;
    let height = 10.0;
    let width = height * radius.tan();
    let floor = flat_patch(100.0, 0.0);
    let open = visible_cosine_integral(1e6, height, radius) / PI;

    for (reversed, opacity) in [
        (false, 0.1),
        (false, 0.5),
        (false, 0.9),
        (true, 0.5),
        (true, 0.9),
    ] {
        let mut plate = half_sheet(height);
        if reversed {
            plate.reverse_faces();
        }
        let shadowed = scene(
            &[
                (&floor, lambert([1.0; 3]), 1.0),
                (&plate, lambert([1.0; 3]), opacity),
            ],
            LightSetup::sun_only(overhead_sun(radius)),
        );
        for fraction in [-3.0, -0.5, 0.0, 0.5, 3.0] {
            let x = fraction * width;
            let (mean, error) = floor_radiance(&shadowed, x, 60_000);
            let visible = visible_cosine_integral(x, height, radius) / PI;
            let blocked = open - visible;
            let expected = open - opacity * blocked;
            println!(
                "sheet opacity {opacity} reversed {reversed} at {fraction} widths: \
                 {mean} against {expected} (open {open})"
            );
            assert!(
                (mean - expected).abs() <= 5.0 * error + open * 1e-3,
                "opacity {opacity} reversed {reversed} at {fraction}: {mean} against {expected}"
            );
        }
    }
}

/// A closed box is a part like a sheet: a line goes through its two surfaces on one decision, so
/// its shadow is `open * (1 - a)`, not the `(1 - a)^2` a decision per surface would give.
#[test]
fn a_closed_parts_shadow_counts_the_part_once() {
    let radius = 0.05;
    let floor = flat_patch(100.0, 0.0);
    let slab = closed_slab(10.0);
    let open = visible_cosine_integral(1e6, 10.0, radius) / PI;
    for opacity in [0.3, 0.5, 0.8] {
        let shadowed = scene(
            &[
                (&floor, lambert([1.0; 3]), 1.0),
                (&slab, lambert([1.0; 3]), opacity),
            ],
            LightSetup::sun_only(overhead_sun(radius)),
        );
        // Under the box's middle, every shadow ray goes through both its faces.
        let (mean, error) = floor_radiance(&shadowed, -30.0, 40_000);
        let expected = open * (1.0 - opacity);
        println!("closed slab at {opacity}: {mean} against {expected}");
        assert!(
            (mean - expected).abs() <= 5.0 * error + open * 1e-3,
            "opacity {opacity}: {mean} against {expected}"
        );
    }
}

/// Two sheets in series scale the shadow by the product of what each leaves, in either order.
#[test]
fn two_sheets_in_series_multiply_what_they_leave() {
    let radius = 0.05;
    let floor = flat_patch(100.0, 0.0);
    let (low, high) = (sheet(10.0), sheet(20.0));
    let open = visible_cosine_integral(1e6, 10.0, radius) / PI;
    for (a, b) in [(0.3, 0.6), (0.6, 0.3), (0.9, 0.5)] {
        let shadowed = scene(
            &[
                (&floor, lambert([1.0; 3]), 1.0),
                (&low, lambert([1.0; 3]), a),
                (&high, lambert([1.0; 3]), b),
            ],
            LightSetup::sun_only(overhead_sun(radius)),
        );
        let (mean, error) = floor_radiance(&shadowed, 0.0, 40_000);
        let expected = open * (1.0 - a) * (1.0 - b);
        println!("series {a} then {b}: {mean} against {expected}");
        assert!(
            (mean - expected).abs() <= 5.0 * error + open * 1e-3,
            "{a} then {b}: {mean} against {expected}"
        );
    }
}

/// A path passing through a see-through part spends no bounce: with one bounce allowed, a camera
/// above a half-present sheet still reaches the sunlit floor below it on the half of paths that
/// pass, and the floor's value there is what it would be with the sheet at no opacity at all.
#[test]
fn a_pass_through_uses_no_bounce() {
    let radius = 0.05;
    let floor = flat_patch(100.0, 0.0);
    let plate = sheet(10.0);
    let lights = LightSetup::sun_only(overhead_sun(radius));
    let camera = top_down_camera(20.0, 2.0);
    let settings = PathTraceSettings {
        max_bounces: 1,
        ..PathTraceSettings::default()
    };
    let (open_floor, _) = estimate(
        &scene(&[(&floor, lambert([1.0; 3]), 1.0)], lights),
        camera,
        (4, 4),
        settings,
        4_000,
    );
    let (half, _) = estimate(
        &scene(
            &[
                (&floor, lambert([1.0; 3]), 1.0),
                (&plate, lambert([0.0; 3]), 0.5),
            ],
            lights,
        ),
        camera,
        (4, 4),
        settings,
        20_000,
    );
    // Half of the paths stop on the black sheet with nothing; the other half go through, reach
    // the floor at their only bounce, and find its shadow-ray visibility of the sun halved.
    let expected = open_floor[0].mean() * 0.5 * 0.5;
    println!(
        "one-bounce pass-through {} against {expected}",
        half[0].mean()
    );
    assert!(
        (half[0].mean() - expected).abs()
            <= 5.0 * half[0].standard_error() + open_floor[0].mean() * 1e-3,
        "{} against {expected}",
        half[0].mean()
    );
}

/// Half the coverage of a red part is half the red: the hue stays, and the brightness is half
/// the fully present part's, with the channels in the same ratio.
#[test]
fn a_half_present_red_part_keeps_its_color() {
    let red = lambert([0.8, 0.2, 0.1]);
    let black = lambert([0.0; 3]);
    let floor = flat_patch(100.0, -1.0);
    let plate = flat_patch(5.0, 0.0);
    let lights = LightSetup::sun_only(overhead_sun(0.05));
    let camera = top_down_camera(5.0, 4.0);
    let settings = PathTraceSettings {
        max_bounces: 1,
        ..PathTraceSettings::default()
    };

    let opaque = scene(&[(&floor, black, 1.0), (&plate, red, 1.0)], lights);
    let half = scene(&[(&floor, black, 1.0), (&plate, red, 0.5)], lights);
    let (opaque_stats, _) = estimate(&opaque, camera, (4, 4), settings, 8_000);
    let (half_stats, _) = estimate(&half, camera, (4, 4), settings, 8_000);

    for channel in 0..3 {
        let opaque_mean = opaque_stats[channel].mean();
        let half_mean = half_stats[channel].mean();
        println!("red part channel {channel}: half {half_mean} of opaque {opaque_mean}");
        let tolerance = 5.0
            * (0.25 * opaque_stats[channel].standard_error().powi(2)
                + half_stats[channel].standard_error().powi(2))
            .sqrt()
            + 1e-9;
        assert!(
            (half_mean - 0.5 * opaque_mean).abs() <= tolerance,
            "channel {channel}: {half_mean} against half of {opaque_mean}"
        );
    }
    // The hue is the channels' ratio, which coverage must not touch.
    let opaque_ratio = opaque_stats[1].mean() / opaque_stats[0].mean();
    let half_ratio = half_stats[1].mean() / half_stats[0].mean();
    assert!(
        (opaque_ratio - half_ratio).abs() < 0.02 * opaque_ratio,
        "hue moved: {opaque_ratio} against {half_ratio}"
    );
}

/// A stack far deeper than a path could bounce through, at low opacity: every path ends, and the
/// mean is the closed form. Plate `k` (0 the top) is found with chance `a (1 - a)^k`, and its
/// shadow ray then passes the `k` plates above with chance `(1 - a)^k`, so the mean is
/// `lit_floor * a * sum((1 - a)^(2k))` over the plates.
#[test]
fn a_deep_stack_ends_and_agrees_with_the_closed_form() {
    let count = 40_u32;
    let opacity = 0.2_f64;
    let meshes: Vec<Mesh> = (0..count).map(|k| sheet(f64::from(k) * 0.5)).collect();
    let radius = 0.05_f64;
    let list: Vec<(&Mesh, ScatteringSurface, f64)> = meshes
        .iter()
        .map(|mesh| (mesh, lambert([1.0; 3]), opacity))
        .collect();
    let stack = scene(&list, LightSetup::sun_only(overhead_sun(radius)));

    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 30.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        1.0,
    )
    .unwrap();
    let settings = PathTraceSettings {
        max_bounces: 1,
        ..PathTraceSettings::default()
    };
    let (stats, work) = estimate(&stack, camera, (1, 1), settings, 80_000);

    let lit_floor = visible_cosine_integral(1e6, 1.0, radius) / PI;
    let survive = (1.0 - opacity) * (1.0 - opacity);
    let geometric: f64 = (0..count).map(|k| survive.powi(k as i32)).sum();
    let expected = lit_floor * opacity * geometric;
    let (mean, error) = (stats[0].mean(), stats[0].standard_error());
    println!(
        "stack mean {mean} against {expected}; passes {}",
        work.pass_through_rays
    );
    assert!(
        (mean - expected).abs() <= 5.0 * error + expected * 1e-3,
        "{mean} against {expected}"
    );
    assert!(work.pass_through_rays > 80_000, "the stack was not walked");
}

/// A line through more surfaces than the cap ends there: a stack cannot run on forever, and its
/// path loses what is behind it, as past the bounce limit. Below the cap, an invisible stack is
/// nothing at all.
#[test]
fn a_line_past_the_crossing_cap_ends() {
    let meshes: Vec<Mesh> = (0..MAX_CROSSINGS_PER_LINE + 20)
        .map(|k| sheet(f64::from(k) * 0.01))
        .collect();
    let list: Vec<(&Mesh, ScatteringSurface, f64)> = meshes
        .iter()
        .map(|mesh| (mesh, lambert([1.0; 3]), 0.0))
        .collect();
    let environment = Environment::constant([0.5; 3]).unwrap();
    let too_deep = scene(&list, LightSetup::environment_only(environment));
    let camera = top_down_camera(50.0, 1.0);
    let (stats, work) = estimate(&too_deep, camera, (1, 1), PathTraceSettings::default(), 4);
    assert_eq!(stats[0].mean(), 0.0, "the line went past the cap");
    assert!(work.pass_through_rays <= u64::from(MAX_CROSSINGS_PER_LINE) * 4);

    let short = scene(&list[..20], LightSetup::environment_only(environment));
    let (stats, _) = estimate(&short, camera, (1, 1), PathTraceSettings::default(), 4);
    assert_eq!(stats[0].mean(), 0.5, "the invisible stack dimmed the sky");
}

/// A part at opacity 0 is not there for camera or shadow rays: the image with it matches the
/// image without it.
#[test]
fn opacity_zero_hides_a_part_from_every_ray() {
    let floor = flat_patch(100.0, 0.0);
    let plate = half_sheet(10.0);
    let lights = LightSetup {
        environment: Environment::constant([0.4, 0.5, 0.6]).unwrap(),
        sun: Some(Sun::new(Vector3::new(0.3, 0.4, 0.8), 0.1, [5.0; 3]).unwrap()),
    };
    let without = scene(&[(&floor, lambert([0.7; 3]), 1.0)], lights);
    let with_hidden = scene(
        &[
            (&floor, lambert([0.7; 3]), 1.0),
            (&plate, lambert([0.3; 3]), 0.0),
        ],
        lights,
    );
    let camera = top_down_camera(30.0, 8.0);
    let settings = PathTraceSettings {
        max_bounces: 4,
        ..PathTraceSettings::default()
    };
    let (a, _) = estimate(&without, camera, (5, 5), settings, 200);
    let (b, _) = estimate(&with_hidden, camera, (5, 5), settings, 200);
    for channel in 0..3 {
        assert!(
            (a[channel].mean() - b[channel].mean()).abs() <= 1e-12 * a[channel].mean().max(1.0),
            "channel {channel}: {} against {}",
            a[channel].mean(),
            b[channel].mean()
        );
    }
}

/// Stating opacity 1 for every part is the same as stating nothing, to the bit, and also the same
/// as the render pinned from before parts could be see-through.
#[test]
fn opacity_one_changes_no_bit_of_a_render() {
    let stated = golden_scene().with_part_opacities(vec![1.0, 1.0]).unwrap();
    assert_eq!(render_digest(&golden_scene()), render_digest(&stated));
}

/// A render with see-through parts is bit-identical done whole, in tiles in any order, or across
/// threads, and repeats itself for the same seed.
#[test]
fn a_see_through_render_is_deterministic_across_tiles_and_threads() {
    const SIZE: (u32, u32) = (13, 7);
    let floor = flat_patch(30.0, -2.0);
    let sphere = fine_sphere(10.0);
    let plate = half_sheet(12.0);
    let lights = LightSetup {
        environment: Environment::gradient([0.2, 0.3, 0.8], [0.9, 0.8, 0.7], [0.1, 0.1, 0.1])
            .unwrap(),
        sun: Some(Sun::new(Vector3::new(0.3, 0.2, 0.9), 0.1, [5.0; 3]).unwrap()),
    };
    let built = scene(
        &[
            (&floor, lambert([0.5; 3]), 1.0),
            (
                &sphere,
                ScatteringSurface::new([0.8, 0.3, 0.2], 0.0, 0.4, 0.0, 1.5),
                0.45,
            ),
            (&plate, lambert([0.6; 3]), 0.3),
        ],
        lights,
    );
    let settings = PathTraceSettings {
        seed: 5,
        ..PathTraceSettings::default()
    };
    let tracer = PathTracer::new(
        &built,
        top_down_camera(40.0, 26.0),
        SIZE.0,
        SIZE.1,
        settings,
    )
    .unwrap();

    let sums = |surface: &RenderSurface| {
        let mut sums = Vec::new();
        for y in 0..SIZE.1 {
            for x in 0..SIZE.0 {
                sums.extend(surface.pixel(x, y).unwrap().sum().map(f64::to_bits));
            }
        }
        sums
    };
    let render_whole = || {
        let mut surface = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
        for sample in 0..3 {
            tracer
                .render_pass(&mut surface, sample, &CancelToken::new())
                .unwrap();
        }
        surface
    };

    let expected = sums(&render_whole());
    assert_eq!(expected, sums(&render_whole()));

    let mut tiled = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
    for sample in 0..3 {
        let mut tiles = tiled.tiles_mut(4, 3).unwrap();
        tiles.reverse();
        for tile in &mut tiles {
            tracer
                .render_tile(tile, sample, &CancelToken::new())
                .unwrap();
        }
    }
    assert_eq!(expected, sums(&tiled));

    let mut threaded = RenderSurface::new(SIZE.0, SIZE.1).unwrap();
    let cancel = CancelToken::new();
    let (tracer_ref, cancel_ref) = (&tracer, &cancel);
    for sample in 0..3 {
        let tiles = threaded.tiles_mut(3, 2).unwrap();
        std::thread::scope(|scope| {
            for mut tile in tiles {
                scope.spawn(move || {
                    tracer_ref
                        .render_tile(&mut tile, sample, cancel_ref)
                        .unwrap()
                });
            }
        });
    }
    assert_eq!(expected, sums(&threaded));
}

/// What a caller needs to composite: a ray's coverage is the product through each part once, and
/// part opacities are readable by index.
#[test]
fn a_camera_rays_coverage_is_reachable() {
    let floor = flat_patch(50.0, 0.0);
    let (a, b) = (sheet(10.0), closed_slab(20.0));
    let empty_sky = LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap());
    let built = scene(
        &[
            (&floor, lambert([0.5; 3]), 1.0),
            (&a, lambert([0.5; 3]), 0.25),
            (&b, lambert([0.5; 3]), 0.5),
        ],
        empty_sky,
    );
    assert_eq!(built.part_opacity(1), Some(0.25));
    assert_eq!(built.part_opacity(9), None);

    let ray = |x: f64, y: f64| {
        RenderCamera::orthographic(
            Vector3::new(x, y, 100.0),
            Vector3::new(x, y, 0.0),
            Vector3::UNIT_Y,
            1.0,
        )
        .unwrap()
        .ray_through(0.5, 0.5, 1, 1)
    };
    // Over the floor past the slab: the floor is opaque, so all is covered.
    assert_eq!(built.coverage_along(ray(10.0, 10.0)).unwrap(), 1.0);

    // Open sky behind the sheet and slab: a quarter from the sheet alone, more through the slab
    // too (once through its two faces), and none past both.
    let open_sky = scene(
        &[(&a, lambert([0.5; 3]), 0.25), (&b, lambert([0.5; 3]), 0.5)],
        empty_sky,
    );
    assert_eq!(open_sky.coverage_along(ray(180.0, 0.0)).unwrap(), 0.25);
    assert_eq!(
        open_sky.coverage_along(ray(-30.0, 0.0)).unwrap(),
        1.0 - 0.75 * 0.5
    );
    assert_eq!(open_sky.coverage_along(ray(500.0, 0.0)).unwrap(), 0.0);
}

/// A bad opacity list is refused with a reason, and sane ones are accepted.
#[test]
fn bad_opacities_are_refused() {
    let floor = flat_patch(10.0, 0.0);
    let lights = LightSetup::environment_only(Environment::black());
    let build = || {
        scene_of(
            &floor,
            SHARP_CREASE,
            Matrix4X4::IDENTITY,
            lambert([1.0; 3]),
            lights,
        )
    };
    assert!(build().with_part_opacities(vec![0.5, 0.5]).is_err());
    assert!(build().with_part_opacities(vec![1.5]).is_err());
    assert!(build().with_part_opacities(vec![-0.1]).is_err());
    assert!(build().with_part_opacities(vec![f64::NAN]).is_err());
    assert!(build().with_part_opacities(vec![0.5]).is_ok());
    assert!(RenderScene::empty(lights)
        .with_part_opacities(Vec::new())
        .is_ok());
}
