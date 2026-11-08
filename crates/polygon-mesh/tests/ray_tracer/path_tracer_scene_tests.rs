//! The integrator against scenes: a sun's shadow and penumbra have closed forms, paths through
//! interreflecting plates are unbiased by roulette and the bounce limit, and colored and placed
//! parts render as their colors and baked geometry say.

use std::f64::consts::{PI, TAU};

use solidmesh_math::{Color, Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene,
    ScatteringSurface, Sun,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

use crate::path_tracer_support::{
    agreement_tolerance, estimate, fine_sphere, flat_patch, scene_of, top_down_camera,
    SHARP_CREASE, SMOOTH_CREASE,
};
use crate::surface_hit_support::tree_of;

fn lambert(albedo: [f64; 3]) -> ScatteringSurface {
    ScatteringSurface::new(albedo, 0.0, 1.0, 0.0, 1.0)
}

fn surface_of(mesh: &Mesh, crease: f64) -> std::sync::Arc<solidmesh_mesh::ray_tracer::MeshSurface> {
    std::sync::Arc::new(solidmesh_mesh::ray_tracer::MeshSurface::new(mesh, crease).unwrap())
}

/// A floor and a blocking plate over it, as one scene.
fn floor_and_blocker(plate: &Mesh, sun: Sun) -> RenderScene {
    let floor = flat_patch(100.0, 0.0);
    let parts = vec![
        PlacedPart::new(
            tree_of(&floor),
            surface_of(&floor, SHARP_CREASE),
            Matrix4X4::IDENTITY,
        )
        .unwrap(),
        PlacedPart::new(
            tree_of(plate),
            surface_of(plate, SHARP_CREASE),
            Matrix4X4::IDENTITY,
        )
        .unwrap(),
    ];
    RenderScene::new(
        PlacedParts::new(parts),
        vec![lambert([1.0; 3]), lambert([1.0; 3])],
        LightSetup::sun_only(sun),
    )
    .unwrap()
}

/// A plate from `x <= 0` to -60, at height 10, its edge over the origin.
fn half_plane_plate() -> Mesh {
    let vertices = [
        -60.0, -100.0, 10.0, 0.0, -100.0, 10.0, 0.0, 100.0, 10.0, -60.0, 100.0, 10.0,
    ];
    Mesh::from_arrays(&vertices, &[0, 1, 2, 0, 2, 3])
}

/// The sun straight overhead, `radius` wide, as seen by the floor point at `x` past the edge of a
/// plate at `height`: the integral of `cos` over the directions the plate does not block, by a
/// fine midpoint rule in the disc's own angles (the plate's shadow line through the cone, exactly,
/// with no small-angle step).
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

fn floor_radiance_at(scene: &RenderScene, x: f64, samples: u32) -> (f64, f64) {
    let camera = RenderCamera::orthographic(
        Vector3::new(x, 0.0, 5.0),
        Vector3::new(x, 0.0, 0.0),
        Vector3::UNIT_Y,
        1e-3,
    )
    .unwrap();
    // One scattering event only: this test is about what the sun does directly, and the light a
    // lit floor bounces under the blocker would hide it.
    let settings = PathTraceSettings {
        max_bounces: 1,
        ..PathTraceSettings::default()
    };
    let (stats, _) = estimate(scene, camera, (1, 1), settings, samples);
    (stats[0].mean(), stats[0].standard_error())
}

/// Directly under the blocker the floor gets nothing, in the full sun it gets the unblocked
/// value, and across the shadow's edge the brightness follows the visible fraction of the sun's
/// disc, whose width is `2 * height * tan(radius)`.
#[test]
fn a_shadow_has_its_blocker_and_its_penumbra() {
    let radius = 0.1_f64;
    let height = 10.0;
    let width = height * radius.tan();
    let sun = Sun::new(Vector3::UNIT_Z, radius, [1.0; 3]).unwrap();
    let scene = floor_and_blocker(&half_plane_plate(), sun);

    let (dark, _) = floor_radiance_at(&scene, -2.0 * width, 200);
    assert_eq!(dark, 0.0, "the deep shadow took some sun");

    let (full, full_error) = floor_radiance_at(&scene, 2.0 * width, 1000);
    assert!(full > 0.0 && full_error < full * 0.01);

    // The cross-section of the penumbra against the exact integral; enough samples that a width
    // wrong by a couple of percent shows at the steep part (the 0.1% is the quadrature).
    for fraction in [-0.6, -0.3, 0.0, 0.3, 0.6] {
        let x = fraction * width;
        let (mean, error) = floor_radiance_at(&scene, x, 120_000);
        let expected = visible_cosine_integral(x, height, radius) / PI;
        let allowed = 5.0 * error + full * 1e-3;
        assert!(
            (mean - expected).abs() <= allowed,
            "at {fraction}: {mean} against {expected} (allowed {allowed})"
        );
    }
    // And the fully lit floor is that same integral over the whole disc.
    assert!(
        (full - visible_cosine_integral(1e6, height, radius) / PI).abs()
            < 5.0 * full_error + full * 1e-3
    );
}

/// Two facing white plates in a furnace: paths bounce between them many times, so a roulette
/// that forgets to reweight, or a bounce limit that cuts deep, loses visible energy. The plates
/// have no analytic answer at a finite size; instead the unbiased result without roulette, with
/// a deep bounce limit, is the reference the shallow-and-roulette configuration must reach.
#[test]
fn interreflection_converges_the_same_with_and_without_roulette() {
    let bottom = flat_patch(30.0, 0.0);
    let top = {
        let z = 6.0;
        let vertices = [
            -30.0, -30.0, z, -30.0, 30.0, z, 30.0, 30.0, z, 30.0, -30.0, z,
        ];
        Mesh::from_arrays(&vertices, &[0, 1, 2, 0, 2, 3])
    };
    let parts = vec![
        PlacedPart::new(
            tree_of(&bottom),
            surface_of(&bottom, SHARP_CREASE),
            Matrix4X4::IDENTITY,
        )
        .unwrap(),
        PlacedPart::new(
            tree_of(&top),
            surface_of(&top, SHARP_CREASE),
            Matrix4X4::IDENTITY,
        )
        .unwrap(),
    ];
    let albedo = 0.7;
    let scene = RenderScene::new(
        PlacedParts::new(parts),
        vec![lambert([albedo; 3]), lambert([albedo; 3])],
        LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap()),
    )
    .unwrap();
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 3.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        4.0,
    )
    .unwrap();

    let run = |settings, samples| estimate(&scene, camera, (4, 4), settings, samples).0[0];
    let reference = run(
        PathTraceSettings {
            russian_roulette_after: None,
            max_bounces: 64,
            ..PathTraceSettings::default()
        },
        4000,
    );
    let rouletted = run(
        PathTraceSettings {
            russian_roulette_after: Some(1),
            max_bounces: 64,
            ..PathTraceSettings::default()
        },
        4000,
    );
    let shallow = run(
        PathTraceSettings {
            russian_roulette_after: None,
            max_bounces: 2,
            ..PathTraceSettings::default()
        },
        4000,
    );
    println!(
        "plates: reference {} +- {} roulette {} +- {} shallow {}",
        reference.mean(),
        reference.standard_error(),
        rouletted.mean(),
        rouletted.standard_error(),
        shallow.mean()
    );
    let tolerance = agreement_tolerance(&reference, &rouletted);
    assert!(
        (reference.mean() - rouletted.mean()).abs() <= tolerance,
        "{} against {}",
        reference.mean(),
        rouletted.mean()
    );
    // Interreflection is real here: the limit of two bounces is clearly below the deep answer,
    // so a bias that left the deep paths out would have shown up above.
    assert!(reference.mean() > shallow.mean() * 1.5);
    // The first bounce alone can do no better than a floor that sees only the sky and no worse
    // than one that sees only the other plate's unlit back, so these bracket the answer.
    assert!(reference.mean() > 0.0 && reference.mean() < albedo);
}

/// A part's color multiplies what it reflects: in a white furnace a convex Lambert sphere
/// returns exactly its albedo in each channel.
#[test]
fn a_parts_color_multiplies_the_light() {
    let albedo = [0.8, 0.4, 0.2];
    let scene = scene_of(
        &fine_sphere(10.0),
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        lambert(albedo),
        LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap()),
    );
    let (stats, _) = estimate(
        &scene,
        top_down_camera(40.0, 15.0),
        (4, 4),
        PathTraceSettings::default(),
        64,
    );
    for (channel, stat) in stats.iter().enumerate() {
        assert!(
            (stat.mean() - albedo[channel]).abs() <= 5.0 * stat.standard_error() + 0.002,
            "channel {channel}: {} against {}",
            stat.mean(),
            albedo[channel]
        );
    }
}

/// A face's stored 8-bit color is decoded from sRGB and multiplied in: the same result as a part
/// whose base color is that color's linear value.
#[test]
fn a_face_color_multiplies_the_light_after_decoding() {
    let mut cube = create_cube(20.0, 20.0, 20.0);
    let color = Color::rgba(128, 64, 255, 255);
    cube.face_colors = Some(vec![color; cube.faces.len()]);
    let lights = LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap());
    let scene = scene_of(
        &cube,
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        lambert([0.5, 1.0, 1.0]),
        lights,
    );
    let decode = |code: f64| -> f64 {
        let value = code / 255.0;
        if value <= 0.04045 {
            value / 12.92
        } else {
            ((value + 0.055) / 1.055).powf(2.4)
        }
    };
    let expected = [0.5 * decode(128.0), decode(64.0), 1.0];
    let (stats, _) = estimate(
        &scene,
        top_down_camera(40.0, 4.0),
        (2, 2),
        PathTraceSettings::default(),
        64,
    );
    for (channel, stat) in stats.iter().enumerate() {
        assert!(
            (stat.mean() - expected[channel]).abs() <= 5.0 * stat.standard_error() + 1e-3,
            "channel {channel}: {} against {}",
            stat.mean(),
            expected[channel]
        );
    }
}

/// A part placed by a matrix (moved, scaled unevenly, mirrored) renders the same as the same
/// geometry baked into the mesh: the matrices reach the normals, the offsets and the hits.
#[test]
fn placed_scaled_and_mirrored_parts_render_like_their_baked_geometry() {
    let matrix =
        Matrix4X4::create_scale(-1.2, 0.8, 1.5) * Matrix4X4::create_translation(2.0, -3.0, 5.0);
    let sphere = fine_sphere(10.0);
    let vertices: Vec<f64> = sphere
        .vertices
        .iter()
        .flat_map(|vertex| {
            let world = Vector3::new(
                f64::from(vertex.x),
                f64::from(vertex.y),
                f64::from(vertex.z),
            )
            .transform_position(matrix);
            [world.x, world.y, world.z]
        })
        .collect();
    // A mirror flips the winding, so the baked copy swaps it back to stay outward.
    let faces: Vec<i32> = sphere
        .faces
        .iter()
        .flat_map(|face| [face.v0, face.v2, face.v1])
        .collect();
    let baked = Mesh::from_arrays(&vertices, &faces);

    let surface = ScatteringSurface::new([0.7, 0.5, 0.3], 0.0, 0.4, 0.0, 1.5);
    let lights = LightSetup {
        environment: Environment::gradient([0.2, 0.3, 0.8], [0.9, 0.9, 0.8], [0.1, 0.1, 0.1])
            .unwrap(),
        sun: Some(Sun::new(Vector3::new(0.4, 0.2, 0.8), 0.15, [8.0, 7.5, 6.5]).unwrap()),
    };
    let placed = scene_of(&sphere, SMOOTH_CREASE, matrix, surface, lights);
    let baked = scene_of(&baked, SMOOTH_CREASE, Matrix4X4::IDENTITY, surface, lights);
    let camera = RenderCamera::perspective(
        Vector3::new(30.0, -40.0, 30.0),
        Vector3::new(2.0, -3.0, 5.0),
        Vector3::UNIT_Z,
        35.0,
    )
    .unwrap();
    let (placed_stats, _) = estimate(&placed, camera, (8, 8), PathTraceSettings::default(), 64);
    let (baked_stats, _) = estimate(&baked, camera, (8, 8), PathTraceSettings::default(), 64);
    for channel in 0..3 {
        let tolerance = agreement_tolerance(&placed_stats[channel], &baked_stats[channel]) + 2e-3;
        assert!(
            (placed_stats[channel].mean() - baked_stats[channel].mean()).abs() <= tolerance,
            "channel {channel}: placed {} against baked {} (tolerance {tolerance})",
            placed_stats[channel].mean(),
            baked_stats[channel].mean()
        );
    }
}

/// A cube is the first test a triangle mesh sees: rendered head on, a white Lambert cube in a
/// furnace is a flat white square against the furnace.
#[test]
fn a_cube_in_a_furnace_is_invisible_too() {
    let scene = scene_of(
        &create_cube(20.0, 20.0, 20.0),
        SHARP_CREASE,
        Matrix4X4::IDENTITY,
        lambert([1.0; 3]),
        LightSetup::environment_only(Environment::constant([0.6; 3]).unwrap()),
    );
    let (stats, _) = estimate(
        &scene,
        top_down_camera(40.0, 30.0),
        (6, 6),
        PathTraceSettings::default(),
        32,
    );
    assert!((stats[0].mean() - 0.6).abs() <= 1e-9, "{}", stats[0].mean());
}

/// A closed, coarsely faceted sphere with smooth shading in a furnace and sun: from the inside no
/// light may enter, however bent the shading normal is against its triangle. A direction fine by
/// the smooth normal but heading out through the triangle must be dropped, not started on the
/// far side of the wall.
#[test]
fn a_sealed_cavity_lets_no_light_in() {
    let lights = LightSetup {
        environment: Environment::constant([1.0; 3]).unwrap(),
        sun: Some(Sun::new(Vector3::new(0.3, 0.4, 0.8), 0.2, [20.0; 3]).unwrap()),
    };
    // Eight segments and four rings: faces wide enough that smooth normals are far from flat.
    let scene = scene_of(
        &crate::surface_hit_support::sphere(10.0, Vector3::ZERO, 8, 4),
        SMOOTH_CREASE,
        Matrix4X4::IDENTITY,
        lambert([1.0; 3]),
        lights,
    );
    let camera = RenderCamera::perspective(
        Vector3::new(0.3, -0.2, 0.1),
        Vector3::new(4.0, 2.0, 1.0),
        Vector3::UNIT_Z,
        80.0,
    )
    .unwrap();
    let (stats, _) = estimate(&scene, camera, (8, 8), PathTraceSettings::default(), 16);
    for (channel, stat) in stats.iter().enumerate() {
        assert_eq!(
            stat.mean(),
            0.0,
            "light leaked into the cavity on channel {channel}"
        );
    }
}
