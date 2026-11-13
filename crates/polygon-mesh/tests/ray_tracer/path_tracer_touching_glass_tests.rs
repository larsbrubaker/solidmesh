//! Glass parts that touch: two pieces of the same glass lying face on face are one piece of glass
//! to light, so a path crossing from one into the other neither bends, nor reflects, nor changes
//! what it carries. Each scene of separate parts is checked against the same shape unioned into
//! one solid, in a white furnace (where no sample may carry more than the light it started in) and
//! under a sky, with the shared faces covering each other whole or in part; two touching panes are
//! checked against the closed form of one pane their combined thickness; and a single glass
//! solid, alone and on a floor, renders pinned bits, those of glass touching no other glass, and
//! under `SunThroughGlass::Exact` the bits it had before the sun could go through glass. Glass
//! parts that overlap in volume are not one piece of glass (their inner faces show);
//! `path_tracer_glass_contact_tests` pins them.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, MeshSurface, PathTraceSettings, PathTracer, PlacedPart, PlacedParts,
    RenderCamera, RenderScene, RenderWork, ScatteringSurface, Sun, SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::glass_tray_support::{
    box_meshes, part_of, parts_scene, rig_lights, tray_meshes, unioned,
};
use crate::path_tracer_golden_tests::{render_digest, render_digest_with};
use crate::path_tracer_support::{estimate, flat_patch, SHARP_CREASE};
use crate::scattering_support::Stats;
use crate::surface_hit_support::tree_of;

/// How far above 1 a furnace sample may come out from rounding alone.
const FURNACE_SLACK: f64 = 1e-9;

fn furnace() -> LightSetup {
    LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap())
}

fn no_roulette() -> PathTraceSettings {
    PathTraceSettings {
        russian_roulette_after: None,
        ..PathTraceSettings::default()
    }
}

/// Every sample of a `size` image, `samples` a pixel.
fn every_sample(
    scene: &RenderScene,
    camera: RenderCamera,
    size: (u32, u32),
    settings: PathTraceSettings,
    samples: u32,
) -> Vec<[f64; 3]> {
    let tracer = PathTracer::new(scene, camera, size.0, size.1, settings).unwrap();
    let mut work = RenderWork::default();
    let mut all = Vec::new();
    for y in 0..size.1 {
        for x in 0..size.0 {
            for sample in 0..samples {
                all.push(tracer.sample_pixel(x, y, sample, &mut work).unwrap());
            }
        }
    }
    all
}

/// The brightest channel of any sample.
fn brightest(samples: &[[f64; 3]]) -> f64 {
    samples
        .iter()
        .flat_map(|c| c.iter().copied())
        .fold(0.0, f64::max)
}

/// A camera looking down across the x = 0 plane from the -x, -y, +z side, so its rays enter the
/// part on the -x side and many cross into the one on the +x side.
fn oblique_camera() -> RenderCamera {
    RenderCamera::orthographic(
        Vector3::new(-20.0, -20.0, 20.0),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        14.0,
    )
    .unwrap()
}

/// Checks the boxes as separate parts against their union: in a furnace no sample's channel is
/// over 1, and in the furnace and under the sky the mean matches the union's.
fn check_against_union(boxes: &[([f64; 3], [f64; 3])], what: &str) {
    let glass = ScatteringSurface::glass();
    let pieces = box_meshes(boxes);
    let solid = vec![unioned(&pieces)];
    let size = (16, 16);
    let separate = every_sample(
        &parts_scene(&pieces, glass, furnace()),
        oblique_camera(),
        size,
        no_roulette(),
        8,
    );
    let most = brightest(&separate);
    assert!(
        most <= 1.0 + FURNACE_SLACK,
        "{what}: a furnace sample came out at {most}, above the light it started in"
    );
    for (lights, light_name) in [(furnace(), "furnace"), (rig_lights(), "sun and sky")] {
        let (parts, _) = estimate(
            &parts_scene(&pieces, glass, lights),
            oblique_camera(),
            size,
            no_roulette(),
            16,
        );
        let (one, _) = estimate(
            &parts_scene(&solid, glass, lights),
            oblique_camera(),
            size,
            no_roulette(),
            16,
        );
        for channel in 0..3 {
            let tolerance =
                crate::path_tracer_support::agreement_tolerance(&parts[channel], &one[channel]);
            let (a, b) = (parts[channel].mean(), one[channel].mean());
            assert!(
                (a - b).abs() <= tolerance,
                "{what}, {light_name}, channel {channel}: separate parts {a} against one solid \
                 {b}, tolerance {tolerance}"
            );
        }
    }
}

#[test]
fn glass_boxes_sharing_a_face_light_as_one_box() {
    check_against_union(
        &[
            ([10.0, 10.0, 10.0], [-5.0, 0.0, 0.0]),
            ([10.0, 10.0, 10.0], [5.0, 0.0, 0.0]),
        ],
        "two boxes sharing a face",
    );
}

/// The shared faces only partly cover each other: where they meet there is no interface, and
/// where one runs past the other it meets air.
#[test]
fn glass_boxes_sharing_part_of_a_face_light_as_their_union() {
    check_against_union(
        &[
            ([10.0, 10.0, 10.0], [-5.0, 0.0, 0.0]),
            ([10.0, 10.0, 10.0], [5.0, 5.0, 0.0]),
        ],
        "two boxes sharing half a face",
    );
}

/// MatterCAD's typical view of the tray.
fn tray_camera() -> RenderCamera {
    RenderCamera::perspective(
        Vector3::new(60.0, -140.0, 150.0),
        Vector3::new(0.0, 0.0, 4.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap()
}

/// The tray of touching boxes seen from MatterCAD's view gains nothing in a furnace.
#[test]
fn a_tray_of_touching_glass_boxes_gains_nothing_in_a_furnace() {
    let scene = parts_scene(&tray_meshes(false), ScatteringSurface::glass(), furnace());
    let samples = every_sample(&scene, tray_camera(), (40, 30), no_roulette(), 4);
    let most = brightest(&samples);
    assert!(
        most <= 1.0 + FURNACE_SLACK,
        "a furnace sample through the touching tray came out at {most}"
    );
}

/// Touching glasses of different index are a real interface, lit as two steps through a gap of
/// air too thin to see: in the tray with its pieces alternating between index 1.5 and 1.9, a
/// furnace sample still carries no more than the light it started in.
#[test]
fn touching_glasses_of_different_index_gain_nothing_in_a_furnace() {
    let pieces = tray_meshes(false);
    let parts = pieces
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let surfaces = (0..pieces.len())
        .map(|at| ScatteringSurface::new([1.0; 3], 0.0, 0.0, 1.0, [1.5, 1.9][at % 2]))
        .collect();
    let scene = RenderScene::new(PlacedParts::new(parts), surfaces, furnace()).unwrap();
    let samples = every_sample(&scene, tray_camera(), (40, 30), no_roulette(), 4);
    let most = brightest(&samples);
    assert!(
        most <= 1.0 + FURNACE_SLACK,
        "a furnace sample through glasses of index 1.5 and 1.9 came out at {most}"
    );
}

/// Fresnel reflectance of unpolarized light going from index 1 into index `ior` at incidence
/// cosine `cos_i`.
fn fresnel_into(cos_i: f64, ior: f64) -> f64 {
    let sin2_t = (1.0 - cos_i * cos_i) / (ior * ior);
    let cos_t = (1.0 - sin2_t).sqrt();
    let perpendicular = (cos_i - ior * cos_t) / (cos_i + ior * cos_t);
    let parallel = (ior * cos_i - cos_t) / (ior * cos_i + cos_t);
    0.5 * (parallel * parallel + perpendicular * perpendicular)
}

/// Two glass panes 50 mm square, of (thickness, center height), seen at 45 degrees with a small
/// sun straight along the camera's rays behind them and nothing else lit: a sample reaches the
/// sun only if its ray left the panes along the line it came in on. Every sample is the sun's
/// whole radiance or nothing, and their mean is what one pane of their combined extent passes:
/// only its two outer faces reflect, so with every internal reflection counted it passes
/// `(1 - F) / (1 + F)`, whatever its thickness.
fn check_panes_pass_as_one(panes: &[(f64, f64)], what: &str) {
    let toward = Vector3::new(0.0, 1.0, -1.0).get_normal();
    let lights = LightSetup {
        environment: Environment::constant([0.0; 3]).unwrap(),
        sun: Some(Sun::new(toward, 0.02, [1.0; 3]).unwrap()),
    };
    let boxes: Vec<_> = panes
        .iter()
        .map(|&(thickness, center)| ([50.0, 50.0, thickness], [0.0, 0.0, center]))
        .collect();
    let scene = parts_scene(&box_meshes(&boxes), ScatteringSurface::glass(), lights);
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -20.0, 20.0),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        2.0,
    )
    .unwrap();
    let samples = every_sample(&scene, camera, (4, 4), no_roulette(), 1024);
    let mut stats = Stats::default();
    for sample in &samples {
        for channel in sample {
            assert!(
                *channel == 0.0 || (channel - 1.0).abs() <= FURNACE_SLACK,
                "{what}: a sample carried {channel}, so its ray left bent or with energy changed"
            );
        }
        stats.add(sample[0]);
    }
    let fresnel = fresnel_into(1.0 / 2.0_f64.sqrt(), 1.5);
    let expected = (1.0 - fresnel) / (1.0 + fresnel);
    let tolerance = 5.0 * stats.standard_error() + 1e-9;
    assert!(
        (stats.mean() - expected).abs() <= tolerance,
        "{what}: passed {} where one pane passes {expected} (tolerance {tolerance})",
        stats.mean()
    );
}

#[test]
fn two_touching_panes_pass_light_straight_through_as_one_pane() {
    check_panes_pass_as_one(&[(1.0, -0.5), (1.0, -1.5)], "touching panes");
}

/// A glass cube over a floor, as the golden opaque scene has its shiny cube, under `lights`.
fn glass_cube_on_floor(lights: LightSetup) -> RenderScene {
    let floor = flat_patch(30.0, 0.0);
    let cube = create_cube(10.0, 10.0, 10.0);
    let parts = vec![
        part_of(&floor, Matrix4X4::IDENTITY),
        part_of(&cube, Matrix4X4::create_translation(0.0, 0.0, 7.0)),
    ];
    RenderScene::new(
        PlacedParts::new(parts),
        vec![
            ScatteringSurface::new([0.6, 0.55, 0.5], 0.0, 1.0, 0.0, 1.0),
            ScatteringSurface::glass(),
        ],
        lights,
    )
    .unwrap()
}

/// The unioned tray standing on a floor it shares its bottom face with.
fn one_solid_tray_on_floor(lights: LightSetup) -> RenderScene {
    let floor = flat_patch(2000.0, 0.0);
    let tray = unioned(&box_meshes(&crate::glass_tray_support::tray_boxes()));
    let parts = vec![
        PlacedPart::new(
            tree_of(&floor),
            std::sync::Arc::new(MeshSurface::new(&floor, SHARP_CREASE).unwrap()),
            Matrix4X4::IDENTITY,
        )
        .unwrap(),
        part_of(&tray, Matrix4X4::IDENTITY),
    ];
    RenderScene::new(
        PlacedParts::new(parts),
        vec![
            ScatteringSurface::new([0.3; 3], 0.0, 1.0, 0.0, 1.0),
            ScatteringSurface::glass(),
        ],
        lights,
    )
    .unwrap()
}

/// Pinned with one glass solid in the scene, where no path ever crosses from glass into glass, so
/// glass touching glass plays no part in these bits. The one-solid tray rests on the floor, so its
/// bits are those of glass leaving through its own surface onto the floor (`glass_rest`); the cube
/// stands 2 above the floor. Under a sun both are lit by the sun going straight through the glass
/// and the exact transport through it at the floor's own weight (`SunThroughGlass::Straight`); the
/// furnace has no sun, and its bits are the ones it always had.
const GLASS_CUBE_SUN_AND_SKY_DIGEST: u64 = 0xb4f1_e307_451a_ddd3;
const GLASS_CUBE_FURNACE_DIGEST: u64 = 0xa4b4_ca5d_bcff_8912;
const ONE_SOLID_TRAY_DIGEST: u64 = 0xe4cb_51fd_2d75_2b91;

/// The cube under the sun and the tray under `SunThroughGlass::Exact`, where shadow rays stop at
/// glass: the bits they had before the sun could go through glass, which Exact keeps.
const EXACT_GLASS_CUBE_SUN_AND_SKY_DIGEST: u64 = 0x18ec_7fb1_377a_dc7b;
const EXACT_ONE_SOLID_TRAY_DIGEST: u64 = 0xc022_6d9f_48b0_2518;

#[test]
fn a_single_glass_solid_renders_as_it_always_did() {
    let golden_lights = LightSetup {
        environment: Environment::gradient([0.3, 0.4, 0.9], [0.9, 0.85, 0.8], [0.15, 0.1, 0.1])
            .unwrap(),
        sun: Some(Sun::new(Vector3::new(0.5, 0.3, 0.8), 0.1, [6.0, 5.5, 5.0]).unwrap()),
    };
    let digests = [
        render_digest(&glass_cube_on_floor(golden_lights)),
        render_digest(&glass_cube_on_floor(furnace())),
        render_digest(&one_solid_tray_on_floor(rig_lights())),
    ];
    println!("glass digests {digests:#018x?}");
    let exact = [
        render_digest_with(&glass_cube_on_floor(golden_lights), SunThroughGlass::Exact),
        render_digest_with(
            &one_solid_tray_on_floor(rig_lights()),
            SunThroughGlass::Exact,
        ),
    ];
    assert_eq!(
        exact,
        [
            EXACT_GLASS_CUBE_SUN_AND_SKY_DIGEST,
            EXACT_ONE_SOLID_TRAY_DIGEST
        ],
        "got {exact:#018x?} exactly"
    );
    assert_eq!(
        digests,
        [
            GLASS_CUBE_SUN_AND_SKY_DIGEST,
            GLASS_CUBE_FURNACE_DIGEST,
            ONE_SOLID_TRAY_DIGEST
        ],
        "got {digests:#018x?}"
    );
}
